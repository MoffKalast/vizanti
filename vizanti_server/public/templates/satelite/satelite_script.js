let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let navsatModule = await import(`${base_url}/js/modules/navsat.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let view = viewModule.view;
let tf = tfModule.tf;
let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let navsat = navsatModule.navsat;
let Status = StatusModule.Status;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let copyright = "© OpenStreetMap";
let topic = getTopic("{uniqueID}");
let server_url = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
let listener = undefined;
let zoomLevel = 12;

let map_topic = undefined;
let map_fix = undefined;
let fix_data = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const tileServerString = document.getElementById('{uniqueID}_tileserver');
const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');
const smoothingCheckbox = document.getElementById('{uniqueID}_smoothing');
const ignoreRotationCheckbox = document.getElementById('{uniqueID}_ignore_rotation');

const text_lat = document.getElementById("{uniqueID}_latitude");
const text_lon = document.getElementById("{uniqueID}_longitude");
const text_alt = document.getElementById("{uniqueID}_altitude");
const text_cov = document.getElementById("{uniqueID}_covariance");

const placeholder = new Image();
placeholder.src = "assets/tile_loading.png";

function setOpacityText(val){
	if(val == 0.0)
		opacityValue.textContent = "0.0 (Tile rendering disabled)";
	else
		opacityValue.textContent = val;
}

opacitySlider.addEventListener('input', function () {
	setOpacityText(this.value);
	saveSettings();
});

smoothingCheckbox.addEventListener('change', saveSettings);
ignoreRotationCheckbox.addEventListener('change', saveSettings);

tileServerString.addEventListener('input', function () {
	server_url = this.value;

	if(server_url.includes("tile.openstreetmap.org"))
		copyright = "© OpenStreetMap";
	else
		copyright = "";

	saveSettings();
});

//dirty hack to show entire datalist when dropdown is clicked
let tileServer_prev = '';
tileServerString.addEventListener('focus', function () {
    if (this.value.trim() !== '') {
        tileServer_prev = this.value; // store for later
        this.setAttribute('placeholder', this.value);
    }
    this.value = '';
});
tileServerString.addEventListener('blur', function () {
    if (this.value.trim() === '') {
        this.value = tileServer_prev;
    }
});

let drawPending = false;
function scheduleDraw() {
    if (!drawPending) {
        drawPending = true;
        requestAnimationFrame(() => {
            drawPending = false;
            drawTiles();
        });
    }
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });
ctx.clip = function(){};

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	server_url = loaded_data.server_url;

	if(server_url.includes("tile.openstreetmap.org"))
		copyright = "© OpenStreetMap";
	else
		copyright = "";

	tileServerString.value = server_url;

	smoothingCheckbox.checked = loaded_data.smoothing;
	ignoreRotationCheckbox.checked = loaded_data.ignore_rotation ?? false;

	opacitySlider.value = loaded_data.opacity;
	setOpacityText(loaded_data.opacity);
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		server_url: server_url,
		opacity: opacitySlider.value,
		smoothing: smoothingCheckbox.checked,
		ignore_rotation: ignoreRotationCheckbox.checked
	}
	settings.save();
}

function findParentTile(x, y, z, maxLevelsUp = 4) {
	for (let dz = 1; dz <= maxLevelsUp && (z - dz) >= 0; dz++) {
		const parentX = x >> dz;
		const parentY = y >> dz;
		const parentURL = server_url.replace("{z}", z - dz).replace("{x}", parentX).replace("{y}", parentY);
		const parentImage = navsat.live_cache[parentURL];
		if (parentImage && parentImage.complete) {
			const regionSize = navsat.tile_size >> dz; // size of our tile's region within the parent
			const srcX = (x % (1 << dz)) * regionSize;
			const srcY = (y % (1 << dz)) * regionSize;
			return { image: parentImage, srcX, srcY, srcSize: regionSize };
		}
	}
	return null;
}

function drawTile(screenSize, i, j, tempMeterSize, tempZoomLevel, maxtile) {
	const x = (fix_data.tilePos.x + i + maxtile + 1) % (maxtile + 1);
	const y = (fix_data.tilePos.y + j + maxtile + 1) % (maxtile + 1);

	const offsetX = fix_data.offset.x - i * tempMeterSize;
	const offsetY = fix_data.offset.y - j * tempMeterSize;

	const tileURL = server_url.replace("{z}", tempZoomLevel).replace("{x}", x).replace("{y}", y);
	let tileImage = navsat.live_cache[tileURL];
	let parentCrop = null;

	if (!tileImage || !tileImage.complete) {
		navsat.enqueue(tileURL);
		parentCrop = findParentTile(x, y, tempZoomLevel);
		if (!parentCrop)
			tileImage = placeholder;
	}
	let transformed;
	if (!ignoreRotationCheckbox.checked) {
		transformed = tf.transformPoseStamped(map_fix.header, {x: -offsetX, y: offsetY, z: 0}, new Quaternion());
	} else {
		transformed = tf.transformPoseStamped(map_fix.header, {x: 0, y: 0, z: 0}, new Quaternion());
		transformed.translation.x -= offsetX;
		transformed.translation.y += offsetY;
		transformed.rotation = new Quaternion();
	}

	const pos = view.fixedToScreen({x: transformed.translation.x, y: transformed.translation.y});
	const matrix = view.quaterionToProjectionMatrix(transformed.rotation);
	ctx.setTransform(matrix[0], matrix[1], matrix[2], matrix[3], pos.x, pos.y);

	if (parentCrop){
		ctx.globalAlpha = opacitySlider.value * 0.8;
		ctx.drawImage(parentCrop.image, parentCrop.srcX, parentCrop.srcY, parentCrop.srcSize, parentCrop.srcSize, 0, 0, screenSize, screenSize);
		ctx.globalAlpha = opacitySlider.value;
	}
	else
		ctx.drawImage(tileImage, 0, 0, screenSize, screenSize);
}

function clamp(val, from, to){
    if(val > to)
        return to;
    if(val < from)
        return from;
    return val;
}

//Rendering
async function drawTiles(){

	const wid = canvas.width;
    const hei = canvas.height;

	ctx.clearRect(0, 0, wid, hei);
	ctx.globalAlpha = opacitySlider.value;
	ctx.imageSmoothingEnabled = smoothingCheckbox.checked;

	if(!map_fix){
		return;
	}

	if(opacitySlider.value == 0.0){
		status.setOK();
		return;
	}

	const frame = tf.getAbsoluteTransform(map_fix.header);

	let	tempZoomLevel = Math.round(Math.log2(view.scale)+17);
	tempZoomLevel = clamp(tempZoomLevel, 7, 19);
	if(tempZoomLevel != zoomLevel){
		navsat.clear_queue();
		zoomLevel = tempZoomLevel;
		updateFixData();
	}


	if(frame){

		let metersSize = navsat.tileSizeInMeters(map_fix.latitude, tempZoomLevel)
		const tileScreenSize = view.getMapUnitsInPixels(metersSize);
		const corners = [
			{ x: 0, y: 0, z: 0 },
			{ x: wid, y: 0, z: 0 },
			{ x: wid, y: hei, z: 0  },
			{ x: 0, y: hei, z: 0  },
		];

		// Convert the corners from pixels to meters, transform them to map_fix frame and convert to latitude, longitude
		const cornerCoords = corners.map((corner) => {
			const meters = view.screenToFixed(corner);

			let transformed;
			if(ignoreRotationCheckbox.checked){
				transformed = {
					translation: {
						x: -frame.translation.x + meters.x,
						y: -frame.translation.y + meters.y
					}
				}
			}else{
				transformed = tf.transformPose(
					tf.fixed_frame,
					map_fix.header.frame_id,
					meters,
					new Quaternion()
				);
			}
			return {
				latitude: map_fix.latitude + (transformed.translation.y * fix_data.degreesPerMeter.latitude),
				longitude: map_fix.longitude + (transformed.translation.x * fix_data.degreesPerMeter.longitude)
			};
		});

		// Convert the corners to tile coordinates
		const cornerTileCoords = cornerCoords.map((coord) =>
			navsat.coordToTile(coord.longitude, coord.latitude, tempZoomLevel)
		);

		// Calculate the range of tiles to cover the screen
		const minX = Math.min(...cornerTileCoords.map((coord) => coord.x)) - fix_data.tilePos.x - 1;
		const maxX = Math.max(...cornerTileCoords.map((coord) => coord.x)) - fix_data.tilePos.x + 1;
		const minY = Math.min(...cornerTileCoords.map((coord) => coord.y)) - fix_data.tilePos.y - 1;
		const maxY = Math.max(...cornerTileCoords.map((coord) => coord.y)) - fix_data.tilePos.y + 1;

		//draw tiles in concentric circles, starting from the center of the screen
		const matrixWidth = (maxX - minX)+2;
		const matrixHeight = (maxY - minY)+2;
		const centerX = Math.round((maxX+minX)/2);
		const centerY = Math.round((maxY+minY)/2)-1;
		const maxtile = Math.pow(2, tempZoomLevel) - 1;

		let x = 0;
		let y = 0;
		let dx = 0;
		let dy = -1;

		const maxDimension = Math.max(matrixWidth, matrixHeight);
		for (let i = 0; i < maxDimension ** 2; i++) {
			if (-matrixWidth / 2 < x && x <= matrixWidth / 2 && -matrixHeight / 2 < y && y <= matrixHeight / 2) {
				drawTile(tileScreenSize, centerX+x, centerY+y, metersSize, tempZoomLevel, maxtile);
			}
			if (x === y || (x < 0 && x === -y) || (x > 0 && x === 1 - y)) {
				[dx, dy] = [-dy, dx];
			}
			x += dx;
			y += dy;
		}

		//transform reset
		ctx.setTransform(1, 0, 0, 1, 0, 0);

		ctx.globalAlpha = 0.6;
		ctx.fillStyle = "#171717";
		ctx.fillRect(0, hei-20, 120, 20);

		ctx.globalAlpha = 1.0;
		ctx.font = "12px Monospace";
		ctx.fillStyle = "white";
		ctx.fillText(copyright, 5, hei-5);

		status.setOK();
	}else{
		status.setError("Required transform frame \""+map_fix.header.frame_id+"\" not found.");
	}
}

const COVARIANCE_TYPE = {
	0: "(unknown)",
	1: "(approximated)",
	2: "(diagonal known)",
	3: "(known)"
}

//Topic
function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(map_topic !== undefined){
		map_topic.unsubscribe(listener);
	}

	map_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/msg/NavSatFix',
		throttle_rate: 33
	});

	status.setWarn("No data received.");
	text_lat.innerText = "Latitude: ?";
	text_lon.innerText = "Longitude: ?";
	text_alt.innerText = "Altitude: ?";
	text_cov.innerText = "Ground Covariance: ?";
	
	listener = map_topic.subscribe((msg) => {

		const cov_mat = msg.position_covariance;
		const covariance_meters = Math.hypot(Math.sqrt(cov_mat[0]), Math.sqrt(cov_mat[4]))

		if(msg.latitude != null)
			text_lat.innerText = "Latitude: " + msg.latitude.toFixed(8)+"°";

		if(msg.longitude != null)
			text_lon.innerText = "Longitude: " + msg.longitude.toFixed(8)+"°";

		if(msg.altitude != null)
			text_alt.innerText = "Altitude: " + msg.altitude.toFixed(2)+" m";

		text_cov.innerText = "Ground Covariance: " + covariance_meters.toFixed(2)+ " m " + COVARIANCE_TYPE[msg.position_covariance_type];

		if(msg.status.status == -1 || isNaN(msg.longitude) || isNaN(msg.latitude)){
			status.setWarn("No fix.");
			return;
		}

		map_fix = msg;
		updateFixData();
		drawTiles();
	});

	saveSettings();
}

function updateFixData(){
	const tilePos = navsat.coordToTile(map_fix.longitude, map_fix.latitude, zoomLevel);
	const tileCoords = navsat.tileToCoord(tilePos.x, tilePos.y, zoomLevel);
	const nextTileCoords = navsat.tileToCoord(tilePos.x+1, tilePos.y+1, zoomLevel);
	const metersSize = navsat.tileSizeInMeters(map_fix.latitude, zoomLevel);

	fix_data = {
		tilePos: tilePos,
		tileCoords: tileCoords,
		offset:{
			x: navsat.haversine(map_fix.latitude, tileCoords.longitude, map_fix.latitude, map_fix.longitude),
			y: navsat.haversine(tileCoords.latitude, map_fix.longitude, map_fix.latitude, map_fix.longitude)
		},
		metersSize: metersSize,
		degreesPerMeter: {
			longitude: Math.abs(tileCoords.longitude - nextTileCoords.longitude)/metersSize,
			latitude: Math.abs(tileCoords.latitude - nextTileCoords.latitude)/metersSize
		}
	}
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/msg/NavSatFix");

	let topiclist = "";
	result.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+"</option>"
	});
	selectionbox.innerHTML = topiclist

	if(topic == "")
		topic = selectionbox.value;
	else{
		if(result.includes(topic)){
			selectionbox.value = topic;
		}else{
			topiclist += "<option value='"+topic+"'>"+topic+"</option>"
			selectionbox.innerHTML = topiclist
			selectionbox.value = topic;
		}
	}
	connect();
}

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	scheduleDraw();
}

window.addEventListener("navsat_tilecache_updated", scheduleDraw);
window.addEventListener("tf_fixed_frame_changed", scheduleDraw);
window.addEventListener("tf_changed", ()=>{
	if(map_fix && map_fix.header.frame_id != tf.fixed_frame){
		scheduleDraw();
	}
});

window.addEventListener("view_changed", scheduleDraw);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

document.getElementById("{uniqueID}_export_DB").addEventListener("click", async (event) =>{

	let filename = await prompt("Enter file name for tile DB export (.json will be appended automatically):", "navsat_tile_db");
	if (filename != null) {
		navsatModule.exportDatabase(filename+'.json');
	}
});

document.getElementById("{uniqueID}_import_DB").addEventListener("click", (event) =>{

	const input = document.createElement('input');
	input.type = 'file';
	input.accept = '.json';

	input.onchange = (event) => {
		const file = event.target.files[0];
		const reader = new FileReader();
		reader.onload = () => {
			try {
				navsatModule.importDatabase(reader.result);
			} catch (error) {
				console.error('Error importing DB file:', error);
			}
		};

		reader.readAsText(file);
	};

	input.click();
});

resizeScreen();

console.log("Satelite Widget Loaded {uniqueID}")