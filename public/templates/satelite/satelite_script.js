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
let zoomLevel = 19;

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

const placeholder = new Image();
placeholder.src = "assets/tile_loading.png";

opacitySlider.addEventListener('input', function () {
	opacityValue.textContent = this.value;
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

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	server_url = loaded_data.server_url;

	if(server_url.includes("tile.openstreetmap.org"))
		copyright = "© OpenStreetMap";
	else
		copyright = "";

	smoothingCheckbox.checked = loaded_data.smoothing;
	ignoreRotationCheckbox.checked = loaded_data.ignore_rotation ?? false;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
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

function drawTile(screenSize, i, j){

	const x = fix_data.tilePos.x + i;
	const y = fix_data.tilePos.y + j;

	const offsetX = fix_data.offset.x - i * fix_data.metersSize;
	const offsetY = fix_data.offset.y - j * fix_data.metersSize;

	const tileURL = server_url.replace("{z}",zoomLevel).replace("{x}",x).replace("{y}",y);
	let tileImage = navsat.live_cache[tileURL];

	if(!tileImage || !tileImage.complete){
		tileImage = placeholder;
		navsat.enqueue(tileURL);
	}

	let transformed = undefined;

	if(!ignoreRotationCheckbox.checked){
		transformed = tf.transformPose(
			map_fix.header.frame_id,
			tf.fixed_frame,
			{x: -offsetX, y: offsetY, z: 0},
			new Quaternion()
		);
	}else{
		transformed = tf.transformPose(
			map_fix.header.frame_id,
			tf.fixed_frame,
			{x: 0, y: 0, z: 0},
			new Quaternion()
		);

		transformed.translation.x -= offsetX;
		transformed.translation.y += offsetY;
		transformed.rotation = Quaternion()
	}

	const pos = view.fixedToScreen({
		x: transformed.translation.x,
		y: transformed.translation.y,
	});

	const yaw = transformed.rotation.toEuler().h;

	ctx.save();
	ctx.translate(pos.x, pos.y);
	ctx.scale(1.0, 1.0);
	ctx.rotate(-yaw);
	ctx.drawImage(tileImage, 0, 0, screenSize, screenSize);
	ctx.restore();
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

	const frame = tf.absoluteTransforms[map_fix.header.frame_id];

	if(frame){

		const tileScreenSize = view.getMapUnitsInPixels(fix_data.metersSize);
		const corners = [
			{ x: 0, y: 0, z: 0 },
			{ x: wid, y: 0, z: 0 },
			{ x: wid, y: hei, z: 0  },
			{ x: 0, y: hei, z: 0  },
		];

		// Convert the corners from pixels to meters, transform them to map_fix frame and convert to latitude, longitude
		const cornerCoords = corners.map((corner) => {
			const meters = view.screenToFixed(corner);
			const transformed = tf.transformPose(
				tf.fixed_frame,
				map_fix.header.frame_id,
				meters,
				new Quaternion()
			);
			return {
				latitude: map_fix.latitude + (transformed.translation.y * fix_data.degreesPerMeter.latitude),
				longitude: map_fix.longitude + (transformed.translation.x * fix_data.degreesPerMeter.longitude)
			};
		});

		// Convert the corners to tile coordinates
		const cornerTileCoords = cornerCoords.map((coord) =>
			navsat.coordToTile(coord.longitude, coord.latitude, zoomLevel)
		);

		// Calculate the range of tiles to cover the screen
		const minX = Math.min(...cornerTileCoords.map((coord) => coord.x)) - fix_data.tilePos.x;// - 1;
		const maxX = Math.max(...cornerTileCoords.map((coord) => coord.x)) - fix_data.tilePos.x;// + 1;
		const minY = Math.min(...cornerTileCoords.map((coord) => coord.y)) - fix_data.tilePos.y;// - 1;
		const maxY = Math.max(...cornerTileCoords.map((coord) => coord.y)) - fix_data.tilePos.y;// + 1;

		for (let i = minX; i <= maxX; i++) {
			for (let j = minY; j <= maxY; j++) {
				drawTile(tileScreenSize, i, j);
			}
		}

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
		messageType : 'sensor_msgs/NavSatFix'
	});

	status.setWarn("No data received.");
	
	listener = map_topic.subscribe((msg) => {

		if(isNaN(msg.longitude) || isNaN(msg.latitude)){
			status.setError("Invalid fix.");
			return;
		}

		map_fix = msg;

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

		drawTiles();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/NavSatFix");

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
	drawTiles();
}

window.addEventListener("tf_changed", drawTiles);
window.addEventListener("view_changed", drawTiles);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("Satelite Widget Loaded {uniqueID}")