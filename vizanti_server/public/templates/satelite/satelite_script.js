let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let navsatModule = await import(`${base_url}/js/modules/navsat.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let view = viewModule.view;
let tf = tfModule.tf;
let applyRotation = tfModule.applyRotation;
let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let navsat = navsatModule.navsat;
let Navsat = navsatModule.Navsat;
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
let enu_origin = undefined;
let enuToScreenMat = undefined;
let last_fix_key = undefined;
let update_throttle = undefined;

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
const text_frame = document.getElementById("{uniqueID}_frame");

const placeholder = new Image();
placeholder.src = "assets/tile_loading.png";

function setOpacityText(val){
	if(val == 0.0)
		opacityValue.textContent = "0.0 (Tile rendering disabled)";
	else
		opacityValue.textContent = val;

	canvas.style.opacity = parseFloat(opacitySlider.value);
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

// The widget framework disables clip() on this context (see below), but the
// quad renderer needs real clipping for triangle texture mapping. Grab the
// native implementation off the prototype before the override so we can call
// it explicitly, scoped inside save()/restore() so no clip state ever leaks.
const nativeClip = CanvasRenderingContext2D.prototype.clip;
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

// Per-frame cache of tile corner ENU positions (corners are shared between
// neighbouring tiles, so this saves ~4x the trig). Cleared at the start of
// every drawTiles() pass, so it can never go stale w.r.t. the ENU origin.
const cornerCache = new Map();
function tileCornerEnu(x, y, z){
	const key = x + "," + y + "," + z;
	let v = cornerCache.get(key);
	if(v === undefined){
		const c = Navsat.tileToCoord(x, y, z);
		v = Navsat.llaToEnu(c.latitude, c.longitude, undefined, enu_origin); // at origin altitude

		//clear if it gets over 1MB
		if (cornerCache.size > 16384)
			cornerCache.clear();

		cornerCache.set(key, v);
	}
	return v;
}

// Texture-map one triangle of an image onto three screen points.
//
// (su, sv) are source coordinates in image pixels, p* are destination screen
// points. The affine is solved exactly so each source vertex lands exactly on
// its destination vertex; the clip confines the draw to this triangle so two
// of these calls render an arbitrary quadrilateral.
//
// The clip path is inflated by ~0.75 px outward from the triangle centroid to
// hide antialiasing hairlines along the internal diagonal and between tiles.
// The affine itself is fitted to the exact (un-inflated) corners so geometry
// stays exact; the inflated rim just shows a sliver of extrapolated texture.
function drawImageTriangle(img, su0, sv0, su1, sv1, su2, sv2, p0, p1, p2) {
	const cx = (p0.x + p1.x + p2.x) / 3;
	const cy = (p0.y + p1.y + p2.y) / 3;
	const GROW = 1.1; // px

	function inflate(p) {
		const dx = p.x - cx, dy = p.y - cy;
		const len = Math.hypot(dx, dy) || 1;
		return { x: p.x + (dx / len) * GROW, y: p.y + (dy / len) * GROW };
	}
	const q0 = inflate(p0), q1 = inflate(p1), q2 = inflate(p2);

	// Solve the affine T with T(su,sv) = p for all three vertices.
	const du1 = su1 - su0, dv1 = sv1 - sv0;
	const du2 = su2 - su0, dv2 = sv2 - sv0;
	const det = du1 * dv2 - du2 * dv1;
	if (det === 0) return;

	const a = ((p1.x - p0.x) * dv2 - (p2.x - p0.x) * dv1) / det;
	const b = ((p1.y - p0.y) * dv2 - (p2.y - p0.y) * dv1) / det;
	const c = ((p2.x - p0.x) * du1 - (p1.x - p0.x) * du2) / det;
	const d = ((p2.y - p0.y) * du1 - (p1.y - p0.y) * du2) / det;
	const e = p0.x - a * su0 - c * sv0;
	const f = p0.y - b * su0 - d * sv0;

	ctx.save();
	ctx.setTransform(1, 0, 0, 1, 0, 0);
	ctx.beginPath();
	ctx.moveTo(q0.x, q0.y);
	ctx.lineTo(q1.x, q1.y);
	ctx.lineTo(q2.x, q2.y);
	ctx.closePath();
	nativeClip.call(ctx);
	ctx.setTransform(a, b, c, d, e, f);
	ctx.drawImage(img, 0, 0);
	ctx.restore();
}

// Draw an image (or a sub-rectangle of it) onto an arbitrary screen-space
// quadrilateral, split along the NW-SE... actually the NE-SW diagonal:
//   triangle 1: NW, NE, SW    triangle 2: NE, SE, SW
// Because all four corners are mapped exactly (no parallelogram extrapolation
// of SE), adjacent tiles share identical corner positions and identical edge
// chords, so the tile mosaic is gap-free at every zoom level by construction.
function drawImageQuad(img, sx, sy, sw, sh, pNW, pNE, pSW, pSE) {
	drawImageTriangle(img, sx, sy, sx + sw, sy, sx, sy + sh, pNW, pNE, pSW);
	drawImageTriangle(img, sx + sw, sy, sx + sw, sy + sh, sx, sy + sh, pNE, pSE, pSW);
}

function drawTile(i, j, tempZoomLevel, maxtile) {
	const tx = fix_data.tilePos.x + i;
	const ty = fix_data.tilePos.y + j;

	// web mercator never wraps vertically — skip out-of-range rows instead
	if (ty < 0 || ty > maxtile)
		return;

	// proper positive modulo for horizontal antimeridian wrap (any negative tx)
	const wrappedX = ((tx % (maxtile + 1)) + (maxtile + 1)) % (maxtile + 1);

	// All four ENU corners of this tile. tileToCoord is evaluated with the
	// *unwrapped* tx so tiles across the antimeridian still land at a
	// continuous easting (sin/cos of longitude are periodic).
	const nw = tileCornerEnu(tx,     ty,     tempZoomLevel);
	const ne = tileCornerEnu(tx + 1, ty,     tempZoomLevel);
	const sw = tileCornerEnu(tx,     ty + 1, tempZoomLevel);
	const se = tileCornerEnu(tx + 1, ty + 1, tempZoomLevel);

	const tileURL = server_url.replace("{z}", tempZoomLevel).replace("{x}", wrappedX).replace("{y}", ty);
	let tileImage = navsat.live_cache[tileURL];
	let parentCrop = null;

	if (!tileImage || !tileImage.complete) {
		navsat.enqueue(tileURL);
		parentCrop = findParentTile(wrappedX, ty, tempZoomLevel);
		if (!parentCrop)
			tileImage = placeholder;
	}

	function inflateCorners(pNW, pNE, pSW, pSE, grow){
		const cx = (pNW.x+pNE.x+pSW.x+pSE.x)/4;
		const cy = (pNW.y+pNE.y+pSW.y+pSE.y)/4;
		const push = (p) => {
			const dx=p.x-cx, dy=p.y-cy, len=Math.hypot(dx,dy)||1;
			return { x:p.x+(dx/len)*grow, y:p.y+(dy/len)*grow };
		};
		return [push(pNW), push(pNE), push(pSW), push(pSE)];
	}

	function enuToScreen(p){
		const m = enuToScreenMat;
		return {
			x: m.a * p.x + m.b * p.y + m.e,
			y: m.c * p.x + m.d * p.y + m.f
		};
	}

	const pNW = enuToScreen(nw);
	const pNE = enuToScreen(ne);
	const pSW = enuToScreen(sw);
	const pSE = enuToScreen(se);

	const [iNW, iNE, iSW, iSE] = inflateCorners(pNW, pNE, pSW, pSE, 1.4);

	// Map the image onto the exact quadrilateral. The SE corner is no longer
	// extrapolated as a parallelogram (NE + SW - NW) — the tile footprint in a
	// tangent plane is a trapezoid, and that extrapolation is what produced
	// the triangular gaps between the bottom corners of adjacent tiles when
	// zoomed far out.
	if (parentCrop){
		drawImageQuad(parentCrop.image, parentCrop.srcX, parentCrop.srcY, parentCrop.srcSize, parentCrop.srcSize, iNW, iNE, iSW, iSE);
	}
	else {
		const sw_px = tileImage.naturalWidth || navsat.tile_size;
		const sh_px = tileImage.naturalHeight || navsat.tile_size;
		drawImageQuad(tileImage, 0, 0, sw_px, sh_px, iNW, iNE, iSW, iSE);
	}
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
	ctx.globalAlpha = 1.0;
	ctx.imageSmoothingEnabled = smoothingCheckbox.checked;

	if(!map_fix){
		return;
	}

	if(opacitySlider.value == 0.0){
		status.setOK();
		return;
	}

	let	tempZoomLevel = Math.round(Math.log2(view.scale)+17);
	tempZoomLevel = clamp(tempZoomLevel, 7, 19);
	if(tempZoomLevel != zoomLevel){
		navsat.clear_queue();
		zoomLevel = tempZoomLevel;
		updateFixData();
	}


	// ENU -> screen is one affine per frame: screen = S * (R * enu + t).
	// Build it once; per corner it's then 4 multiplies + 2 adds.
	const frame = map_fix.frame;
	const ignoreRot = ignoreRotationCheckbox.checked;
	const q = frame.rotation;
	const m00 = ignoreRot ? 1 : 1 - 2 * (q.y * q.y + q.z * q.z);
	const m01 = ignoreRot ? 0 : 2 * (q.x * q.y - q.w * q.z);
	const m10 = ignoreRot ? 0 : 2 * (q.x * q.y + q.w * q.z);
	const m11 = ignoreRot ? 1 : 1 - 2 * (q.x * q.x + q.z * q.z);
	const p0 = view.fixedToScreen({x: frame.translation.x, y: frame.translation.y});
	const s = view.scale;
	enuToScreenMat = {
		a:  s * m00, b:  s * m01, e: p0.x,
		c: -s * m10, d: -s * m11, f: p0.y
	};

	const corners = [
		{ x: 0, y: 0, z: 0 },
		{ x: wid, y: 0, z: 0 },
		{ x: wid, y: hei, z: 0  },
		{ x: 0, y: hei, z: 0  },
	];

	// Convert the corners from pixels to ENU meters in the map_fix frame,
	// then invert the exact same ENU projection used for tile placement to
	// get latitude/longitude. Because culling and drawing now use one and
	// the same (exact, invertible) mapping, they can never diverge no
	// matter how far the view moves from the origin.
	// Note: the inverse transform uses the same stamped absolute transform
	// ("frame") that transformPoseStamped uses in drawTile, so cull and
	// draw also agree on the TF sample.
	const cornerCoords = corners.map((corner) => {
		const meters = view.screenToFixed(corner);
		const d = {
			x: meters.x - frame.translation.x,
			y: meters.y - frame.translation.y,
			z: -frame.translation.z
		};

		let local;
		if(ignoreRotationCheckbox.checked){
			local = d;
		}else{
			local = applyRotation(d, frame.rotation, true);
		}

		return Navsat.enuGroundToLla(local.x, local.y, enu_origin);
	});

	// Convert the corners to tile coordinates (exact mercator)
	const cornerTileCoords = cornerCoords.map((coord) =>
		Navsat.coordToTile(coord.longitude, coord.latitude, tempZoomLevel)
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
			drawTile(centerX+x, centerY+y, tempZoomLevel, maxtile);
		}
		if (x === y || (x < 0 && x === -y) || (x > 0 && x === 1 - y)) {
			[dx, dy] = [-dy, dx];
		}
		x += dx;
		y += dy;
	}

	//transform reset
	ctx.setTransform(1, 0, 0, 1, 0, 0);

	if(copyright != ""){
		ctx.globalAlpha = 0.6;
		ctx.fillStyle = "#171717";
		ctx.fillRect(0, hei-20, 120, 20);

		ctx.globalAlpha = 1.0;
		ctx.font = "12px Monospace";
		ctx.fillStyle = "white";
		ctx.fillText(copyright, 5, hei-5);
	}

	status.setOK();
}

const COVARIANCE_TYPE = {
	0: "(unknown)",
	1: "(approximated)",
	2: "(diagonal known)",
	3: "(known)"
}

let connect_retry = 0;

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
		messageType : 'sensor_msgs/msg/NavSatFix'
	});

	status.setWarn("No data received.");
	text_lat.innerText = "Latitude: ?";
	text_lon.innerText = "Longitude: ?";
	text_alt.innerText = "Altitude: ?";
	text_cov.innerText = "Ground Covariance: ?";
	text_frame.innerText = "TF Frame: ?";

	last_fix_key = undefined;
	update_throttle = new Date("2010-3-2");
	
	listener = map_topic.subscribe((msg) => {

		if(new Date() - update_throttle < 4000 || opacitySlider.value == 0.0) //reduces jitter and CPU load in raw receiver mode
			return;

		update_throttle = new Date();

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

		text_frame.innerText = "TF Frame: "+msg.header.frame_id;

		const frame = tf.getAbsoluteTransform(msg.header);

		if(!frame){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			connect_retry = (connect_retry + 1) % 5;
			if(connect_retry != 0){
				console.log("Satelite tiles connect retry...")
				setTimeout(connect, 500);
				return;
			}
			return;
		}

		connect_retry = 0;
		msg.frame = frame;
		
		map_fix = msg;

		// Only rebuild the ENU origin and tile state if the actual fix changed.
		// If it's the same position with a new header, no need to dump the corner cache and redo all the tile math.
		const cov = msg.position_covariance;
		const fix_key = `${msg.latitude},${msg.longitude},${msg.altitude},${cov[0]},${cov[4]},${cov[8]}`;
		if(fix_key !== last_fix_key){
			last_fix_key = fix_key;
			updateFixData();
		}
		
		drawTiles();
	});

	saveSettings();
}

function updateFixData(){
	cornerCache.clear();

	// The ENU tangent plane is anchored at the fix coordinate. If the backend
	// projects GNSS with a fixed datum (recommended), make sure this topic
	// publishes that datum so both ENU frames coincide exactly.
	const alt = Number.isFinite(map_fix.altitude) ? map_fix.altitude : 0;
	enu_origin = Navsat.buildEnuOrigin(map_fix.latitude, map_fix.longitude, alt);

	fix_data = {
		tilePos: Navsat.coordToTile(map_fix.longitude, map_fix.latitude, zoomLevel)
	};
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