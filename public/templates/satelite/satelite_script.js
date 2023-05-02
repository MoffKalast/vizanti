import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';
import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';
import { navsat } from './js/modules/navsat.js';

let topic = "";
let server_url = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
let listener = undefined;

let map_topic = undefined;
let map_fix = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const tileServerString = document.getElementById('{uniqueID}_tileserver');
const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');

opacitySlider.addEventListener('input', function () {
	opacityValue.textContent = this.value;
	saveSettings();
});

tileServerString.addEventListener('input', function () {
	server_url = this.value;
	saveSettings();
});

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	server_url = loaded_data.server_url;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		server_url: server_url,
		opacity: opacitySlider.value
	}
	settings.save();
}

//Rendering
async function drawTiles(){
	
	const wid = canvas.width;
    const hei = canvas.height;

	ctx.clearRect(0, 0, wid, hei);
	ctx.globalAlpha = opacitySlider.value;

	if(!map_fix)
		return;

	const unit = view.getMapUnitsInPixels(1.0);

	// Get the tile coordinates for the current position
	const zoomLevel = 19; // You can adjust the zoom level as needed
	const tileCoords = navsat.coordToTile(map_fix.longitude, map_fix.latitude, zoomLevel);
	const size = navsat.tileSizeInMeters(map_fix.latitude, zoomLevel);
	const screenSize = view.getMapUnitsInPixels(size);

	const frame = tf.absoluteTransforms[map_fix.header.frame_id];

	if(frame){
		const tileURL = server_url.replace("{z}",zoomLevel).replace("{x}",tileCoords.longitude).replace("{y}",tileCoords.latitude);
		const tileImage = navsat.live_cache[tileURL];

		if(tileImage){
			const tileOriginCoords = navsat.tileToCoord(tileCoords.longitude, tileCoords.latitude, zoomLevel);

			// Calculate the offset between the current position and the tile's origin
			const offsetX = navsat.haversine(map_fix.latitude, tileOriginCoords.longitude, map_fix.latitude, map_fix.longitude);
			const offsetY = navsat.haversine(tileOriginCoords.latitude, map_fix.longitude, map_fix.latitude, map_fix.longitude);
	
			let transformed = tf.transformPose(
				map_fix.header.frame_id,
				tf.fixed_frame,
				{x: -offsetX, y: offsetY, z: 0},
				new Quaternion()
			);
	
			const pos = view.mapToScreen({
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
		else{
			navsat.enqueue(tileURL);
		}
	}
}

//Topic
function connect(){

	if(topic == "")
		return;

	if(map_topic !== undefined){
		map_topic.unsubscribe(listener);
	}

	map_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/NavSatFix'
	});
	
	listener = map_topic.subscribe((msg) => {
		map_fix = msg;
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