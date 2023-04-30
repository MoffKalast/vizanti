import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';
import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let topic = "";
let listener = undefined;

let map_topic = undefined;
let map_data = undefined;
let map_canvas = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');

opacitySlider.addEventListener('input', function () {
	opacityValue.textContent = this.value;
	saveSettings();
});

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value
	}
	settings.save();
}

//Rendering

function drawMap(){
	const wid = canvas.width;
    const hei = canvas.height;

	ctx.clearRect(0, 0, wid, hei);
	ctx.imageSmoothingEnabled = false;

	if(!map_canvas)
		return;

	const map_width = view.getMapUnitsInPixels(
		map_canvas.width * map_data.info.resolution
	);

	const map_height = view.getMapUnitsInPixels(
		map_canvas.height * map_data.info.resolution
	);

	const frame = tf.absoluteTransforms[map_data.header.frame_id];

	if(frame){

		const p = map_data.info.origin.position;

		let transformed = tf.transformVector(
			map_data.header.frame_id,
			tf.fixed_frame,
			{
				x: p.x,
				y: p.y,
				z: p.z
			}, 
			map_data.info.origin.orientation
		);

		const pos = view.mapToScreen({
			x: transformed.translation.x,
			y: transformed.translation.y,
		});
	
		const yaw = (new Quaternion(
			transformed.rotation.w,
			transformed.rotation.x,
			transformed.rotation.y,
			transformed.rotation.z
		)).toEuler().h;

		ctx.save();
		ctx.globalAlpha = opacitySlider.value;
		ctx.translate(pos.x, pos.y);
		ctx.scale(1.0, -1.0);
		ctx.rotate(yaw);
		ctx.drawImage(map_canvas, 0, 0, map_width, map_height);
		ctx.restore();
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
		messageType : 'nav_msgs/OccupancyGrid',
		throttle_rate: 2000 // throttle to once every two seconds max
	});
	
	listener = map_topic.subscribe((msg) => {

		map_data = msg;
		map_canvas = document.createElement('canvas');

		const mapctx = map_canvas.getContext('2d');

		const width = msg.info.width;
		const height = msg.info.height;
		const data = msg.data;
	  
		map_canvas.width = width;
		map_canvas.height = height;
	  
		let map_img = mapctx.createImageData(width, height);
	  
		// 3. Iterate through the data array and set the canvas pixel colors
		for (let i = 0; i < data.length; i++) {
			const occupancyValue = data[i];
			let color = 255; // White for unknown

			if (occupancyValue >= 0 && occupancyValue <= 100) {
				color = 255 - (occupancyValue * 255) / 100; // Grayscale for occupancy probability
			}

			map_img.data[i * 4] = color; // R
			map_img.data[i * 4 + 1] = color; // G
			map_img.data[i * 4 + 2] = color; // B
			map_img.data[i * 4 + 3] = 255; // A
		}

		mapctx.putImageData(map_img, 0, 0);

		drawMap();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("nav_msgs/OccupancyGrid");

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
	drawMap();
}

window.addEventListener("tf_changed", drawMap);
window.addEventListener("view_changed", drawMap);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("Map Widget Loaded {uniqueID}")