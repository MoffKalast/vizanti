import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';
import { imageToDataURL } from '/js/modules/util.js';
import { Status } from '/js/modules/status.js';

let img_offset_x = "-999px";
let img_offset_y = "-999px";

let img_width = -1;
let img_height = -1;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

//persistent loading, so we don't re-fetch on every update
let stock_images = {};
stock_images["loading"] = await imageToDataURL("assets/tile_loading.png");
stock_images["error"] = await imageToDataURL("assets/tile_error.png");

let image_topic = undefined;
let listener = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const rotationbox = document.getElementById("{uniqueID}_rotation");

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const canvas = document.getElementById('{uniqueID}_image');
const imgpreview = document.getElementById('{uniqueID}_imgpreview');

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');
opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

const widthSlider = document.getElementById('{uniqueID}_width');
const widthValue = document.getElementById('{uniqueID}_width_value');
widthSlider.addEventListener('input', () =>  {
	widthValue.textContent = widthSlider.value;
	saveSettings();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

rotationbox.addEventListener("change", (event) => {
	saveSettings();
});

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	img_offset_x = loaded_data.img_offset_x;
	img_offset_y = loaded_data.img_offset_y;

	throttle.value = loaded_data.throttle;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
	canvas.style.opacity = loaded_data.opacity;

	widthSlider.value = loaded_data.width;
	widthValue.innerText = loaded_data.width;
	rotationbox.value = loaded_data.rotation;

	canvas.style.width = loaded_data.width+"%";
	canvas.style.transform = `translate(-50%, -50%) rotate(${loaded_data.rotation}deg)`;
	displayImageOffset(img_offset_x, img_offset_y);
}else{
	clampImagePos(0, window.innerHeight);
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		throttle: throttle.value,
		width: widthSlider.value,
		img_offset_x: img_offset_x,
		img_offset_y: img_offset_y,
		rotation: rotationbox.value
	}
	settings.save();

	canvas.style.opacity = opacitySlider.value;
	canvas.style.width = widthSlider.value+"%";

	canvas.style.transform = `translate(-50%, -50%) rotate(${rotationbox.value}deg)`;
}

//Topic

async function getImage(src) {
    return new Promise((resolve, reject) => {
        let img = new Image();
        img.onload = () => resolve(src);
        img.onerror = () => reject(src);
        img.src = src;
    });
}

function connect(){

	canvas.src = stock_images["loading"];

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(image_topic !== undefined){
		image_topic.unsubscribe(listener);
	}

	status.setWarn("No data received.");

	image_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/CompressedImage',
		throttle_rate: parseInt(throttle.value)
	});
	
	listener = image_topic.subscribe(async (msg) => {  
		const src = 'data:image/jpeg;base64,' + msg.data

		getImage(src)
			.then(() => {
				canvas.src = src;
				status.setOK();
			})
			.catch((e) => {
				canvas.src = stock_images["error"];
				status.setError(e.message);
			});
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/CompressedImage");
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

selectionbox.addEventListener("click", connect);

icon.addEventListener("click", ()=> {
	loadTopics();
});

loadTopics();

//preview for definining position
let preview_active = false;

function onStart(event) {
	preview_active = true;
	document.addEventListener('mousemove', onMove);
	document.addEventListener('touchmove', onMove);
	document.addEventListener('mouseup', onEnd);
	document.addEventListener('touchend', onEnd);
}

function clampImagePos(x, y){
	let offset_x = (x/window.innerWidth * 100);
	let offset_y = (y/window.innerHeight * 100);
	let img_width = widthSlider.value/2;
	let img_height = (widthSlider.value * 3/4 * canvas.naturalHeight)/canvas.naturalWidth;

	if(offset_x < img_width){
		offset_x = img_width;
	}else if(offset_x > 100 - img_width){
		offset_x = 100 - img_width;
	}

	if(offset_y < img_height){
		offset_y = img_height;
	}else if(offset_y > 100 - img_height){
		offset_y = 100 - img_height;
	}

	img_offset_x = offset_x +"%";
	img_offset_y = offset_y +"%";

	displayImageOffset(img_offset_x, img_offset_y);
}

function displayImageOffset(x, y){
	imgpreview.style.left = `calc(${img_offset_x} - 50px)`;
	imgpreview.style.top = `calc(${img_offset_y} - 50px)`;

	canvas.style.left = `calc(${img_offset_x})`;
	canvas.style.top = `calc(${img_offset_y})`;
}

function onMove(event) {
	if (preview_active) {
		event.preventDefault();
		let currentX, currentY;

		if (event.type === "touchmove") {
			currentX = event.touches[0].clientX;
			currentY = event.touches[0].clientY;
		} else {
			currentX = event.clientX;
			currentY = event.clientY;
		}

		clampImagePos(currentX, currentY);
		saveSettings();
	}
}

function onEnd() {
	preview_active = false;
	document.removeEventListener('mousemove', onMove);
	document.removeEventListener('touchmove', onMove);
	document.removeEventListener('mouseup', onEnd);
	document.removeEventListener('touchend', onEnd);
}
  
imgpreview.addEventListener('mousedown', onStart);
imgpreview.addEventListener('touchstart', onStart);

displayImageOffset(img_offset_x, img_offset_y);

console.log("Image Widget Loaded {uniqueID}")

