let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let view = viewModule.view;
let tf = tfModule.tf;
let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let Status = StatusModule.Status;

let MODES = {
	"altitude_positive": {dir: "altitude", invert: false},
	"altitude_negative": {dir: "altitude", invert: true},
	"depth_negative": {dir: "depth", invert: true},
	"depth_positive": {dir: "depth", invert: false},
};

let meters = 0;
let meters_smooth = 0;

let frame = "";
let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let img_offset_x = "0";

const clamp = (num, min, max) => Math.min(Math.max(num, min), max);

const arrow = document.getElementById('{uniqueID}_arrow');
const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const icon_bar = document.getElementById("icon_bar");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const selectionbox = document.getElementById("{uniqueID}_topic");
const frameSelector = document.getElementById("{uniqueID}_frame");
const modeSelector = document.getElementById("{uniqueID}_mode");
const text_altitude = document.getElementById("{uniqueID}_altitude");
const stepBox = document.getElementById("{uniqueID}_step");

const imgpreview = document.getElementById('{uniqueID}_imgpreview');

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	modeSelector.value = loaded_data.mode;
	frame = loaded_data.frame;
	topic = loaded_data.topic;

	stepBox.value = loaded_data.step;
	frameSelector.value = frame;
	selectionbox.value = topic;

	img_offset_x = loaded_data.img_offset_x;
}else{

	if(frame == ""){
		frame = "base_link";
		status.setWarn("No frame found, defaulting to base_link");
	}

	if(topic == ""){
		topic = "/depth_tgt";
		status.setWarn("No topic found, defaulting to /depth_tgt");
	}

	img_offset_x = (document.querySelectorAll('.altimeter_canvas').length-1) * 110;
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		mode: modeSelector.value,
		frame: frame,
		topic: topic,
		step: stepBox.value,
		img_offset_x: img_offset_x
	}
	settings.save();
}

function sendMessage(floatval){
	const publisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: 'std_msgs/Float32'
	});

	const floatMsg = new ROSLIB.Message({
		data: floatval
	});

	publisher.publish(floatMsg);
}

function getMeters(){
	const robotframe = tf.absoluteTransforms[frame];
	if(robotframe)
	{
		const mode = MODES[modeSelector.value];
		let z_meters = robotframe.translation.z;

		if(mode.invert)
			z_meters = -z_meters;

		if(z_meters < 0)
			z_meters = 0;

		status.setOK();
		return parseFloat(z_meters.toFixed(3));
	}
	else
	{
		status.setError("Required transform frame \""+frame+"\" not found.");
		return 0;
	}
}

function drawDepth(){

	const hei = canvas.height;
	const centerY = hei / 2;
	const step = Math.min(Math.max(0.1, parseFloat(stepBox.value)), 1000);

	const offsetMeters = meters_smooth;
    const pixelOffset = (offsetMeters / step) * -100 + centerY;

	const flip = img_offset_x > window.innerWidth/2;
	const flip_offset = flip ? 110: 0;
	const flip_mult = flip ? -1: 1;

	ctx.fillStyle = "#4070bfff";
	ctx.fillRect(flip ? 60: 0, pixelOffset-50, 50, 50);

	ctx.strokeStyle = "lightgray";
	ctx.lineWidth = 1;
	ctx.beginPath();
	for (let y = pixelOffset, x = 0; y <= hei; y += 10, x+=1) {
		if(x % 5 == 0 || y < 0)
			continue;

		ctx.moveTo(flip_offset, y);
		ctx.lineTo(flip_offset + 25 * flip_mult, y);
	}
	ctx.stroke();

	ctx.strokeStyle = "white";
	ctx.lineWidth = 2;

	ctx.font = "bold 16px Monospace";
	ctx.fillStyle = "white";
	ctx.textAlign = flip ? "right" : "left";

	ctx.beginPath();
	let lineCount = 0.0;
	for (let y = pixelOffset, x = 0; y <= hei; y += 50, x+=1) {

		if(y < 0){
			if(x % 2 == 0)
				lineCount += step;
			continue;
		}
			
		if(x % 2 == 0){
			ctx.moveTo(flip_offset, y);
			ctx.lineTo(flip_offset + 50 * flip_mult, y);

			ctx.fillText(Number.isInteger(lineCount) ? lineCount : lineCount.toFixed(1), 55, y + 4);
			lineCount += step;
		}else{
			ctx.moveTo(flip_offset, y);
			ctx.lineTo(flip_offset + 35 * flip_mult, y);
		}
	}
	ctx.stroke();
}

function drawAltitude(){

	const hei = canvas.height;
	const centerY = hei / 2;
	const step = Math.min(Math.max(0.1, parseFloat(stepBox.value)), 1000);

	const offsetMeters = meters_smooth;
    const pixelOffset = (offsetMeters / step) * 100 + centerY;

	const flip = img_offset_x > window.innerWidth/2;
	const flip_offset = flip ? 110: 0;
	const flip_mult = flip ? -1: 1;

	ctx.fillStyle = "#4070bfff";
	ctx.fillRect(flip ? 60: 0, pixelOffset, 50, 50);

	ctx.strokeStyle = "lightgray";
	ctx.lineWidth = 1;
	ctx.beginPath();
	for (let y = pixelOffset, x = 0; y >= 0; y -= 10, x+=1) {
		if(x % 5 == 0 || y > hei)
			continue;

		ctx.moveTo(flip_offset, y);
		ctx.lineTo(flip_offset + 25 * flip_mult, y);
	}
	ctx.stroke();

	ctx.strokeStyle = "white";
	ctx.lineWidth = 2;

	ctx.font = "bold 16px Monospace";
	ctx.fillStyle = "white";
	ctx.textAlign = flip ? "right" : "left";

	ctx.beginPath();
	let lineCount = 0.0;
	for (let y = pixelOffset, x = 0; y >= 0; y -= 50, x+=1) {

		if(y > hei){
			if(x % 2 == 0)
				lineCount += step;
			continue;
		}

		if(x % 2 == 0){
			ctx.moveTo(flip_offset, y);
			ctx.lineTo(flip_offset + 50 * flip_mult, y);

			ctx.fillText(Number.isInteger(lineCount) ? lineCount : lineCount.toFixed(1), 55, y + 4);
			lineCount += step;
		}else{
			ctx.moveTo(flip_offset, y);
			ctx.lineTo(flip_offset + 35 * flip_mult, y);
		}

	}
	ctx.stroke();
}

async function drawWidget() {
	const mode = MODES[modeSelector.value];

	if(mode.dir === "depth")
		text_altitude.innerText = "Depth: "+meters.toFixed(3)+" m";
	else
		text_altitude.innerText = "Altitude: "+meters.toFixed(3)+" m";

	ctx.clearRect(0, 0, canvas.width, canvas.height);

 	if(mode.dir === "depth"){
		drawDepth();
	}else{
		drawAltitude();
	} 
}

function enqueueRender() {
	if(Math.abs(meters - meters_smooth) > 0.005){
		meters_smooth = meters_smooth * 0.95 + meters * 0.05;
		drawWidget();
	}

	window.requestAnimationFrame(enqueueRender);
}

function resizeScreen(){
	canvas.width = 110;
	canvas.height = window.innerHeight - icon_bar.offsetHeight;

	canvas.style.height = (window.innerHeight - icon_bar.offsetHeight) +"px";
	canvas.style.width = "110px";

	arrow.style.bottom = (canvas.height/2 - 60) +"px";

	drawWidget();
}

window.addEventListener("tf_fixed_frame_changed", drawWidget);
window.addEventListener("tf_changed", ()=>{
	let new_val = getMeters();
	if(new_val != meters){
		meters = new_val;
		drawWidget();
	}
});

window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);
window.addEventListener("iconbar_height_change", resizeScreen);

// TF frame list
function setFrameList(){
	let framelist = "";
	for (const key of tf.frame_list.values()) {
		framelist += "<option value='"+key+"'>"+key+"</option>"
	}
	frameSelector.innerHTML = framelist;

	if(tf.transforms.hasOwnProperty(frame)){
		frameSelector.value = frame;
	}else{
		framelist += "<option value='"+frame+"'>"+frame+"</option>"
		frameSelector.innerHTML = framelist
		frameSelector.value = frame;
	}
}

modeSelector.addEventListener("change", (event) => {
	saveSettings();
	drawWidget();
});

frameSelector.addEventListener("change", (event) =>{
	frame = frameSelector.value;
	saveSettings();
	drawWidget();
});

stepBox.addEventListener("change", (event) =>{	
	saveSettings();
	drawWidget();
});


// Topics
async function loadTopics(){
	let result = await rosbridge.get_topics("std_msgs/Float32");

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
}

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	saveSettings();
});

icon.addEventListener("click", ()=>{
	setFrameList();
	loadTopics();
});

resizeScreen();
loadTopics();

//targeting



//preview for definining position
let preview_active = false;

function onStart(event) {
	preview_active = true;
	document.addEventListener('mousemove', onMove);
	document.addEventListener('touchmove', onMove);
	document.addEventListener('mouseup', onEnd);
	document.addEventListener('touchend', onEnd);
}

function displayImageOffset(x){
	imgpreview.style.left = x + canvas.width/2 + "px";
	canvas.style.left = x +"px";

	if(img_offset_x > window.innerWidth/2){
		canvas.style.borderLeft = "5px none transparent";
		canvas.style.borderRight = "5px solid #4070bfff";
		canvas.style.backgroundImage = "linear-gradient(to left, rgba(0, 0, 0, 0.589) , transparent)";
		icon.style.transform = "rotate(180deg)";

		arrow.style.left = (x+55) +"px";
		arrow.style.transform = "translateY(-50%) rotate(180deg)";
	}else{

		canvas.style.borderRight = "5px none transparent";
		canvas.style.borderLeft = "5px solid #4070bfff";
		canvas.style.backgroundImage = "linear-gradient(to right, rgba(0, 0, 0, 0.589) , transparent)";
		icon.style.transform = "";

		arrow.style.left = x +"px";
		arrow.style.transform = "translateY(-50%)";
	}
}

window.addEventListener('resize', ()=>{
	displayImageOffset(img_offset_x);
});

function onMove(event) {
	if (preview_active) {
		event.preventDefault();
		const wid = window.innerWidth-5;
		let currentX;

		if (event.type === "touchmove") {
			currentX = event.touches[0].clientX;
		} else {
			currentX = event.clientX;
		}

		if(currentX > wid/2){
			currentX = wid - currentX + 110;
			img_offset_x = wid - parseInt(currentX/110)*110;
		}else{
			img_offset_x = parseInt(currentX/110)*110;
		}

		displayImageOffset(img_offset_x);
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

displayImageOffset(img_offset_x);
enqueueRender();

console.log("Altimeter Widget Loaded {uniqueID}")
