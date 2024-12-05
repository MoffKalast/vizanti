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

let step = 1.0;
let meters = 0;
let meters_smooth = 0;
let target = NaN;

let frame = "";
let topic = getTopic("{uniqueID}");

let float_topic = undefined;
let listener = undefined;

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
const stepBox = document.getElementById("{uniqueID}_step");

const text_altitude = document.getElementById("{uniqueID}_altitude_text");
const text_target = document.getElementById("{uniqueID}_target_text");

const imgpreview = document.getElementById('{uniqueID}_imgpreview');

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	modeSelector.value = loaded_data.mode;
	frame = loaded_data.frame;
	topic = loaded_data.topic;

	stepBox.value = loaded_data.step;
	frameSelector.value = frame;
	selectionbox.value = topic;

	step = loaded_data.step;
	img_offset_x = loaded_data.img_offset_x;
}else{

	if(frame == ""){
		frame = "base_link";
		status.setWarn("No frame found, defaulting to base_link");
	}

	/* if(topic == ""){
		topic = "/depth_tgt";
		status.setWarn("No topic found, defaulting to /depth_tgt");
	} */

	img_offset_x = (document.querySelectorAll('.altimeter_canvas').length-1) * 110;
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		mode: modeSelector.value,
		frame: frame,
		topic: topic,
		step: step,
		img_offset_x: img_offset_x
	}
	settings.save();
}

//topic
function connect(){

	if(topic == ""){
		status.setWarn("Empty topic.");
		return;
	}

	if(float_topic !== undefined){
		float_topic.unsubscribe(listener);
	}

	float_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'std_msgs/Float32'
	});
	
	listener = float_topic.subscribe((msg) => {
		const mode = MODES[modeSelector.value];
		target = Math.abs(msg.data);
		if(msg.data > 0){
			if(mode.dir == "depth" && mode.invert)
				target = NaN;

			if(mode.dir == "altitude" && mode.invert)
				target = NaN;

		}else if (msg.data < 0){
			if(mode.dir == "depth" && !mode.invert)
				target = NaN;

			if(mode.dir == "altitude" && !mode.invert)
				target = NaN;
		}

		if(isNaN(target))
			text_target.innerText = "Target: N/A";
		else
			text_target.innerText = "Target: "+target.toFixed(3)+" m";
		
		drawWidget();
	});

	saveSettings();
}

function publishTarget(value){

	if(topic == "")
		return;

	if(MODES[modeSelector.value].invert)
		value = -value;

	const publisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: 'std_msgs/Float32'
	});

	const floatMsg = new ROSLIB.Message({
		data: value
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

function drawTarget(flip_offset, flip_mult, pos){
	ctx.fillStyle = "#e3df6f";
	ctx.lineWidth = 1;
	ctx.beginPath();
	ctx.moveTo(flip_offset, pos-25);
	ctx.lineTo(flip_offset+ 25 * flip_mult, pos);
	ctx.lineTo(flip_offset+ 25 * flip_mult, pos);
	ctx.lineTo(flip_offset, pos+25);
	ctx.lineTo(flip_offset, pos-25);
	ctx.fill();
}

function drawDepth(){

	const hei = canvas.height;
	const centerY = hei/2;
    const pixelOffset = (meters_smooth / step) * -100 + centerY;

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

	if(!isNaN(target)){
		const pos = pixelOffset + ((target / step) * 100);
		drawTarget(flip_offset, flip_mult, pos);
	}
}

function drawAltitude(){

	const hei = canvas.height;
	const centerY = hei/2;
    const pixelOffset = (meters_smooth / step) * 100 + centerY;

	const flip = img_offset_x > window.innerWidth/2;
	const flip_offset = flip ? 110: 0;
	const flip_mult = flip ? -1: 1;

	ctx.fillStyle = "#5a9558ff";
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

	if(!isNaN(target)){
		const pos = pixelOffset + ((target / step) * -100);
		drawTarget(flip_offset, flip_mult, pos);

	}
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
	if(Math.abs(meters - meters_smooth) > step * 0.01){
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

	if(MODES[modeSelector.value].dir == "depth")
		arrow.style.bottom = (canvas.height/2 - 60) +"px";
	else
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
	target = NaN;
	text_target.innerText = "Target: N/A";
	saveSettings();
	refreshStyleSetup();
	drawWidget();
});

frameSelector.addEventListener("change", (event) =>{
	frame = frameSelector.value;
	saveSettings();
	refreshStyleSetup();
	drawWidget();
});

stepBox.addEventListener("change", (event) =>{	
	step = Math.min(Math.max(0.1, parseFloat(stepBox.value)), 1000);
	saveSettings();
	refreshStyleSetup();
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

	connect();
}

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	saveSettings();
	connect();
});

icon.addEventListener("click", ()=>{
	setFrameList();
	loadTopics();
});

resizeScreen();
loadTopics();

//targeting
function getEventXY(event){
	let globalX, globalY;
	if (event.type === "touchmove") {
		globalX = event.touches[0].clientX;
		globalY = event.touches[0].clientY;
	} else {
		globalX = event.clientX;
		globalY = event.clientY;
	}
	return [globalX, globalY];
}

function getEventLocalXY(event){

	const [globalX, globalY] = getEventXY(event);

	const rect = event.target.getBoundingClientRect();
	let x = globalX - rect.left;
	let y = globalY - rect.top;

	//are we flipped
	if(img_offset_x > window.innerWidth/2)
		x = rect.width - x;

	return [x, y];
}

function setTargetFromPixels(y){
	
	const centerY = canvas.height/2;
	let newtgt = 0;

	if(MODES[modeSelector.value].dir == "depth"){
		newtgt = ((y - centerY) / 100 + (meters_smooth / step)) * step;
	}else{
		newtgt = ((y - centerY) / -100 + (meters_smooth / step)) * step;
	}

	if(newtgt > -1){
		publishTarget(newtgt > 0 ? newtgt: 0);
	}
}

let targeting_active = false;
let targeting_point = {
	x: 0, 
	y: 0
};

function onTargetStart(event) {

	if(topic == "")
		return;

	const [x, y] = getEventLocalXY(event);
	if(x > 30)
		return;

	const [globalX, globalY] = getEventXY(event);

	targeting_active = true;
	targeting_point.x = globalX;
	targeting_point.y = globalY;

	document.addEventListener('mouseup', onTargetEnd);
	document.addEventListener('touchend', onTargetEnd);
}

function onTargetEnd(event) {

	targeting_active = false;
	document.removeEventListener('mouseup', onTargetEnd);
	document.removeEventListener('touchend', onTargetEnd);

	const [globalX, globalY] = getEventXY(event);

	if(Math.hypot(targeting_point.y - globalY, targeting_point.x - globalX) < 15){
		const [x, y] = getEventLocalXY(event);
		setTargetFromPixels(y);
	}
}
  
canvas.addEventListener('mousedown', onTargetStart);
canvas.addEventListener('touchstart', onTargetStart);


//preview for definining position
let preview_active = false;

function refreshStyleSetup(){
	imgpreview.style.left = img_offset_x + canvas.width/2 + "px";
	canvas.style.left = img_offset_x +"px";

	let color;
	if(MODES[modeSelector.value].dir == "depth"){
		arrow.src = "assets/altimeter_arrow.svg"
		icon.src = "assets/altimeter.svg";
		color = "#4070bfff";
	}else{
		arrow.src = "assets/altimeter_arrow_green.svg";
		icon.src = "assets/altimeter_green.svg";
		color = "#5a9558ff";
	}

	if(img_offset_x > window.innerWidth/2){
		canvas.style.borderLeft = "5px none transparent";
		canvas.style.borderRight = "5px solid "+color;
		canvas.style.backgroundImage = "linear-gradient(to left, rgba(0, 0, 0, 0.589) , transparent)";
		icon.style.transform = "rotate(180deg)";

		arrow.style.left = (img_offset_x + 55) +"px";
		arrow.style.transform = "translateY(-50%) rotate(180deg)";
	}else{

		canvas.style.borderRight = "5px none transparent";
		canvas.style.borderLeft = "5px solid "+color;
		canvas.style.backgroundImage = "linear-gradient(to right, rgba(0, 0, 0, 0.589) , transparent)";
		icon.style.transform = "";

		arrow.style.left = img_offset_x +"px";
		arrow.style.transform = "translateY(-50%)";
	}
}

window.addEventListener('resize', ()=>{
	refreshStyleSetup();
});

function onStart(event) {
	preview_active = true;
	document.addEventListener('mousemove', onMove);
	document.addEventListener('touchmove', onMove);
	document.addEventListener('mouseup', onEnd);
	document.addEventListener('touchend', onEnd);
}

function onMove(event) {
	if (preview_active) {
		event.preventDefault();
		const wid = window.innerWidth-5;
		let [currentX, currentY] = getEventXY(event);

		if(currentX > wid/2){
			currentX = wid - currentX + 110;
			img_offset_x = wid - parseInt(currentX/110)*110;
		}else{
			img_offset_x = parseInt(currentX/110)*110;
		}

		refreshStyleSetup();
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

refreshStyleSetup();
enqueueRender();

//manual targeting
document.getElementById("{uniqueID}_manual_target").addEventListener("click", async (event) =>{

	let value = await prompt("Enter target "+MODES[modeSelector.value].dir+" (meters, positive only):", "0.0");
	if (value != null) {
		publishTarget(Math.abs(value));
		drawWidget();
	}
});

console.log("Altimeter Widget Loaded {uniqueID}")
