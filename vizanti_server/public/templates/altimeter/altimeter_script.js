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
let frame = "";
let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const selectionbox = document.getElementById("{uniqueID}_topic");
const frameSelector = document.getElementById("{uniqueID}_frame");
const modeSelector = document.getElementById("{uniqueID}_mode");
const text_altitude = document.getElementById("{uniqueID}_altitude");
const stepBox = document.getElementById("{uniqueID}_step");

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	modeSelector.value = loaded_data.mode;
	frame = loaded_data.frame;
	topic = loaded_data.topic;

	stepBox.value = loaded_data.step;
	frameSelector.value = frame;
	selectionbox.value = topic;

}else{

	if(frame == ""){
		frame = "base_link";
		status.setWarn("No frame found, defaulting to base_link");
	}

	if(topic == ""){
		topic = "/depth_tgt";
		status.setWarn("No topic found, defaulting to /depth_tgt");
	}

	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		mode: modeSelector.value,
		frame: frame,
		topic: topic,
		step: stepBox.value
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

async function drawWidget() {

	const mode = MODES[modeSelector.value];

	if(mode.dir === "depth")
		text_altitude.innerText = "Depth: "+meters.toFixed(3)+" m";
	else
		text_altitude.innerText = "Altitude: "+meters.toFixed(3)+" m";

	const icon_offset = document.getElementById("icon_bar").getBoundingClientRect().height;
	const wid = canvas.width;
	const hei = canvas.height;
	const centerY = hei / 2;
	const step = Math.min(Math.max(0.1, parseFloat(stepBox.value)), 1000);

	const offsetMeters = meters;
    const pixelOffset = (offsetMeters / step) * -100 + centerY;

	ctx.clearRect(0, 0, wid, hei);
	
	ctx.strokeStyle = "lightgray";
	ctx.lineWidth = 1;
	ctx.beginPath();
	for (let y = pixelOffset, x = 0; y <= hei; y += 10, x+=1) {
		if(x % 5 == 0)
			continue;

		ctx.moveTo(0, y);
		ctx.lineTo(25, y);
	}
	ctx.stroke();

	ctx.strokeStyle = "white";
	ctx.lineWidth = 2;
	ctx.font = "bold 16px Monospace";
	ctx.fillStyle = "white";
	
	ctx.beginPath();
	let lineCount = 0.0;
	for (let y = pixelOffset, x = 0; y <= hei; y += 50, x+=1) {

		if(x % 2 == 0){
			ctx.moveTo(0, y);
			ctx.lineTo(50, y);
			
			ctx.fillText(lineCount.toFixed(1), 55, y + 4);
			lineCount += step;
		}else{
			ctx.moveTo(0, y);
			ctx.lineTo(30, y);
		}
	}
	ctx.stroke();
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = 110;
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

console.log("Altimeter Widget Loaded {uniqueID}")
