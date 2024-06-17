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

let frame = "";
let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const selectionbox = document.getElementById("{uniqueID}_topic");
const frameSelector = document.getElementById("{uniqueID}_frame");
const modeSelector = document.getElementById("{uniqueID}_mode");
const text_altitude = document.getElementById("{uniqueID}_altitude");

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	modeSelector.value = loaded_data.mode;
	frame = loaded_data.frame;
	topic = loaded_data.topic;

	frameSelector.value = frame;
	selectionbox.value = topic;

}else{

	if(frame == ""){
		frame = "base_link";
		status.setWarn("No frame found, defaulting to base_link");
	}

	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		mode: modeSelector.value,
		frame: frame,
		topic: topic
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

async function drawWidget() {

	const robotframe = tf.absoluteTransforms[frame];
	if(robotframe){

		const mode = MODES[modeSelector.value];
		let meters = robotframe.translation.z;

		if(mode.invert)
			meters = -meters;

		if(meters < 0)
			meters = 0;

		if(mode.dir === "depth")
			text_altitude.innerText = "Depth: "+meters.toFixed(3)+" m";
		else
			text_altitude.innerText = "Altitude: "+meters.toFixed(3)+" m";

		status.setOK();
	}else{
		status.setError("Required transform frame \""+frame+"\" not found.");
	}
}

function resizeScreen(){
	//canvas.height = window.innerHeight;
	//canvas.width = window.innerWidth;
	drawWidget();
}

window.addEventListener("tf_fixed_frame_changed", drawWidget);
window.addEventListener("tf_changed", drawWidget);
window.addEventListener("view_changed", drawWidget);
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
});

frameSelector.addEventListener("change", (event) =>{
	frame = frameSelector.value;
	saveSettings();
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
