let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);

let view = viewModule.view;
let tf = tfModule.tf;
let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let Status = StatusModule.Status;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let listener = undefined;
let poses_topic = undefined;

let poses = [];
let frame = "";

const scaleSlider = document.getElementById('{uniqueID}_scale');
const scaleSliderValue = document.getElementById('{uniqueID}_scale_value');

scaleSlider.addEventListener('input', function () {
	scaleSliderValue.textContent = this.value;
	drawArrows();
});

scaleSlider.addEventListener('change', saveSettings);

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
	drawArrows();
});

const selectionbox = document.getElementById("{uniqueID}_topic");
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	colourpicker.value = loaded_data.color ?? "#f74127";

	scaleSlider.value = loaded_data.scale;
	scaleSliderValue.textContent = scaleSlider.value;

}else{
	saveSettings();
}

//update the icon colour when it's loaded or when the image source changes
icon.onload = () => {
	utilModule.setIconColor(icon, colourpicker.value);
};
if (icon.contentDocument) {
	utilModule.setIconColor(icon, colourpicker.value);
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		scale: parseFloat(scaleSlider.value),
		color: colourpicker.value
	}
	settings.save();
}

//Rendering

async function drawArrows(){

	function drawArrow(height, width, tip, tipwidth){
		ctx.moveTo(0, -width);
		ctx.lineTo(height - tip, -width);
		ctx.lineTo(height - tip, -tipwidth);
		ctx.lineTo(height, 0);
		ctx.lineTo(height - tip, tipwidth);
		ctx.lineTo(height - tip, width);
		ctx.lineTo(0, width);
		ctx.lineTo(0, -width);
	}

	const unit = view.getMapUnitsInPixels(1.0);

	const wid = canvas.width;
    const hei = canvas.height;

	const scale = unit * parseFloat(scaleSlider.value);
	const arrow_height = parseInt(scale*0.5);
	const arrow_width = parseInt(scale*0.01)+1;
	const arrow_tip = parseInt(scale*0.07)+1;
	const arrow_tipwidth = parseInt(scale*0.07)+1;

	ctx.clearRect(0, 0, wid, hei);
	ctx.fillStyle = colourpicker.value;

	if(frame === tf.fixed_frame && poses.length > 0){
		ctx.beginPath();

		for (let i = 0; i < poses.length; i++) {
			const p = poses[i];
			const screenpos = view.fixedToScreen(p);

			ctx.save();
			ctx.translate(screenpos.x, screenpos.y);
			ctx.scale(1, -1);
			ctx.rotate(p.yaw);

			drawArrow(arrow_height, arrow_width, arrow_tip, arrow_tipwidth);

			ctx.restore();
		}

		ctx.fill();
	}
}

//Topic
function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(poses_topic !== undefined){
		poses_topic.unsubscribe(listener);
	}

	poses_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'geometry_msgs/PoseArray',
		throttle_rate: 50,
		compression: "cbor"
	});

	status.setWarn("No data received.");
	
	listener = poses_topic.subscribe((msg) => {

		let error = false;
		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
			msg.header.frame_id = tf.fixed_frame;
			error = true;
		}

		if(!tf.absoluteTransforms[msg.header.frame_id]){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

		poses = [];
		frame = tf.fixed_frame;

		msg.poses.forEach(p => {
			const transformed = tf.transformPose(
				msg.header.frame_id, 
				tf.fixed_frame, 
				p.position, 
				p.orientation
			);

			poses.push({
				x: transformed.translation.x,
				y: transformed.translation.y,
				yaw: transformed.rotation.toEuler().h
			});
		});
		drawArrows();

		if(!error){
			status.setOK();
		}		
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("geometry_msgs/PoseArray");

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
	poses = [];
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

click_icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawArrows();
}

window.addEventListener("tf_fixed_frame_changed", drawArrows);
window.addEventListener("view_changed", drawArrows);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("PoseArray Widget Loaded {uniqueID}")

