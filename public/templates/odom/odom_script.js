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

if(topic != "")
	topic += " (Odometry)";

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let listener = undefined;
let odom_topic = undefined;

let sample_array = [];

let mode = "" //see setMode()
let raw_target = "";

const selectionbox = document.getElementById("{uniqueID}_topic");
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const drawarrows = document.getElementById('{uniqueID}_draw_arrows');
drawarrows.addEventListener('change', ()=>{
	saveSettings();
	drawHistory();
});

const drawpath = document.getElementById('{uniqueID}_draw_path');
drawpath.addEventListener('change', ()=>{
	saveSettings();
	drawHistory();
});

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
	drawHistory();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

const historypicker = document.getElementById('{uniqueID}_history');
historypicker.addEventListener("input", (event) =>{
	saveSettings();

	while (sample_array.length > historypicker.value) {
		sample_array.shift();
	}

	drawHistory();
});

const clearHistoryButton = document.getElementById("{uniqueID}_clearhistory");
clearHistoryButton.addEventListener('click', ()=>{
	sample_array = [];
});

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	historypicker.value = loaded_data.history;
	drawarrows.checked =  loaded_data.draw_arrows;
	drawpath.checked =  loaded_data.draw_path;
	throttle.value = loaded_data.throttle;
	colourpicker.value = loaded_data.color ?? "#54db67";
	setMode();
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
	setMode();
	settings["{uniqueID}"] = {
		topic: topic,
		history: historypicker.value,
		color: colourpicker.value,
		throttle: throttle.value,
		draw_arrows: drawarrows.checked,
		draw_path: drawpath.checked
	}
	settings.save();
}

//Rendering
async function drawHistory(){

	function drawArrow(height, tipwidth) {
		const half = height/2;
		ctx.moveTo(-half, -tipwidth); 
		ctx.lineTo(half, 0);
		ctx.lineTo(-half, tipwidth);
	}

	const wid = canvas.width;
    const hei = canvas.height;
	ctx.setTransform(1,0,0,1,0,0); 
	ctx.clearRect(0, 0, wid, hei);

	if(!drawarrows.checked && !drawpath.checked)
		return;

	if(sample_array.length < 2){
		status.setWarn("No data yet!");
		return;
	}

	ctx.lineWidth = 3;
	ctx.strokeStyle = colourpicker.value;
	ctx.fillStyle = colourpicker.value;
	ctx.beginPath();

	let view_points = [];
	for (let i = 0; i < sample_array.length; i++) {
		view_points[i] = view.fixedToScreen(sample_array[i]);
		view_points[i].yaw = sample_array[i].yaw;
	}

	//continious line 
	if(drawpath.checked){
		ctx.moveTo(view_points[0].x, view_points[0].y);
		for (let i = 1; i < view_points.length; i++) {
			ctx.lineTo(view_points[i].x, view_points[i].y);
		}
		ctx.globalAlpha = 0.6;
		ctx.stroke();
	}

	//yaw indicator
	if(drawarrows.checked){
		ctx.beginPath();

		let prev_p = Infinity;
		for (let i = 0; i < view_points.length; i++) {
			const p = view_points[i];
			if(i == 0 || Math.hypot(p.x - prev_p.x, p.y - prev_p.y) > 20){
				ctx.setTransform(1,0,0,-1,p.x, p.y); //sx,0,0,sy,px,py
				ctx.rotate(p.yaw);
				drawArrow(15, 5);
				prev_p = p;
			}
		}

		ctx.globalAlpha = 1.0;
		ctx.fill();
	}
}

function appendPose(pose){
	const pose2D = {
		x: pose.translation.x,
		y: pose.translation.y,
		yaw: pose.rotation.toEuler().h
	};

	if(sample_array.length > 2){
		const last = sample_array[sample_array.length-1];
		const delta = Math.hypot(last.x - pose2D.x, last.y - pose2D.y);
		if(delta > 0.03){
			sample_array.push(pose2D);
		}else{
			return false;
		}
	}else{
		sample_array.push(pose2D);
	}

	while (sample_array.length > historypicker.value) {
		sample_array.shift();
	}

	return true;
}

//Topic
function connect(){
	if(topic == ""){
		status.setError("No target.");
		return;
	}

	if(odom_topic !== undefined){
		odom_topic.unsubscribe(listener);
	}

	if(mode != "topic")
		return;

	odom_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : raw_target,
		messageType : 'nav_msgs/Odometry',
		throttle_rate: parseInt(throttle.value),
		compression: rosbridge.compression
	});

	status.setWarn("No data received.");
	
	listener = odom_topic.subscribe((msg) => {
		
		let error = false;
		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
			msg.header.frame_id = tf.fixed_frame;
			error = true;
		}

		const frame = tf.absoluteTransforms[msg.header.frame_id];

		if(!frame){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			error = true;
			return;
		}

		const transformed = tf.transformPose(
			msg.header.frame_id,
			tf.fixed_frame, 
			msg.pose.pose.position, 
			msg.pose.pose.orientation
		)

		if(appendPose(transformed)){
			drawHistory();
			if(!error){
				status.setOK();
			}
		}
	});

	saveSettings();
}

async function loadTopics(){
	let odom_array = await rosbridge.get_topics("nav_msgs/Odometry");
	let tf_array = Array.from(tf.frame_list);

	let topiclist = "";
	odom_array.forEach(element => {
		topiclist += "<option value='"+element+" (Odometry)'>"+element+" (Odometry)</option>"
	});

	tf_array.forEach(frame => {
		topiclist += "<option value='"+frame+" (Frame)'>"+frame+" (Frame)</option>"
	});

	selectionbox.innerHTML = topiclist

	if(topic == ""){
		topic = selectionbox.value;
		setMode();
	}else{
		setMode();
		if(odom_array.includes(raw_target) || tf_array.includes(raw_target)){
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
	sample_array = [];
	saveSettings();
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

click_icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

function setMode(){
	if(topic.endsWith("(Frame)")){
		mode = "tf";
		raw_target = topic.replace(" (Frame)", "");
	}else if(topic.endsWith("(Odometry)")){
		mode = "topic";
		raw_target = topic.replace(" (Odometry)", "");
	}else{
		mode = "";
		raw_target = "";
	}
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawHistory();
}

let tf_throttle_stamp = 0;
window.addEventListener("tf_changed", ()=>{
	if(mode == "tf"){
		const now = Date.now();
		if(now - tf_throttle_stamp >= parseInt(throttle.value)){
			const frame = tf.absoluteTransforms[raw_target];

			if(!frame){
				status.setError("Required transform frame \""+raw_target+"\" not found.");
				return;
			}

			if(appendPose(frame)){
				drawHistory();
				status.setOK();
				tf_throttle_stamp = now;
			}
		}
	}
});

window.addEventListener("tf_fixed_frame_changed", ()=>{
	sample_array = []; //TODO keep a ref to the old frame and trasform the history
	drawHistory();
});

window.addEventListener("view_changed", drawHistory);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("Odom Pose Tracker Widget Loaded {uniqueID}")

