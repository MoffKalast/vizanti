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
let path_topic = undefined;

let pose_array = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
	drawPath();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});


//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	colourpicker.value = loaded_data.color ?? "#54db67";
	throttle.value = loaded_data.throttle ?? 100;
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
		color: colourpicker.value,
		throttle: throttle.value	
	}
	settings.save();
}

//Rendering
async function drawPath(){

	const wid = canvas.width;
    const hei = canvas.height;
	ctx.clearRect(0, 0, wid, hei);

	if(pose_array === undefined || pose_array.length < 2){
		return false;
	}

	ctx.lineWidth = 2;
	ctx.strokeStyle = colourpicker.value;
	ctx.beginPath();

	const firstPos = view.fixedToScreen({
		x: pose_array[0].translation.x,
		y: pose_array[0].translation.y
	});
	ctx.moveTo(firstPos.x, firstPos.y);

	for (let i = 1; i < pose_array.length; i++) {
		const point = pose_array[i];
		const pos = view.fixedToScreen({
			x: point.translation.x,
			y: point.translation.y
		});
		ctx.lineTo(pos.x, pos.y);
	}

	ctx.stroke();
}

//Topic
function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}	

	if(path_topic !== undefined){
		path_topic.unsubscribe(listener);
	}

	path_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'nav_msgs/Path',
		compression: "cbor",
		throttle_rate: parseInt(throttle.value)
	});

	status.setWarn("No data received.");
	
	listener = path_topic.subscribe((msg) => {
		let error = false;
		let newposes = [];

		if(msg.poses == undefined){
			status.setWarn("Received uninitialized list of poses. Wat.");
			error = true;
			return;
		}

		msg.poses.forEach((point, index) => {

			if(point.header.frame_id == ""){
				status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
				point.header.frame_id = tf.fixed_frame;
				error = true;
			}

			const frame = tf.absoluteTransforms[point.header.frame_id];
	
			if(!frame){
				status.setError("Required transform frame \""+point.header.frame_id+"\" not found.");
				error = true;
				return;
			}
	
			newposes.push(tf.transformPose(
				point.header.frame_id, 
				tf.fixed_frame, 
				point.pose.position, 
				point.pose.orientation
			));
		});

		pose_array = newposes;
		drawPath();

		if(!error){
			status.setOK();
		}
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("nav_msgs/Path");

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
	pose_array = undefined;
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
	drawPath();
}

window.addEventListener("tf_fixed_frame_changed", connect);
window.addEventListener("view_changed", drawPath);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("MarkerArray Widget Loaded {uniqueID}")

