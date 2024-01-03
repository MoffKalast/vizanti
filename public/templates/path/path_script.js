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

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let listener = undefined;
let path_topic = undefined;

let pose_array = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	saveSettings();
});

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	colourpicker.value = loaded_data.color ?? "#5ED753";
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		color: colourpicker.value,
	}
	settings.save();
}

//Rendering
async function drawPath(){

	const wid = canvas.width;
    const hei = canvas.height;
	ctx.clearRect(0, 0, wid, hei);

	if(pose_array === undefined){
		return false;
	}

	ctx.lineWidth = 2;
	ctx.strokeStyle = colourpicker.value;
	ctx.beginPath();

	pose_array.forEach((point, index) => {
		const pos = view.fixedToScreen({
			x: point.translation.x,
			y: point.translation.y
		});

		if (index === 0) {
			ctx.moveTo(pos.x, pos.y);
		} else {
			ctx.lineTo(pos.x, pos.y);
		}
	});

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
		compression: "cbor"		
	});

	status.setWarn("No data received.");
	
	listener = path_topic.subscribe((msg) => {
		let error = false;
		let newposes = [];
		msg.poses.forEach((point, index) => {
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

icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawPath();
}

window.addEventListener("tf_changed", drawPath);
window.addEventListener("view_changed", drawPath);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("MarkerArray Widget Loaded {uniqueID}")

