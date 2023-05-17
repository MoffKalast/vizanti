import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';
import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let topic = "";
let listener = undefined;
let path_topic = undefined;

let pose_array = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic
	}
	settings.save();
}

//Rendering
function drawPath(){

	const wid = canvas.width;
    const hei = canvas.height;
	ctx.clearRect(0, 0, wid, hei);

	if(pose_array === undefined)
		return;

	ctx.lineWidth = 2;
	ctx.strokeStyle = "#5ED753";
	ctx.beginPath();

	pose_array.forEach((point, index) => {
		const frame = tf.absoluteTransforms[point.header.frame_id];

		if(!frame)
			return;

		let transformed = tf.transformPose(
			point.header.frame_id, 
			tf.fixed_frame, 
			point.pose.position, 
			point.pose.orientation
		);

		const pos = view.fixedToScreen({
			x: transformed.translation.x,
			y: transformed.translation.y
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

	if(topic == "")
		return;

	if(path_topic !== undefined){
		path_topic.unsubscribe(listener);
	}

	path_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'nav_msgs/Path'
	});
	
	listener = path_topic.subscribe((msg) => {
		pose_array = msg.poses;
		drawPath();
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

