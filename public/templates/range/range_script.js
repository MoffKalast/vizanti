import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';
import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let topic = "";

let range_topic = undefined;
let listener = undefined;

let data = {};

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');

opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value
	}
	settings.save();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

function drawRanges() {

	function drawPizza(start_angle, end_angle, min_len, max_len){
        ctx.beginPath();
        ctx.arc(0, 0, min_len, start_angle, end_angle);
        ctx.lineTo(max_len * Math.cos(end_angle), max_len * Math.sin(end_angle));
        ctx.arc(0, 0, max_len, end_angle, start_angle, true);
        ctx.lineTo(min_len * Math.cos(start_angle), min_len * Math.sin(start_angle));
        ctx.closePath();
        ctx.fill();
	}

	const unit = view.getMapUnitsInPixels(1.0);

	const wid = canvas.width;
	const hei = canvas.height;

	ctx.clearRect(0, 0, wid, hei);
	ctx.globalAlpha = opacitySlider.value;

	for (const [key, sample] of Object.entries(data)) {

		let pos = view.fixedToScreen({
			x: sample.pose.translation.x,
			y: sample.pose.translation.y,
		});
	
		let yaw = sample.pose.rotation.toEuler().h;

		let start_angle = -sample.field_of_view/2;
		let end_angle = sample.field_of_view/2;

		ctx.save();
		ctx.translate(pos.x, pos.y);
		ctx.scale(1.0, -1.0);
		ctx.rotate(yaw);

		ctx.fillStyle = "#33414e96";
		drawPizza(start_angle, end_angle, unit*sample.min_range, unit*sample.max_range)

		ctx.fillStyle = "#5eb4ffff";
		let minarc = unit*sample.range-10;

		if(minarc < 0)
			minarc = 10;

		drawPizza(start_angle, end_angle, minarc, unit*sample.range)
		
		ctx.restore();

	}
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawRanges();
}

window.addEventListener("tf_changed", drawRanges);
window.addEventListener("view_changed", drawRanges);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

//Topic

function connect(){

	if(topic == "")
		return;

	if(range_topic !== undefined){
		range_topic.unsubscribe(listener);
	}

	range_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/Range'
	});
	
	listener = range_topic.subscribe((msg) => {		

		const pose = tf.absoluteTransforms[msg.header.frame_id];

		if(!pose)
			return;

		data[msg.header.frame_id] = {
			field_of_view: msg.field_of_view,
			min_range: msg.min_range,
			max_range: msg.max_range,
			range: msg.range,
			type: msg.radiation_type,
			pose: pose
		}
		drawRanges();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/Range");
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
icon.addEventListener("click", loadTopics);

loadTopics();
resizeScreen();

console.log("Range Widget Loaded {uniqueID}")