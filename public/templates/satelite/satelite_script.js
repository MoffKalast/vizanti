import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';
import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let topic = "";
let server_url = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
let listener = undefined;

let map_topic = undefined;
let map_fix = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const tileServerString = document.getElementById('{uniqueID}_tileserver');
const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');

opacitySlider.addEventListener('input', function () {
	opacityValue.textContent = this.value;
	saveSettings();
});

tileServerString.addEventListener('input', function () {
	server_url = this.value;
	saveSettings();
});

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	server_url = loaded_data.server_url;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		server_url: server_url,
		opacity: opacitySlider.value
	}
	settings.save();
}

//Rendering
function drawTiles(){
	const wid = canvas.width;
    const hei = canvas.height;

	ctx.clearRect(0, 0, wid, hei);

	if(!map_fix)
		return;

	console.log(map_fix);
}

//Topic
function connect(){

	if(topic == "")
		return;

	if(map_topic !== undefined){
		map_topic.unsubscribe(listener);
	}

	map_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/NavSatFix'
	});
	
	listener = map_topic.subscribe((msg) => {
		map_fix = msg;
		drawTiles();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/NavSatFix");

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
	drawTiles();
}

window.addEventListener("tf_changed", drawTiles);
window.addEventListener("view_changed", drawTiles);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("Satelite Widget Loaded {uniqueID}")