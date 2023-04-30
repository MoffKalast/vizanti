import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let topic = "";
let listener = undefined;
let temperature_topic = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const highBox = document.getElementById('{uniqueID}_hightemp');
const midBox = document.getElementById('{uniqueID}_midtemp');
const lowBox = document.getElementById('{uniqueID}_lowtemp');

const text_temperature = document.getElementById("{uniqueID}_temperature");
const text_variance = document.getElementById("{uniqueID}_variance");

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	highBox.value = loaded_data.high;
	midBox.value = loaded_data.mid;
	lowBox.value = loaded_data.low;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		low: highBox.value,
		mid: midBox.value,
		high: highBox.value
	}
	settings.save();
}

function connect(){

	if(topic == "")
		return;

	if(temperature_topic !== undefined){
		temperature_topic.unsubscribe(listener);
	}

	temperature_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/Temperature'
	});
	
	listener = temperature_topic.subscribe((msg) => {

		if(msg.temperature > highBox.value){
			icon.src = "assets/temp_hot.svg";
		}
		else if(msg.temperature < lowBox.value){
			icon.src = "assets/temp_cold.svg";
		}
		else{
			icon.src = "assets/temp_warm.svg";
		}

		text_temperature.innerText = "Temperature (°C): "+(Math.round(msg.temperature * 100) / 100).toFixed(2);
		text_variance.innerText = "Variance: "+(Math.round(msg.variance * 100) / 100).toFixed(2);
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/Temperature");

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