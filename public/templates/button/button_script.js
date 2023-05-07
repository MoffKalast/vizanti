import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let topic = "/toggle";

const selectionbox = document.getElementById("{uniqueID}_topic");
const icondiv = document.getElementById("{uniqueID}_icon");
const icon = icondiv.getElementsByTagName('img')[0];
const icontext = icondiv.getElementsByTagName('p')[0];
const namebox = document.getElementById("{uniqueID}_name");

namebox.addEventListener('input', function() {
	icontext.textContent = namebox.value;
	saveSettings();
});

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	namebox.value = loaded_data.text;
	icontext.textContent = loaded_data.text;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		text: namebox.value
	}
	settings.save();
}

//Messaging

function sendMessage(){
	const publisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: 'std_msgs/Bool',
	});

	const boolMessage = new ROSLIB.Message({
		data: !value,
	});	

	publisher.publish(boolMessage);

}

let value = false;
let listener = undefined;
let booltopic = undefined;

function connect(){

	if(topic == "")
		return;

	if(booltopic !== undefined){
		booltopic.unsubscribe(listener);
	}

	booltopic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'std_msgs/Bool'
	});
	
	listener = booltopic.subscribe((msg) => {
		value = msg.data;
		icon.src = "assets/button_"+value+".svg";
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("std_msgs/Bool");
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
	icon.src = "assets/button.svg";
	connect();
});

selectionbox.addEventListener("click", connect);
icon.addEventListener("click", loadTopics);

loadTopics();

// Long press modal open stuff
let longPressTimer;
let isLongPress = false;

icondiv.addEventListener("click", (event) =>{
	if(!isLongPress){
		sendMessage();
	}else{
		isLongPress = false;
	}
});

icondiv.addEventListener("mousedown", startLongPress);
icondiv.addEventListener("touchstart", startLongPress);

icondiv.addEventListener("mouseup", cancelLongPress);
icondiv.addEventListener("mouseleave", cancelLongPress);
icondiv.addEventListener("touchend", cancelLongPress);
icondiv.addEventListener("touchcancel", cancelLongPress);

icondiv.addEventListener("contextmenu", (event) => {
	event.preventDefault();
});

function startLongPress(event) {
	isLongPress = false;
	longPressTimer = setTimeout(() => {
		isLongPress = true;
		loadTopics();
		openModal("{uniqueID}_modal");
	}, 500);
}

function cancelLongPress(event) {
	clearTimeout(longPressTimer);
}

console.log("Button Widget Loaded {uniqueID}")