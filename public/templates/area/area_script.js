import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';
import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let topic = ""
let seq = 0;

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

function sendMessage(start, end){
	if(!start || !end)
		return;

	let start_pos = view.screenToFixed(start);
	let end_pos = view.screenToFixed(end);

	const centerX = (start_pos.x + end_pos.x) / 2;
	const centerY = (start_pos.y + end_pos.y) / 2;

	const sizeX = Math.abs(end_pos.x - start_pos.x) / 2;
	const sizeY = Math.abs(end_pos.y - start_pos.y) / 2;

	const publisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: 'vision_msgs/BoundingBox3D'
	});

	const boundingBox3DMessage = new ROSLIB.Message({
		center: {
			position: {
				x: centerX,
				y: centerY,
				z: 0
			},
			orientation: {
				x: 0,
				y: 0,
				z: 0,
				w: 1.0
			}
		},
		size: {
			x: sizeX,
			y: sizeY,
			z: 0
		}
	});

	publisher.publish(boundingBox3DMessage);

}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

const view_container = document.getElementById("view_container");

const icon = document.getElementById("{uniqueID}_icon");
const iconImg = icon.getElementsByTagName('img')[0];

let active = false;
let start_point = undefined;
let end_point = undefined;

function drawBox() {
    const wid = canvas.width;
    const hei = canvas.height;

    ctx.clearRect(0, 0, wid, hei);

	if(end_point !== undefined)
	{
		ctx.fillStyle = "lime";
		ctx.globalAlpha = 0.3;
	
		ctx.fillRect(
			start_point.x, 
			start_point.y, 
			end_point.x - start_point.x, 
			end_point.y - start_point.y
		);
	}
}

function startDrag(event){
	const { clientX, clientY } = event.touches ? event.touches[0] : event;
	start_point = {
		x: clientX,
		y: clientY
	};
}

function drag(event){
	if (start_point === undefined) return;

	const { clientX, clientY } = event.touches ? event.touches[0] : event;
	end_point = {
		x: clientX,
		y: clientY
	};

	drawBox();	
}

function endDrag(event){
	sendMessage(start_point, end_point);

	start_point = undefined;
	end_point = undefined;
	drawBox();
	setActive(false);
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
}

window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

function addListeners(){
	view_container.addEventListener('mousedown', startDrag);
	view_container.addEventListener('mousemove', drag);
	view_container.addEventListener('mouseup', endDrag);

	view_container.addEventListener('touchstart', startDrag);
	view_container.addEventListener('touchmove', drag);
	view_container.addEventListener('touchend', endDrag);	
}

function removeListeners(){
	view_container.removeEventListener('mousedown', startDrag);
	view_container.removeEventListener('mousemove', drag);
	view_container.removeEventListener('mouseup', endDrag);

	view_container.removeEventListener('touchstart', startDrag);
	view_container.removeEventListener('touchmove', drag);
	view_container.removeEventListener('touchend', endDrag);	
}

function setActive(value){
	active = value;
	view.setInputMovementEnabled(!active);

	if(active){
		addListeners();
		icon.style.backgroundColor = "rgba(255, 255, 255, 1.0)";
		view_container.style.cursor = "pointer";
	}else{
		removeListeners()
		icon.style.backgroundColor = "rgba(124, 124, 124, 0.3)";
		view_container.style.cursor = "";
	}
}

// Topics
const selectionbox = document.getElementById("{uniqueID}_topic");

async function loadTopics(){
	let result = await rosbridge.get_topics("vision_msgs/BoundingBox3D");

	let topiclist = "";
	result.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+"</option>"
	});
	selectionbox.innerHTML = topiclist

	if(result.includes(topic)){
		selectionbox.value = topic;
	}else{
		topiclist += "<option value='"+topic+"'>"+topic+"</option>"
		selectionbox.innerHTML = topiclist
		selectionbox.value = topic;
	}
}

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	saveSettings();
});

// Long press modal open stuff

let longPressTimer;
let isLongPress = false;

icon.addEventListener("click", (event) =>{
	if(!isLongPress)
		setActive(!active);
	else
		isLongPress = false;
});

icon.addEventListener("mousedown", startLongPress);
icon.addEventListener("touchstart", startLongPress);

icon.addEventListener("mouseup", cancelLongPress);
icon.addEventListener("mouseleave", cancelLongPress);
icon.addEventListener("touchend", cancelLongPress);
icon.addEventListener("touchcancel", cancelLongPress);

icon.addEventListener("contextmenu", (event) => {
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

resizeScreen();
loadTopics();

console.log("Area Widget Loaded {uniqueID}")