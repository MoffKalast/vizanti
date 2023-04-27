import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';
import { nipplejs } from '/js/modules/joystick.js';

let topic = "/cmd_vel";
let cmdVelPublisher = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

// Sliders and checkboxes

const linearVelSlider = document.getElementById('{uniqueID}_linear_velocity');
const angularVelSlider = document.getElementById('{uniqueID}_angular_velocity');
const smoothnessSlider = document.getElementById('{uniqueID}_smoothness');

const linearVelValue = document.getElementById('{uniqueID}_linear_velocity_value');
const angularVelValue = document.getElementById('{uniqueID}_angular_velocity_value');
const smoothnessValue = document.getElementById('{uniqueID}_smoothness_value');

const invertAngularCheckbox = document.getElementById('{uniqueID}_invert_angular');
const holonomicSwapCheckbox = document.getElementById('{uniqueID}_holonomic');

linearVelSlider.addEventListener('input', function () {
	linearVelValue.textContent = this.value;
});

angularVelSlider.addEventListener('input', function () {
	angularVelValue.textContent = this.value;
});

smoothnessSlider.addEventListener('input', function () {
	smoothnessValue.textContent = parseInt((0.5 - this.value)*200) + "%";
});

linearVelSlider.addEventListener('change', saveSettings);
angularVelSlider.addEventListener('change', saveSettings);
smoothnessSlider.addEventListener('change', saveSettings);

invertAngularCheckbox.addEventListener('change', saveSettings);
holonomicSwapCheckbox.addEventListener('change', saveSettings);

// Settings

if (settings.hasOwnProperty('{uniqueID}')) {
	const loadedData = settings['{uniqueID}'];
	topic = loadedData.topic;

	linearVelSlider.value = loadedData.linear_velocity;
	angularVelSlider.value = loadedData.angular_velocity;
	smoothnessSlider.value = loadedData.smoothness;

	invertAngularCheckbox.checked = loadedData.invert_angular;
	holonomicSwapCheckbox.checked = loadedData.holonomic_swap;

	linearVelValue.textContent = linearVelSlider.value;
	angularVelValue.textContent = angularVelSlider.value;
}
else{
	saveSettings();
}

function saveSettings() {
	settings['{uniqueID}'] = {
		topic: topic,
		linear_velocity: linearVelSlider.value,
		angular_velocity: angularVelSlider.value,
		smoothness: smoothnessSlider.value,
		invert_angular: invertAngularCheckbox.checked,
		holonomic_swap: holonomicSwapCheckbox.checked,
	};
	settings.save();
}

// Topic and connections

async function loadTopics(){
	let result = await rosbridge.get_topics("geometry_msgs/Twist");
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

function connect(){

	cmdVelPublisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: 'geometry_msgs/Twist',
	});
}

function publishTwist(linearX, linearY, angularZ) {
	const twist = new ROSLIB.Message({
		linear: { x: linearX, y: linearY, z: 0 },
		angular: { x: 0, y: 0, z: angularZ },
	});
	cmdVelPublisher.publish(twist);
}

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	saveSettings();
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

// Joystick

const joystickContainer = document.getElementById('{uniqueID}_joystick');

linearVelSlider.addEventListener('input', function () {
	linearVelValue.textContent = this.value;
	console.log(this.value)
});

angularVelSlider.addEventListener('input', function () {
	angularVelValue.textContent = this.value;
});

// Teleop logic
let targetLinearVel = 0;
let targetAngularVel = 0;
let currentLinearVel = 0;
let currentAngularVel = 0;

const joystick = nipplejs.create({
	zone: joystickContainer,
	mode: 'static',
	position: { left: '50%', bottom: '15%' },
	size: 200,
});

joystick.on('move', (event, data) => {
	const maxLinearVel = parseFloat(linearVelSlider.value);
	const maxAngularVel = parseFloat(angularVelSlider.value);
	const invertAngular = invertAngularCheckbox.checked;
	const holonomicSwap = holonomicSwapCheckbox.checked;

	// Calculate target velocities based on joystick data
	targetLinearVel = maxLinearVel * Math.sin(data.angle.radian) * data.force;
	targetAngularVel = - maxAngularVel * Math.cos(data.angle.radian) * data.force;

	if (settings['{uniqueID}'].invert_angular && targetLinearVel < 0) {
		targetAngularVel = -targetAngularVel;
	}
});

joystick.on('end', () => {
	targetLinearVel = 0;
	targetAngularVel = 0;
});

function lerp(start, end, t) {
	return start + (end - start) * t;
}

const updateInterval = setInterval(() => {
	const lerpFactor = settings['{uniqueID}'].smoothness;

	currentLinearVel = lerp(currentLinearVel, targetLinearVel, lerpFactor);
	//currentAngularVel = lerp(currentAngularVel, targetAngularVel, lerpFactor);

	if(settings['{uniqueID}'].holonomic_swap){
		publishTwist(currentLinearVel, targetAngularVel, 0);
	}
	else{
		publishTwist(currentLinearVel, 0, targetAngularVel);
	}
	
}, 1000 / 30);  // 30 Hz update rate