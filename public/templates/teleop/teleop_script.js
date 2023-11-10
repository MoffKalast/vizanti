let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let joystickModule = await import(`${base_url}/js/modules/joystick.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let nipplejs = joystickModule.nipplejs;
let Status = StatusModule.Status;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let joy_offset_x = "50%";
let joy_offset_y = "85%";
let cmdVelPublisher = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

// Sliders and checkboxes

const linearVelSlider = document.getElementById('{uniqueID}_linear_velocity');
const angularVelSlider = document.getElementById('{uniqueID}_angular_velocity');
const accelSlider = document.getElementById('{uniqueID}_accel');

const linearVelValue = document.getElementById('{uniqueID}_linear_velocity_value');
const angularVelValue = document.getElementById('{uniqueID}_angular_velocity_value');
const accelValue = document.getElementById('{uniqueID}_accel_value');

const invertAngularCheckbox = document.getElementById('{uniqueID}_invert_angular');
const holonomicSwapCheckbox = document.getElementById('{uniqueID}_holonomic');

linearVelSlider.addEventListener('input', function () {
	linearVelValue.textContent = this.value;
	saveSettings();
});

angularVelSlider.addEventListener('input', function () {
	angularVelValue.textContent = this.value;
	saveSettings();
});

accelSlider.addEventListener('input', function () {
	accelValue.textContent = this.value;
	saveSettings();
});

linearVelSlider.addEventListener('change', saveSettings);
angularVelSlider.addEventListener('change', saveSettings);
accelSlider.addEventListener('change', saveSettings);

invertAngularCheckbox.addEventListener('change', saveSettings);
holonomicSwapCheckbox.addEventListener('change', saveSettings);

// Settings

if (settings.hasOwnProperty('{uniqueID}')) {
	const loaded_data = settings['{uniqueID}'];
	topic = loaded_data.topic;

	joy_offset_x = loaded_data.joy_offset_x;
	joy_offset_y = loaded_data.joy_offset_y;

	linearVelSlider.value = loaded_data.linear_velocity;
	angularVelSlider.value = loaded_data.angular_velocity;
	accelSlider.value = loaded_data.accel;

	invertAngularCheckbox.checked = loaded_data.invert_angular;
	holonomicSwapCheckbox.checked = loaded_data.holonomic_swap;

	linearVelValue.textContent = linearVelSlider.value;
	angularVelValue.textContent = angularVelSlider.value;
	accelValue.textContent = accelSlider.value;
}else{
	saveSettings();
}

if(topic == ""){
	topic = "/cmd_vel";
	status.setWarn("No topic found, defaulting to /cmd_vel");
	saveSettings();
}

function saveSettings() {
	settings['{uniqueID}'] = {
		topic: topic,
		linear_velocity: parseFloat(linearVelSlider.value),
		angular_velocity: parseFloat(angularVelSlider.value),
		joy_offset_x: joy_offset_x,
		joy_offset_y: joy_offset_y,
		accel: parseFloat(accelSlider.value),
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
	selectionbox.innerHTML = topiclist;

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
		queue_size: 1
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
	status.setOK();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

// Joystick
linearVelSlider.addEventListener('input', function () {
	linearVelValue.textContent = this.value;
});

angularVelSlider.addEventListener('input', function () {
	angularVelValue.textContent = this.value;
});

// Teleop logic
let linearVel = 0;
let angularVel = 0;

let targetLinearVel = 0;
let targetAngularVel = 0;

let interval = undefined;

const joystickContainer  = document.getElementById('{uniqueID}_joystick');
const joypreview = document.getElementById('{uniqueID}_joypreview');
joypreview.style.left = `calc(${joy_offset_x} - 50px)`;
joypreview.style.top = `calc(${joy_offset_y} - 50px)`;

function makeJoystick(){
	return nipplejs.create({
		zone: joystickContainer,
		mode: 'static',
		position: {
			left: joy_offset_x,
			top: joy_offset_y 
		},
		size: 150,
		threshold: 0.1,
		color: 'black',
		restOpacity: 0.7
	})
}

let joystick = makeJoystick();

function addJoystickListeners(){
	joystick.on('move', onJoystickMove);
	joystick.on('touchmove', onJoystickMove);
	joystick.on('end', onJoystickEnd);
	joystick.on('touchend', onJoystickEnd);
}

function onJoystickMove(event, data) {
	const maxLinearVel = parseFloat(linearVelSlider.value);
	const maxAngularVel = parseFloat(angularVelSlider.value);
	const force = Math.min(Math.max(data.force, 0.0), 1.0);

	targetLinearVel = maxLinearVel * Math.sin(data.angle.radian) * force;
	targetAngularVel = - maxAngularVel * Math.cos(data.angle.radian) * force;

	if (settings['{uniqueID}'].invert_angular && targetLinearVel < 0 && !settings['{uniqueID}'].holonomic_swap) {
		targetAngularVel = -targetAngularVel;
	}

	if(interval === undefined){
		interval = setInterval(() => {
			let accel = settings['{uniqueID}'].accel;

			if (targetLinearVel == 0 && targetAngularVel == 0)
				accel *= 2;
		
			if(linearVel != targetLinearVel){
				if(linearVel < targetLinearVel){
					linearVel += accel;
		
					if(linearVel > targetLinearVel)
						linearVel = targetLinearVel;
				}
				else if(linearVel > targetLinearVel){
					linearVel -= accel;
		
					if(linearVel < targetLinearVel)
						linearVel = targetLinearVel;
				}
			}

			let angular_accel = settings['{uniqueID}'].accel * 10;

			if(settings['{uniqueID}'].holonomic_swap)
				angular_accel = accel;


			if(angularVel != targetAngularVel){
				if(angularVel < targetAngularVel){
					angularVel += angular_accel;
		
					if(angularVel > targetAngularVel)
						angularVel = targetAngularVel;
				}
				else if(angularVel > targetAngularVel){
					angularVel -= angular_accel;
		
					if(angularVel < targetAngularVel)
						angularVel = targetAngularVel;
				}
			}
		
			if(Math.abs(linearVel) < 0.005 && Math.abs(angularVel) < 0.005){
				linearVel = 0;
				targetLinearVel = 0;
				targetAngularVel = 0;
				publishTwist(0, 0, 0);
				clearInterval(interval);
				interval = undefined;
				return;
			}
		
			if(settings['{uniqueID}'].holonomic_swap){
				publishTwist(linearVel, angularVel, 0);
			}
			else{
				publishTwist(linearVel, 0, angularVel);
			}
			
		}, 1000 / 20);
	}
};

function onJoystickEnd(event) {
	targetLinearVel = 0;
	targetAngularVel = 0;
}

addJoystickListeners();

//preview for moving around

let preview_active = false;

function onStart(event) {
	preview_active = true;
	document.addEventListener('mousemove', onMove);
	document.addEventListener('mouseup', onEnd);
	document.addEventListener('touchmove', onMove);
	document.addEventListener('touchend', onEnd);
}

function onMove(event) {
	if (preview_active) {
		event.preventDefault();
		let currentX, currentY;

		if (event.type === "touchmove") {
			currentX = event.touches[0].clientX;
			currentY = event.touches[0].clientY;
		} else {
			currentX = event.clientX;
			currentY = event.clientY;
		}

		joy_offset_x = (currentX/window.innerWidth * 100) +"%";
		joy_offset_y = (currentY/window.innerHeight * 100) +"%";
		saveSettings();

		joypreview.style.left = `calc(${joy_offset_x} - 50px)`;
		joypreview.style.top = `calc(${joy_offset_y} - 50px)`;

		joystick.destroy();
		joystick = makeJoystick();
	
		addJoystickListeners(joystick);
	}
}

function onEnd() {
	preview_active = false;
	document.removeEventListener('mousemove', onMove);
	document.removeEventListener('mouseup', onEnd);
	document.removeEventListener('touchmove', onMove);
	document.removeEventListener('touchend', onEnd);
}
  
joypreview.addEventListener('mousedown', onStart);
joypreview.addEventListener('touchstart', onStart);

console.log("Teleop Widget Loaded {uniqueID}")