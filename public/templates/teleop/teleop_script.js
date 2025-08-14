let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let joystickModule = await import(`${base_url}/js/modules/joystick.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);

let tf = tfModule.tf;
let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let nipplejs = joystickModule.nipplejs;
let Status = StatusModule.Status;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let stamp_seq = 0;
let typedict = {};
let joy_offset_x = "50%";
let joy_offset_y = "85%";
let cmdVelPublisher = undefined;

//experimental keyboard control
let keybindings = {
	key_up: 'w',
	key_left: 'a',
	key_down: 's', 
	key_right: 'd'
};
let key_move = {
	vert: 0.0,
	horiz: 0.0,
	up: false,
	down: false,
	left: false,
	right: false
};
let active_keybinds = [];
let keyboard_interval = undefined;
let new_keybind_listener = undefined;

function updateColor(color, alpha){
	const toHex8 = (hex, transparency) => hex + Math.round(transparency * 255).toString(16).padStart(2, '0');
	const combined_color = toHex8(color, parseFloat(alpha)+0.3);
	utilModule.setIconColor(icon, combined_color);
}

const selectionbox = document.getElementById("{uniqueID}_topic");
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

const colourpickerBox = document.getElementById("{uniqueID}_colorpicker");
colourpickerBox.addEventListener("input", (event) =>{
	updateColor(colourpickerBox.value, opacityBox.value);
	joystick.destroy();
	joystick = makeJoystick();
	saveSettings();
});

const opacityValue = document.getElementById('{uniqueID}_opacity_value');
const opacityBox = document.getElementById("{uniqueID}_opacity");
opacityBox.addEventListener("input", (event) =>{
	updateColor(colourpickerBox.value, opacityBox.value);
	opacityValue.textContent = opacityBox.value;
	joystick.destroy();
	joystick = makeJoystick();
	saveSettings();
});

// Axis dropdown
const presetSelectorBox = document.getElementById("{uniqueID}_preset");
const axisVerticalBox = document.getElementById("{uniqueID}_axis_vert");
const axisHorizontalBox = document.getElementById("{uniqueID}_axis_horiz");

// Velocity, accel sliders
const velocityVerticalSlider = document.getElementById('{uniqueID}_vel_vert');
const velocityHorizontalSlider = document.getElementById('{uniqueID}_vel_horiz');

const accelVerticalSlider = document.getElementById('{uniqueID}_accel_vert');
const accelHorizontalSlider = document.getElementById('{uniqueID}_accel_horiz');

const invertVerticalCheckbox = document.getElementById('{uniqueID}_invert_vert');
const invertHorizontalCheckbox = document.getElementById('{uniqueID}_invert_horiz');
const specialAckermannCheckbox = document.getElementById('{uniqueID}_special_ackermann_emulation');
const specialInstantStopCheckbox = document.getElementById('{uniqueID}_special_instant_stop');

const specialKeyboardCheckbox = document.getElementById('{uniqueID}_special_keyboard');
const button_key_up = document.getElementById('{uniqueID}_key_up');
const button_key_down = document.getElementById('{uniqueID}_key_down');
const button_key_right = document.getElementById('{uniqueID}_key_right');
const button_key_left = document.getElementById('{uniqueID}_key_left');

// Value text
const velocityVerticalValue = document.getElementById('{uniqueID}_vel_vert_value');
const velocityHorizontalValue = document.getElementById('{uniqueID}_vel_horiz_value');

const accelVerticalValue = document.getElementById('{uniqueID}_accel_vert_value');
const accelHorizontalValue = document.getElementById('{uniqueID}_accel_horiz_value');

velocityVerticalSlider.addEventListener('input', function () {
	velocityVerticalValue.textContent = this.value;
});

velocityHorizontalSlider.addEventListener('input', function () {
	velocityHorizontalValue.textContent = this.value;
});

accelVerticalSlider.addEventListener('input', function () {
	if(parseInt(this.value) == 12)
		accelVerticalValue.textContent = "Instant"
	else
		accelVerticalValue.textContent = this.value;
});

accelHorizontalSlider.addEventListener('input', function () {
	if(parseInt(this.value) == 12)
		accelHorizontalValue.textContent = "Instant"
	else
		accelHorizontalValue.textContent = this.value;
});

//save on release
axisVerticalBox.addEventListener('change', saveSettings);
axisHorizontalBox.addEventListener('change', saveSettings);

velocityVerticalSlider.addEventListener('change', saveSettings);
velocityHorizontalSlider.addEventListener('change', saveSettings);

accelVerticalSlider.addEventListener('change', saveSettings);
accelHorizontalSlider.addEventListener('change', saveSettings);

invertVerticalCheckbox.addEventListener('change', saveSettings);
invertHorizontalCheckbox.addEventListener('change', saveSettings);
specialAckermannCheckbox.addEventListener('change', saveSettings);
specialInstantStopCheckbox.addEventListener('change', saveSettings);

specialKeyboardCheckbox.addEventListener('change', function(){
	updateKeyboardSetup();
	saveSettings();
});

presetSelectorBox.addEventListener('change', function () {

	if(this.value == "none")
		return;

	invertVerticalCheckbox.checked = false;
	invertHorizontalCheckbox.checked = false;
	specialAckermannCheckbox.checked = false;
	specialInstantStopCheckbox.checked = false;

	keybindings = {
		key_up: 'w',
		key_left: 'a',
		key_down: 's', 
		key_right: 'd'
	};

	switch (this.value) {
		case "diffdrive":
			axisVerticalBox.value = "linear_x";	
			axisHorizontalBox.value = "angular_z";
			velocityVerticalSlider.value = 1.0;
			velocityHorizontalSlider.value = 2.0;
			accelVerticalSlider.value = 2.0;
			accelHorizontalSlider.value = 4.0;
			specialAckermannCheckbox.checked = true;
			break;

		case "ackermann":
			axisVerticalBox.value = "linear_x";	
			axisHorizontalBox.value = "angular_z";
			velocityVerticalSlider.value = 1.0;
			velocityHorizontalSlider.value = 0.9; //steering angle, typically about 50 deg max
			accelVerticalSlider.value = 2.0;
			accelHorizontalSlider.value = 6.0;
			break;

		case "altitude":
			axisVerticalBox.value = "linear_z";	
			axisHorizontalBox.value = "none";
			velocityVerticalSlider.value = 2.0;
			velocityHorizontalSlider.value = 0.05;
			accelVerticalSlider.value = 2.0;
			accelHorizontalSlider.value = 0.5;

			keybindings = {
				key_up: ' ',
				key_left: 'a',
				key_down: 'shift', 
				key_right: 'd'
			};

			break;

		case "holonomic":
			axisVerticalBox.value = "linear_x";	
			axisHorizontalBox.value = "linear_y";
			velocityVerticalSlider.value = 1.0;
			velocityHorizontalSlider.value = 1.0;
			accelVerticalSlider.value = 2.0;
			accelHorizontalSlider.value = 2.0;

			keybindings = {
				key_up: 'i',
				key_left: 'j',
				key_down: 'k', 
				key_right: 'l'
			};

			break;

		case "pitchroll":			
			axisVerticalBox.value = "angular_y";	
			axisHorizontalBox.value = "angular_x";
			velocityVerticalSlider.value = 2.0;
			velocityHorizontalSlider.value = 2.0;
			accelVerticalSlider.value = 4.0;
			accelHorizontalSlider.value = 4.0;
			specialInstantStopCheckbox.checked = true;
			break;

		case "pantilt":			
			axisVerticalBox.value = "angular_y";
			axisHorizontalBox.value = "angular_z";
			velocityVerticalSlider.value = 0.5;
			velocityHorizontalSlider.value = 0.5;
			accelVerticalSlider.value = 12.0;
			accelHorizontalSlider.value = 12.0;
			specialInstantStopCheckbox.checked = true;
			break;
	}

	velocityVerticalSlider.dispatchEvent(new Event('input'));
	velocityHorizontalSlider.dispatchEvent(new Event('input'));
	accelVerticalSlider.dispatchEvent(new Event('input'));
	accelHorizontalSlider.dispatchEvent(new Event('input'));

	saveSettings();
});

// Settings
if (settings.hasOwnProperty('{uniqueID}')) {
	const loaded_data = settings['{uniqueID}'];
	topic = loaded_data.topic;

	typedict = loaded_data.typedict ?? {};
	joy_offset_x = loaded_data.joy_offset_x;
	joy_offset_y = loaded_data.joy_offset_y;

	opacityBox.value = loaded_data.opacity ?? 0.5;
	opacityValue.textContent = opacityBox.value;

	colourpickerBox.value = loaded_data.color ?? "#000000";
	updateColor(colourpickerBox.value, opacityBox.value);

	// parse legacy configs
	if(loaded_data.linear_velocity){

		velocityVerticalSlider.value = loaded_data.linear_velocity;
		velocityHorizontalSlider.value = loaded_data.angular_velocity;

		accelVerticalSlider.value = loaded_data.accel * 20 ?? 2.0; //unit fix for old 20 hz speedup
		accelHorizontalSlider.value = loaded_data.accel * 20 ?? 2.0;

		if(loaded_data.holonomic_swap){
			axisHorizontalBox.value = "linear_y";
			axisVerticalBox.value = "linear_x";
		}else{
			axisHorizontalBox.value = "angular_z";
			axisVerticalBox.value = "linear_x";
		}

		specialInstantStopCheckbox.checked = false;
		specialAckermannCheckbox.checked = loaded_data.invert_angular;
		invertVerticalCheckbox.checked = false;
		invertHorizontalCheckbox.checked = false;
		specialKeyboardCheckbox.checked = false;
		saveSettings();

	}else{

		axisHorizontalBox.value = loaded_data.axis_horiz ?? "angular_z";
		axisVerticalBox.value = loaded_data.axis_vert ?? "linear_x";

		velocityVerticalSlider.value = loaded_data.vel_vert ?? 1.0;
		velocityHorizontalSlider.value = loaded_data.vel_horiz ?? 2.0;

		accelVerticalSlider.value = loaded_data.accel_vert ?? 2.0;
		accelHorizontalSlider.value = loaded_data.accel_horiz ?? 4.0;

		invertVerticalCheckbox.checked = loaded_data.invert_vert ?? false;
		invertHorizontalCheckbox.checked = loaded_data.invert_horiz ?? false;
		specialAckermannCheckbox.checked = loaded_data.ackermann_emulation ?? true;
		specialInstantStopCheckbox.checked = loaded_data.instant_stop ?? false;
		specialKeyboardCheckbox.checked = loaded_data.keyboard_control ?? false;

		specialKeyboardCheckbox.dispatchEvent(new Event('input'));
	}

	velocityVerticalSlider.dispatchEvent(new Event('input'));
	velocityHorizontalSlider.dispatchEvent(new Event('input'));
	accelVerticalSlider.dispatchEvent(new Event('input'));
	accelHorizontalSlider.dispatchEvent(new Event('input'));

	if(loaded_data.keybindings){
		keybindings = loaded_data.keybindings;
		updateKeyboardSetup();
	}

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

		color: colourpickerBox.value,
		opacity: opacityBox.value,

		axis_horiz: axisHorizontalBox.value,
		axis_vert: axisVerticalBox.value,

		vel_vert: parseFloat(velocityVerticalSlider.value),
		vel_horiz: parseFloat(velocityHorizontalSlider.value),

		accel_vert: parseFloat(accelVerticalSlider.value),
		accel_horiz: parseFloat(accelHorizontalSlider.value),

		invert_vert: invertVerticalCheckbox.checked,
		invert_horiz: invertHorizontalCheckbox.checked,

		ackermann_emulation: specialAckermannCheckbox.checked,
		instant_stop: specialInstantStopCheckbox.checked,
		keyboard_control: specialKeyboardCheckbox.checked,

		keybindings: keybindings,

		//deprecated legacy stuff
		//linear_velocity: parseFloat(linearVelSlider.value),
		//angular_velocity: parseFloat(angularVelSlider.value),
		//accel: parseFloat(accelSlider.value),
		//invert_angular: invertAngularCheckbox.checked,
		//holonomic_swap: holonomicSwapCheckbox.checked,

		joy_offset_x: joy_offset_x,
		joy_offset_y: joy_offset_y,

		typedict: typedict
	};
	settings.save();
	updateKeyButtons();
}

// Topic and connections

async function loadTopics(){
	let twist_topics = await rosbridge.get_topics("geometry_msgs/Twist");
	let stamped_topics = await rosbridge.get_topics("geometry_msgs/TwistStamped");

	let topiclist = "";
	twist_topics.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+" (Twist)</option>";
		typedict[element] = "geometry_msgs/Twist";
	});
	stamped_topics.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+" (TwistStamped)</option>";
		typedict[element] = "geometry_msgs/TwistStamped";
	});

	selectionbox.innerHTML = topiclist;

	if(topic == "")
		topic = selectionbox.value;
	else{
		if(twist_topics.includes(topic) || stamped_topics.includes(topic)){
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
		messageType : typedict[topic],
		queue_size: 1
	});

	stamp_seq = 0;
}


function publishTwist(x, y, z, wx, wy, wz) {

	function getStamp(){
		const currentTime = new Date();
		const currentTimeSecs = Math.floor(currentTime.getTime() / 1000);
		const currentTimeNsecs = (currentTime.getTime() % 1000) * 1e6;

		return {
			secs: currentTimeSecs,
			nsecs: currentTimeNsecs
		}
	}

	function getTwist(x, y, z, wx, wy, wz){
		return new ROSLIB.Message({
			linear: {
				x: x,
				y: y,
				z: z
			},
			angular: {
				x: wx,
				y: wy,
				z: wz
			}
		});
	}

	function getTwistStamped(x, y, z, wx, wy, wz){
		stamp_seq++;
		return new ROSLIB.Message({
			header: {
				seq: stamp_seq,
				stamp: getStamp(),
				frame_id: tf.fixed_frame
			},
			twist: getTwist(x, y, z, wx, wy, wz)
		});
	}

	if(typedict[topic] == "geometry_msgs/Twist"){
		cmdVelPublisher.publish(getTwist(x, y, z, wx, wy, wz));
	}else{
		cmdVelPublisher.publish(getTwistStamped(x, y, z, wx, wy, wz));
	}
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

click_icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

// Teleop logic
let horiz_vel = 0;
let vert_vel = 0;

let horiz_target = 0;
let vert_target = 0;

//for accelerating smoothly
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
		color: colourpickerBox.value,
		restOpacity: parseFloat(opacityBox.value)+0.3
	})
}

let joystick = makeJoystick();

function addJoystickListeners(){
	joystick.on('move', onJoystickMove);
	joystick.on('touchmove', onJoystickMove);
	joystick.on('end', onJoystickEnd);
	joystick.on('touchend', onJoystickEnd);
}

function mapAndSend(){
	const cfg = settings['{uniqueID}'];

	let vert_move = cfg.invert_vert ? -vert_vel : vert_vel;
	let horiz_move = cfg.invert_horiz ? -horiz_vel: horiz_vel;

	let x = 0;
	let y = 0;
	let z = 0;
	let wx = 0;
	let wy = 0;
	let wz = 0;

	switch (cfg.axis_horiz) {
		case "linear_x": x += horiz_move; break;
		case "linear_y": y += horiz_move; break;
		case "linear_z": z += horiz_move; break;
		case "angular_x": wx += horiz_move; break;
		case "angular_y": wy += horiz_move; break;
		case "angular_z": wz += horiz_move; break;
	}

	switch (cfg.axis_vert) {
		case "linear_x": x += vert_move; break;
		case "linear_y": y += vert_move; break;
		case "linear_z": z += vert_move; break;
		case "angular_x": wx += vert_move; break;
		case "angular_y": wy += vert_move; break;
		case "angular_z": wz += vert_move; break;
	}

	publishTwist(x, y, z, wx, wy, wz);
}

function integrateAcceleration(){
	const cfg = settings['{uniqueID}'];
	const accel_vert = cfg.accel_vert / 20.0;
	const accel_horiz = cfg.accel_horiz / 20.0;

	if(cfg.accel_vert == 12.0){
		vert_vel = vert_target;
	}else{
		if(vert_vel != vert_target){
			if(vert_vel < vert_target){
				vert_vel += accel_vert;
	
				if(vert_vel > vert_target)
					vert_vel = vert_target;
			}
			else if(vert_vel > vert_target){
				vert_vel -= accel_vert;
	
				if(vert_vel < vert_target)
					vert_vel = vert_target;
			}
		}
	}


	if(cfg.accel_horiz == 12.0){
		horiz_vel = horiz_target;
	}else{
		if(horiz_vel != horiz_target){
			if(horiz_vel < horiz_target){
				horiz_vel += accel_horiz;
	
				if(horiz_vel > horiz_target)
					horiz_vel = horiz_target;
			}
			else if(horiz_vel > horiz_target){
				horiz_vel -= accel_horiz;
	
				if(horiz_vel < horiz_target)
					horiz_vel = horiz_target;
			}
		}
	}
}

function joystickStop(){
	vert_vel = 0;
	horiz_vel = 0;
	vert_target = 0;
	horiz_target = 0;
	publishTwist(0, 0, 0, 0, 0, 0);

	if(interval !== undefined){
		clearInterval(interval);
		interval = undefined;
	}
}

function onJoystickMove(event, data) {
	const cfg = settings['{uniqueID}'];
	const force = Math.min(Math.max(data.force, 0.0), 1.0);

	vert_target = cfg.vel_vert * Math.sin(data.angle.radian) * force;
	horiz_target = -cfg.vel_horiz * Math.cos(data.angle.radian) * force;

	if (cfg.ackermann_emulation && vert_target < 0) {
		horiz_target = -horiz_target;
	}

	if(interval === undefined){
		interval = setInterval(() => {

			integrateAcceleration();

			if(Math.abs(vert_vel) < 0.005 && Math.abs(horiz_vel) < 0.005){
				joystickStop();
				return;
			}
		
			mapAndSend();
			
		}, 1000 / 20); //20 hz standard
	}
};

function onJoystickEnd(event) {
	vert_target = 0;
	horiz_target = 0;

	if(settings['{uniqueID}'].instant_stop){
		//pull the parking brake
		joystickStop();
	}
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
	
		addJoystickListeners();
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


// --- keyboard control ---

const buttons = document.querySelectorAll('#{uniqueID}_keyboard_section .key_bind_button');
buttons.forEach(button => {
	button.addEventListener('click', (e) => {
		e.preventDefault();
		
		// Clear any existing listener
		if (new_keybind_listener) {
			new_keybind_listener.classList.remove("listening");
		}
		
		button.classList.add('listening');
		button.textContent = '?';
		
		new_keybind_listener = button;
	});
});

function updateKeyboardSetup(){
	const key_checkbox = document.getElementById(`{uniqueID}_special_keyboard`);
	const key_section = document.getElementById(`{uniqueID}_keyboard_section`);

	if (key_checkbox.checked) {
		key_section.style.display = 'block';
		enableKeyboard();
	} else {
		key_section.style.display = 'none';
		disableKeyboard();
	}
}

function updateKeyButtons() {    

	function getActiveKeybinds(){
		let list = [];

		if(axisHorizontalBox.value != "none"){
			list.push(keybindings.key_right);
			list.push(keybindings.key_left);
		}

		if(axisVerticalBox.value != "none"){
			list.push(keybindings.key_up);
			list.push(keybindings.key_down);
		}

		return list;
	}

	active_keybinds = getActiveKeybinds();

	function keyCodeToDisplay(keyCode) {
		const mappings = {
			" ": "SPACE",
			"arrowleft": "←",
			"arrowright": "→",
			"arrowup": "↑",
			"arrowdown": "↓",
		};

		if(keyCode in mappings)
			return mappings[keyCode];
		return keyCode.replace('Key', '').replace('Digit', '').toUpperCase();
	}

	if(axisHorizontalBox.value == "none"){
		button_key_right.textContent = "";
		button_key_left.textContent = "";
		button_key_right.disabled = true;
		button_key_left.disabled = true;
	}else{
		button_key_right.textContent = keyCodeToDisplay(keybindings.key_right);
		button_key_left.textContent = keyCodeToDisplay(keybindings.key_left);
		button_key_right.disabled = false;
		button_key_left.disabled = false;
	}

	if(axisVerticalBox.value == "none"){
		button_key_up.textContent = "";
		button_key_down.textContent = "";
		button_key_up.disabled = true;
		button_key_down.disabled = true;
	}else{
		button_key_up.textContent = keyCodeToDisplay(keybindings.key_up);
		button_key_down.textContent = keyCodeToDisplay(keybindings.key_down);
		button_key_up.disabled = false;
		button_key_down.disabled = false;
	}


	if (new_keybind_listener) {
		new_keybind_listener.classList.remove("listening");
		new_keybind_listener = null;
	}
}

function keydown(event) {

	if (new_keybind_listener) {
		event.preventDefault();

		const id = new_keybind_listener.id.replace("{uniqueID}_","");
		keybindings[id] = event.key.toLowerCase();
		updateKeyButtons();
		saveSettings();
		return;
	}

	//skip if it's not our key, so we don't fight other joystick widgets
	if(!active_keybinds.includes(event.key.toLowerCase()))
		return;

	switch(event.key.toLowerCase()) {
		case keybindings.key_up:
			key_move.vert = 1.0;         
			key_move.up = true;    
			break;

		case keybindings.key_down: 
			key_move.vert = -1.0;      
			key_move.down = true;
			break;

		case keybindings.key_left:
			key_move.horiz = -1.0; 
			key_move.left = true;
			break;

		case keybindings.key_right: 
			key_move.horiz = 1.0;     
			key_move.right = true;   
			break;
	}

	if(keyboard_interval === undefined){
		setKeyboardInterval();
	}
}

function keyup(event) {

	if(!active_keybinds.includes(event.key.toLowerCase()))
		return;

	switch(event.key.toLowerCase()) {
		case keybindings.key_up:
			key_move.vert = 0.0;
			key_move.up = false;
			break;

		case keybindings.key_down: 
			key_move.vert = 0.0;      
			key_move.down = false;
			break;

		case keybindings.key_left:
			key_move.horiz = 0.0; 
			key_move.left = false;
			break;

		case keybindings.key_right: 
			key_move.horiz = 0.0;     
			key_move.right = false;
			break;
	}

	const vertical_motion = key_move.up || key_move.down;
	const horizontal_motion = key_move.left || key_move.right;
	const motion = key_move.up || key_move.down || key_move.left || key_move.right;

	if(settings['{uniqueID}'].instant_stop){

		if(!vertical_motion){
			vert_target = 0;
			vert_vel = 0;
		}

		if(!horizontal_motion){
			horiz_target = 0;
			horiz_vel = 0;
		}
		
		if(!motion)
			keyboardStop();
	}
}

function keyboardStop(){
	vert_target = 0;
	horiz_target = 0;
	horiz_vel = 0;
	vert_vel = 0;
	publishTwist(0, 0, 0, 0, 0, 0);
	if(keyboard_interval !== undefined){
		clearInterval(keyboard_interval);
		keyboard_interval = undefined;
	}
}

function setKeyboardInterval() {
	const loop = () => {
		const cfg = settings['{uniqueID}'];
		vert_target = cfg.vel_vert * key_move.vert;
		horiz_target = -cfg.vel_horiz * key_move.horiz;

		if (cfg.ackermann_emulation && vert_target < 0) {
			horiz_target = -horiz_target;
		}
		integrateAcceleration();

		if (Math.abs(vert_vel) < 0.005 && Math.abs(horiz_vel) < 0.005) {
			keyboardStop();
			return;
		}

		mapAndSend();
	};

	loop();
	keyboard_interval = setInterval(loop, 1000 / 20);//20 hz standard 
}

function enableKeyboard() {
	document.addEventListener('keydown', keydown);
	document.addEventListener('keyup', keyup);
	updateKeyButtons();
}

function disableKeyboard() {    
	document.removeEventListener('keydown', keydown);
	document.removeEventListener('keyup', keyup);
	if (keyboard_interval !== undefined) {
		clearInterval(keyboard_interval);
		keyboard_interval = undefined;
	}
}

if(settings['{uniqueID}'].keyboard_control){
	enableKeyboard();
}

console.log("Teleop Widget Loaded {uniqueID}")
