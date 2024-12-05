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

let seq = 0;

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
}else{
	saveSettings();
}

if(topic == ""){
	topic = "/initialpose";
	status.setWarn("No topic found, defaulting to /initialpose");
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic
	}
	settings.save();
}

function sendMessage(pos, delta){
	if(!pos || !delta){
		status.setError("Could not send message, pose invalid.");
		return;
	}

	let yaw = Math.atan2(delta.y, -delta.x);
	let quat = Quaternion.fromEuler(yaw, 0, 0, 'ZXY');

	let map_pos = view.screenToFixed(pos);

	const currentTime = new Date();
	const currentTimeSecs = Math.floor(currentTime.getTime() / 1000);
	const currentTimeNsecs = (currentTime.getTime() % 1000) * 1e6;

	const publisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: 'geometry_msgs/PoseWithCovarianceStamped',
	});

	const poseMessage = new ROSLIB.Message({
		header: {
			seq: seq++,
			stamp: {
				secs: currentTimeSecs,
      			nsecs: currentTimeNsecs
			},
			frame_id: tf.fixed_frame
		},
		pose: {
			pose: {
				position: {
					x: map_pos.x,
					y: map_pos.y,
					z: 0.0
				},
				orientation: {
					x: quat.x,
					y: quat.y,
					z: quat.z,
					w: quat.w
				}
			},
			covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
		}
	});	
	publisher.publish(poseMessage);

	status.setOK();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const view_container = document.getElementById("view_container");
const icon = document.getElementById("{uniqueID}_icon");

let active = false;
let sprite = new Image();
let start_point = undefined;
let delta = undefined;
sprite.src = "assets/initialpose.png";

function drawArrow() {
    const wid = canvas.width;
    const hei = canvas.height;

	ctx.setTransform(1,0,0,1,0,0);
    ctx.clearRect(0, 0, wid, hei);

	if(delta){
		let ratio = sprite.naturalHeight/sprite.naturalWidth;
		ctx.setTransform(1,0,0,1,start_point.x, start_point.y); //sx,0,0,sy,px,py
		ctx.rotate(Math.atan2(-delta.y, -delta.x));
		ctx.drawImage(sprite, -80, -80*ratio, 160, 160*ratio);
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
	delta = {
		x: start_point.x - clientX,
		y: start_point.y - clientY,
	};

	drawArrow();	
}

function endDrag(event){
	sendMessage(start_point, delta);

	start_point = undefined;
	delta = undefined;
	drawArrow();
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
	let result = await rosbridge.get_topics("geometry_msgs/PoseWithCovarianceStamped");

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
	status.setOK();
});

loadTopics();

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

console.log("Initialpose Widget Loaded {uniqueID}")