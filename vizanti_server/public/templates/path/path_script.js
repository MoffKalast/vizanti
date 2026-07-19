let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);

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

let listener = undefined;
let path_topic = undefined;

let pose_array = undefined;

const text_frameid = document.getElementById("{uniqueID}_frame_text");
const text_point_count = document.getElementById("{uniqueID}_points_text");
const text_total_dist = document.getElementById("{uniqueID}_distance_text");

const selectionbox = document.getElementById("{uniqueID}_topic");
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
	drawPath();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');

function setOpacityText(val){
	if(val == 0.0)
		opacityValue.textContent = "0.0 (Path rendering disabled)";
	else
		opacityValue.textContent = val;
}

opacitySlider.addEventListener('input', () =>  {
	setOpacityText(opacitySlider.value);
	saveSettings();
	drawPath();
});


//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	opacitySlider.value = loaded_data.opacity ?? 1.0;
	setOpacityText(loaded_data.opacity);

	colourpicker.value = loaded_data.color ?? "#54db67";
	throttle.value = loaded_data.throttle ?? 100;
}else{
	saveSettings();
}

//update the icon colour when it's loaded or when the image source changes
icon.onload = () => {
	utilModule.setIconColor(icon, colourpicker.value);
};
if (icon.contentDocument) {
	utilModule.setIconColor(icon, colourpicker.value);
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		color: colourpicker.value,
		throttle: throttle.value,
		opacity: opacitySlider.value
	}
	settings.save();
}

function getDistance(posearray) {
    if (!Array.isArray(posearray) || posearray.length < 2)
		return 0;
    
    let dist = 0;
    for (let i = 0; i < posearray.length - 1; i++) {
        const pose1 = posearray[i]?.pose?.position;
        const pose2 = posearray[i + 1]?.pose?.position;
        
        // Skip this pair if either point is missing
        if (!pose1 || !pose2) continue;
        
        const dx = pose2.x - pose1.x;
        const dy = pose2.y - pose1.y;
        const dz = pose2.z - pose1.z;
        
        dist += Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    return dist;
}

//Rendering
async function drawPath(){

	const wid = canvas.width;
    const hei = canvas.height;
	ctx.clearRect(0, 0, wid, hei);

	ctx.globalAlpha = opacitySlider.value;

	if(pose_array === undefined || pose_array.length < 2 || opacitySlider.value == 0.0){
		return false;
	}

	ctx.lineWidth = 2;
	ctx.strokeStyle = colourpicker.value;
	ctx.beginPath();

	const firstPos = view.fixedToScreen({
		x: pose_array[0].translation.x,
		y: pose_array[0].translation.y
	});
	ctx.moveTo(firstPos.x, firstPos.y);

	for (let i = 1; i < pose_array.length; i++) {
		const point = pose_array[i];
		const pos = view.fixedToScreen({
			x: point.translation.x,
			y: point.translation.y
		});
		ctx.lineTo(pos.x, pos.y);
	}

	ctx.stroke();
}

//Topic
function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(path_topic !== undefined){
		path_topic.unsubscribe(listener);
	}

	path_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'nav_msgs/msg/Path',
		throttle_rate: parseInt(throttle.value),
		compression: rosbridge.compression,
		queue_length: 1
	});

	status.setWarn("No data received.");
	
	listener = path_topic.subscribe((msg) => {
		
		let error = false;
		let newposes = [];

		if(msg.poses == undefined){
			status.setWarn("Received uninitialized list of poses. Wat.");
			error = true;
			return;
		}

		msg.poses.forEach((point, index) => {

			if(point.header.frame_id == ""){
				status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
				point.header.frame_id = tf.fixed_frame;
				error = true;
			}
	
			if(!tf.absoluteTransforms[point.header.frame_id]){
				status.setError("Required transform frame \""+point.header.frame_id+"\" not found.");
				error = true;
				return;
			}
	
			newposes.push(tf.transformPoseStamped(
				point.header, 
				point.pose.position, 
				point.pose.orientation
			));
		});

		text_frameid.innerText = "Frame: "+msg.header.frame_id;
		text_point_count.innerText = "Points: "+msg.poses.length;

		let dist = getDistance(msg.poses)

		if(dist > 1000.0){
			dist /= 1000.0
			text_total_dist.innerText = "Distance: "+dist.toFixed(3)+" km";
		}else if(dist < 1.0){
			dist *= 100.0
			text_total_dist.innerText = "Distance: "+dist.toFixed(1)+" cm";
		}else{
			text_total_dist.innerText = "Distance: "+dist.toFixed(2)+" m";
		}

		pose_array = newposes;
		drawPath();

		if(!error){
			status.setOK();
		}
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("nav_msgs/msg/Path");

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

click_icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawPath();
}

window.addEventListener("tf_fixed_frame_changed", connect);
window.addEventListener("view_changed", drawPath);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("MarkerArray Widget Loaded {uniqueID}")

