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

let range_topic = undefined;
let listener = undefined;

let data = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

const text_angle = document.getElementById("{uniqueID}_angle_text");
const text_frame = document.getElementById("{uniqueID}_frame_text");
const text_angleinc = document.getElementById("{uniqueID}_angleinc_text");
const text_pointscount = document.getElementById("{uniqueID}_points_text");
const text_scantime = document.getElementById("{uniqueID}_scan_time_text");
const text_min = document.getElementById("{uniqueID}_rangemin_text");
const text_max = document.getElementById("{uniqueID}_rangemax_text");

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');

function setOpacityText(val){
	if(val == 0.0)
		opacityValue.textContent = "0.0 (Scan rendering disabled)";
	else
		opacityValue.textContent = val;
}

opacitySlider.addEventListener('input', () =>  {
	setOpacityText(opacitySlider.value);
	saveSettings();
	drawScan();
});

const thicknessSlider = document.getElementById('{uniqueID}_thickness');
const thicknessValue = document.getElementById('{uniqueID}_thickness_value');
thicknessSlider.addEventListener('input', () =>  {
	thicknessValue.textContent = thicknessSlider.value;
	saveSettings();
	drawScan();
});

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
	drawScan();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	opacitySlider.value = loaded_data.opacity;
	setOpacityText(loaded_data.opacity);

	thicknessSlider.value = loaded_data.thickness;
	thicknessValue.innerText = loaded_data.thickness;

	colourpicker.value = loaded_data.color;
	throttle.value = loaded_data.throttle;
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
		opacity: opacitySlider.value,
		thickness: thicknessSlider.value,
		color: colourpicker.value,
		throttle: throttle.value
	}
	settings.save();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

async function drawScan() {

	const wid = canvas.width;
	const hei = canvas.height;

	ctx.setTransform(1,0,0,1,0,0);
	ctx.clearRect(0, 0, wid, hei);
	ctx.globalAlpha = opacitySlider.value;
	ctx.fillStyle = colourpicker.value;

	if(data == undefined || opacitySlider.value == 0.0){
		return;
	}

	const unit = view.getMapUnitsInPixels(1.0);
	const pixel = view.getMapUnitsInPixels(thicknessSlider.value);

	let pos = view.fixedToScreen({
		x: data.pose.translation.x,
		y: data.pose.translation.y,
	});

	ctx.setTransform(1,0,0,-1,pos.x, pos.y); //sx,0,0,sy,px,py
	const delta = parseInt(pixel/2);

	ctx.beginPath();
	for(let i = 0; i < data.points.length; i++){
		const x = data.points[i].x * unit - delta;
		const y = data.points[i].y * unit - delta;
		ctx.moveTo(x, y);
		ctx.lineTo(x + pixel, y);
		ctx.lineTo(x + pixel, y + pixel);
		ctx.lineTo(x, y + pixel);
		ctx.lineTo(x, y);
	}
	ctx.fill();
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawScan();
}

window.addEventListener("tf_fixed_frame_changed", drawScan);
window.addEventListener("view_changed", drawScan);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

//Topic

function radToDeg(val){
	return (val * (180/Math.PI)).toFixed(2)
}

function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}
	
	if(range_topic !== undefined){
		range_topic.unsubscribe(listener);
	}

	range_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/LaserScan',
		throttle_rate: parseInt(throttle.value),
		compression: "cbor"
	});

	status.setWarn("No data received.");
	text_angle.innerText = "Angle: ?";
	text_frame.innerText = "TF Frame: ?";
	text_angleinc.innerText = "Angle increment: ?";
	text_pointscount.innerText = "Points: ?";
	text_scantime.innerText = "Scan time: ?";
	text_min.innerText = "Min: ?";
	text_max.innerText = "Max: ?";

	listener = range_topic.subscribe((msg) => {	

		let error = false;
		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
			msg.header.frame_id = tf.fixed_frame;
			error = true;
		}

		const pose = tf.absoluteTransforms[msg.header.frame_id];

		if(!pose){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

		text_angle.innerText = "Angle: "+radToDeg(msg.angle_min)+"°"+" to "+radToDeg(msg.angle_max)+"°";
		text_angleinc.innerText = "Angle increment: "+radToDeg(msg.angle_increment)+"°";
		text_frame.innerText = "TF frame: "+msg.header.frame_id;
		text_pointscount.innerText = "Points: "+msg.ranges.length;
		text_scantime.innerText = "Scan time: "+msg.scan_time.toFixed(5)+" s";
		text_min.innerText = "Min: "+msg.range_min.toFixed(2)+" m";
		text_max.innerText = "Max: "+msg.range_max.toFixed(2)+" m";

		let rotatedPointCloud = [];
		msg.ranges.forEach(function (item, index) {
			if (item >= msg.range_min && item <= msg.range_max) {
				const angle = msg.angle_min + index * msg.angle_increment;
				rotatedPointCloud.push(tfModule.applyRotation(
					{
						x: item * Math.cos(angle), 
						y: item * Math.sin(angle), 
						z: 0 
					}, 
					pose.rotation, 
					false
				));
			}
		});

		data = {};
		data.pose = pose;
		data.points = rotatedPointCloud
		drawScan();

		if(!error){
			status.setOK();
		}
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/LaserScan");
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
	data = undefined;
	connect();
});

selectionbox.addEventListener("click", connect);
click_icon.addEventListener("click", loadTopics);

loadTopics();
resizeScreen();

console.log("Laserscan Widget Loaded {uniqueID}")