let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let dbModule = await import(`${base_url}/js/modules/database.js`);

let view = viewModule.view;
let tf = tfModule.tf;
let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let Status = StatusModule.Status;

const db = new dbModule.IndexedDatabase('odom_history');
await db.openDB();
const DB_KEY = "odom_pose_history_{uniqueID}";

let topic = getTopic("{uniqueID}");

if(topic != "")
	topic += " (Odometry)";

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let listener = undefined;
let odom_topic = undefined;

let sample_array = [];

let mode = "" //see setMode()
let raw_target = "";

const text_point_count = document.getElementById("{uniqueID}_points_text");
const text_total_dist = document.getElementById("{uniqueID}_distance_text");

const selectionbox = document.getElementById("{uniqueID}_topic");
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const drawarrows = document.getElementById('{uniqueID}_draw_arrows');
drawarrows.addEventListener('change', ()=>{
	saveSettings();
	drawHistory();
});

const drawpath = document.getElementById('{uniqueID}_draw_path');
drawpath.addEventListener('change', ()=>{
	saveSettings();
	drawHistory();
});

const save_history = document.getElementById('{uniqueID}_save_history');
save_history.addEventListener('change', ()=>{
	saveSettings();
	drawHistory();
});

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
	drawHistory();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

const historypicker = document.getElementById('{uniqueID}_history');
historypicker.addEventListener("input", (event) =>{
	saveSettings();

	while (sample_array.length > parseInt(historypicker.value)) {
		sample_array.shift();
	}

	drawHistory();
});

const clearHistoryButton = document.getElementById("{uniqueID}_clearhistory");
clearHistoryButton.addEventListener('click', ()=>{
	sample_array = [];
	db.setObject(DB_KEY, null);
	updateTextDisplay();
	drawHistory();
});

const downloadCSVButton = document.getElementById("{uniqueID}_downloadcsv");

downloadCSVButton.addEventListener('click', () => {

	function getCurrentDateTimeString() {
		const date = new Date();
		const year = date.getFullYear();
		const month = (date.getMonth() + 1).toString().padStart(2, '0');
		const day = date.getDate().toString().padStart(2, '0');
		const hours = date.getHours().toString().padStart(2, '0');
		const minutes = date.getMinutes().toString().padStart(2, '0');

		return `${year}-${month}-${day}-${hours}-${minutes}`;
	}

	if(sample_array.length === 0){
		alert("No poses to export.");
		return;
	}

	let csv = "x,y,yaw\n";

	for(const pose of sample_array){
		csv += `${pose.x},${pose.y},${pose.yaw}\n`;
	}

	const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });

	const url = URL.createObjectURL(blob);

	const link = document.createElement('a');
	link.href = url;

	const safeTopic = `${raw_target}_in_${tf.fixed_frame}_`.replace(/[^\w\d_-]/g, "_")+getCurrentDateTimeString();
	link.download = `${safeTopic || "odom_history"}.csv`;

	document.body.appendChild(link);
	link.click();
	document.body.removeChild(link);

	URL.revokeObjectURL(url);

	status.setOK("CSV downloaded.");
});

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data = settings["{uniqueID}"];
	topic = loaded_data.topic;

	historypicker.value = loaded_data.history;
	drawarrows.checked = loaded_data.draw_arrows;
	drawpath.checked = loaded_data.draw_path;
	throttle.value = loaded_data.throttle;
	colourpicker.value = loaded_data.color ?? "#54db67";
	save_history.checked = loaded_data.save_history ?? true;
	setMode();
}else{
	saveSettings();
}

if(save_history.checked){
	await loadPoints();
}

//update the icon colour when it's loaded or when the image source changes
icon.onload = () => {
	utilModule.setIconColor(icon, colourpicker.value);
};
if (icon.contentDocument) {
	utilModule.setIconColor(icon, colourpicker.value);
}

function saveSettings(){
	setMode();
	settings["{uniqueID}"] = {
		topic: topic,
		history: historypicker.value,
		color: colourpicker.value,
		throttle: throttle.value,
		draw_arrows: drawarrows.checked,
		draw_path: drawpath.checked,
		save_history: save_history.checked
	}
	settings.save();
}

async function loadPoints(){
	const stored = await db.getObject(DB_KEY);
	if(stored instanceof Float32Array && stored.length % 3 === 0){
		for(let i = 0; i < stored.length; i += 3){
			sample_array.push({ x: stored[i], y: stored[i+1], yaw: stored[i+2] });
		}
		drawHistory();
	}
}

function savePoints(){
	const packed = new Float32Array(sample_array.length * 3);
	for(let i = 0; i < sample_array.length; i++){
		packed[i*3] = sample_array[i].x;
		packed[i*3+1] = sample_array[i].y;
		packed[i*3+2] = sample_array[i].yaw;
	}
	db.setObject(DB_KEY, packed);
}

//Rendering
async function drawHistory(){

	function drawArrow(height, tipwidth) {
		const half = height/2;
		ctx.moveTo(-half, -tipwidth); 
		ctx.lineTo(half, 0);
		ctx.lineTo(-half, tipwidth);
	}

	const wid = canvas.width;
    const hei = canvas.height;
	ctx.setTransform(1,0,0,1,0,0); 
	ctx.clearRect(0, 0, wid, hei);

	if(!drawarrows.checked && !drawpath.checked)
		return;

	if(sample_array.length < 2){
		status.setWarn("No data yet!");
		return;
	}

	ctx.lineWidth = 3;
	ctx.strokeStyle = colourpicker.value;
	ctx.fillStyle = colourpicker.value;
	ctx.beginPath();

	let view_points = [];
	for (let i = 0; i < sample_array.length; i++) {
		view_points[i] = view.fixedToScreen(sample_array[i]);
		view_points[i].yaw = sample_array[i].yaw;
	}

	//continuous line 
	if(drawpath.checked){
		ctx.moveTo(view_points[0].x, view_points[0].y);
		for (let i = 1; i < view_points.length; i++) {
			ctx.lineTo(view_points[i].x, view_points[i].y);
		}
		ctx.globalAlpha = 0.6;
		ctx.stroke();
	}

	//yaw indicator
	if(drawarrows.checked){
		ctx.beginPath();

		let prev_p = null;
		for (let i = 0; i < view_points.length; i++) {
			const p = view_points[i];
			if(prev_p === null || Math.hypot(p.x - prev_p.x, p.y - prev_p.y) > 20){
				ctx.setTransform(1,0,0,-1,p.x, p.y); //sx,0,0,sy,px,py
				ctx.rotate(p.yaw);
				drawArrow(15, 5);
				prev_p = p;
			}
		}

		ctx.globalAlpha = 1.0;
		ctx.fill();
	}
}


function updateTextDisplay(){

	function getDistance(posearray) {
		if (!Array.isArray(posearray) || posearray.length < 2)
			return 0;
		
		let dist = 0;
		for (let i = 0; i < posearray.length - 1; i++) {
			const pose1 = posearray[i];
			const pose2 = posearray[i + 1];
			const dx = pose2.x - pose1.x;
			const dy = pose2.y - pose1.y;
			dist += Math.sqrt(dx * dx + dy * dy);
		}
		return dist;
	}

	text_point_count.innerText = "Points: "+sample_array.length;
	let dist = getDistance(sample_array)

	if(dist > 1000.0){
		dist /= 1000.0
		text_total_dist.innerText = "Distance: "+dist.toFixed(3)+" km";
	}else if(dist < 1.0){
		dist *= 100.0
		text_total_dist.innerText = "Distance: "+dist.toFixed(1)+" cm";
	}else{
		text_total_dist.innerText = "Distance: "+dist.toFixed(2)+" m";
	}
}

let time_since_updated = Date.now();
function appendPose(pose){
	const pose2D = {
		x: pose.translation.x,
		y: pose.translation.y,
		yaw: pose.rotation.toEuler().h
	};

	if(sample_array.length > 0){
		const last = sample_array[sample_array.length-1];
		const delta = Math.hypot(last.x - pose2D.x, last.y - pose2D.y);
		if(delta > 0.03){
			sample_array.push(pose2D);
		}else{
			return false;
		}
	}else{
		sample_array.push(pose2D);
	}

	while (sample_array.length > parseInt(historypicker.value)) {
		sample_array.shift();
	}

	const now = Date.now();
	if(now - time_since_updated > 3000){
		updateTextDisplay();

		if(save_history.checked){
			savePoints();
		}

		time_since_updated = now;
	}

	return true;
}

//Topic
function connect(){
	if(odom_topic !== undefined){
		odom_topic.unsubscribe(listener);
	}

	if(mode != "topic")
		return;

	if(topic == ""){
		status.setError("No target.");
		return;
	}

	odom_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : raw_target,
		messageType : 'nav_msgs/Odometry',
		throttle_rate: parseInt(throttle.value),
		compression: rosbridge.compression,
		queue_length: 1
	});

	status.setWarn("No data received.");
	
	listener = odom_topic.subscribe((msg) => {
		
		let error = false;
		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
			msg.header.frame_id = tf.fixed_frame;
			error = true;
		}

		const frame = tf.absoluteTransforms[msg.header.frame_id];

		if(!frame){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			error = true;
			return;
		}

		const transformed = tf.transformPose(
			msg.header.frame_id,
			tf.fixed_frame, 
			msg.pose.pose.position, 
			msg.pose.pose.orientation
		)

		if(appendPose(transformed)){
			drawHistory();
			if(!error){
				status.setOK();
			}
		}
	});

	saveSettings();
}

async function loadTopics(){
	let odom_array = await rosbridge.get_topics("nav_msgs/Odometry");
	let tf_array = Array.from(tf.frame_list);

	let topiclist = "";
	odom_array.forEach(element => {
		topiclist += "<option value='"+element+" (Odometry)'>"+element+" (Odometry)</option>"
	});

	tf_array.forEach(frame => {
		topiclist += "<option value='"+frame+" (Frame)'>"+frame+" (Frame)</option>"
	});

	selectionbox.innerHTML = topiclist

	if(topic == ""){
		topic = selectionbox.value;
		setMode();
	}else{
		setMode();
		if(odom_array.includes(raw_target) || tf_array.includes(raw_target)){
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
	sample_array = [];
	saveSettings();
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

click_icon.addEventListener("click", (event) => {
	loadTopics();
	updateTextDisplay();
});

loadTopics();

function setMode(){
	if(topic.endsWith("(Frame)")){
		mode = "tf";
		raw_target = topic.replace(" (Frame)", "");
	}else if(topic.endsWith("(Odometry)")){
		mode = "topic";
		raw_target = topic.replace(" (Odometry)", "");
	}else{
		mode = "";
		raw_target = "";
	}
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawHistory();
}

let tf_throttle_stamp = 0;
window.addEventListener("tf_changed", ()=>{
	if(mode == "tf"){
		const now = Date.now();
		if(now - tf_throttle_stamp >= parseInt(throttle.value)){
			const frame = tf.absoluteTransforms[raw_target];

			if(!frame){
				status.setError("Required transform frame \""+raw_target+"\" not found.");
				return;
			}

			if(appendPose(frame)){
				drawHistory();
				status.setOK();
				tf_throttle_stamp = now;
			}
		}
	}
});

window.addEventListener("tf_fixed_frame_changed", ()=>{
	sample_array = []; //TODO keep a ref to the old frame and trasform the history
	drawHistory();
});

window.addEventListener("view_changed", drawHistory);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("Odom Pose Tracker Widget Loaded {uniqueID}")