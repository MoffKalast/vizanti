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

let range_topic = undefined;
let listener = undefined;

let data = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const timestampCheckbox = document.getElementById('{uniqueID}_use_timestamp');
timestampCheckbox.addEventListener('change', saveSettings);

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');
opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	saveSettings();
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
	opacityValue.innerText = loaded_data.opacity;

	timestampCheckbox.checked = loaded_data.use_timestamp ?? false;

	colourpicker.value = loaded_data.color;
	throttle.value = loaded_data.throttle;
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		color: colourpicker.value,
		throttle: throttle.value,
		use_timestamp: timestampCheckbox.checked
	}
	settings.save();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

async function drawCells() {

	ctx.clearRect(0, 0, canvas.width, canvas.height);

	if(!data){
		return;
	}

	const unit = view.getMapUnitsInPixels(1.0);

	ctx.clearRect(0, 0, canvas.width, canvas.height);
	ctx.globalAlpha = opacitySlider.value;
	ctx.fillStyle = colourpicker.value;

	let tf_pose = timestampCheckbox.checked ? data.pose : tf.absoluteTransforms[data.msg.header.frame_id];

	if(tf_pose == undefined)
		return;

	const pos = view.fixedToScreen({
		x: tf_pose.translation.x,
		y: tf_pose.translation.y,
	});

	const yaw = tf_pose.rotation.toEuler().h;

	ctx.save();
	ctx.translate(pos.x, pos.y);
	ctx.scale(1.0, -1.0);
	ctx.rotate(yaw);

	const wid = Math.abs(data.msg.cell_width) * unit;
	const hei = Math.abs(data.msg.cell_height) * unit;

	for(let i = 0; i < data.msg.cells.length; i++){
		ctx.fillRect(data.msg.cells[i].x * unit - wid/2 + 1, data.msg.cells[i].y * unit - hei/2 + 1, wid-1, hei-1);
	}
	
	ctx.restore();
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawCells();
}

window.addEventListener("tf_changed", drawCells);
window.addEventListener("view_changed", drawCells);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

//Topic

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
		messageType : 'nav_msgs/GridCells',
		throttle_rate: parseInt(throttle.value),
		compression: "cbor"
	});

	status.setWarn("No data received.");
	
	listener = range_topic.subscribe((msg) => {	

		if(msg.cells.length == 0){
			status.setWarn("Received empty grid.");
		}

		if(msg.cell_width == 0 | msg.cell_height == 0){
			status.setError("Grid cell size must be nonzero, received width="+msg.cell_width+" height="+msg.cell_height+".");
			return;
		}

		const pose = tf.absoluteTransforms[msg.header.frame_id];

		if(!pose){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

		data = {};
		data.pose = pose;
		data.msg = msg
		status.setOK();
		drawCells();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("nav_msgs/GridCells");
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
icon.addEventListener("click", loadTopics);

loadTopics();
resizeScreen();

console.log("Laserscan Widget Loaded {uniqueID}")