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

const timestampCheckbox = document.getElementById('{uniqueID}_use_timestamp');
timestampCheckbox.addEventListener('change', saveSettings);

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');
opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
	drawCells();
});

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
	drawCells();
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
		opacity: opacitySlider.value,
		color: colourpicker.value,
		throttle: throttle.value,
		use_timestamp: timestampCheckbox.checked
	}
	settings.save();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

async function drawCells() {

	ctx.setTransform(1,0,0,1,0,0);
	ctx.clearRect(0, 0, canvas.width, canvas.height);

	if(!data){
		return;
	}

	ctx.globalAlpha = opacitySlider.value;
	ctx.fillStyle = colourpicker.value;

	let tf_pose = timestampCheckbox.checked ? data.pose : tf.absoluteTransforms[data.msg.header.frame_id];

	if(!tf_pose){
		return;
	}

	const pos = view.fixedToScreen({
		x: tf_pose.translation.x,
		y: tf_pose.translation.y,
	});

	const matrix = view.quaterionToProjectionMatrix(tf_pose.rotation);
	ctx.setTransform(matrix[0], matrix[1], matrix[2], matrix[3], pos.x, pos.y); //sx,0,0,sy,px,py
	ctx.scale(1.0, -1.0);

	const unit = view.getMapUnitsInPixels(1.0);
	const wid = Math.abs(data.msg.cell_width) * unit;
	const hei = Math.abs(data.msg.cell_height) * unit;

	ctx.beginPath();
	for(let i = 0; i < data.msg.cells.length; i++){
		const x = data.msg.cells[i].x * unit - wid / 2 + 1;
		const y = data.msg.cells[i].y * unit - hei / 2 + 1;
		ctx.moveTo(x, y);
		ctx.lineTo(x + wid - 1, y);
		ctx.lineTo(x + wid - 1, y + hei - 1);
		ctx.lineTo(x, y + hei - 1);
		ctx.lineTo(x, y);
	}
	ctx.fill();
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawCells();
}

window.addEventListener("tf_fixed_frame_changed", drawCells);
window.addEventListener("tf_changed", ()=>{
	if (data && data.msg.frame_id != tf.fixed_frame){
		drawCells();
	}
});

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

		if(msg.cells === undefined || msg.cells.length == 0){
			status.setWarn("Received empty grid. Oh no! Anyway...");
			data = undefined;
			drawCells();
			return;
		}

		if(msg.cell_width == 0 | msg.cell_height == 0){
			status.setError("Grid cell size must be nonzero, received width="+msg.cell_width+" height="+msg.cell_height+".");
			data = undefined;
			drawCells();
			return;
		}

		let error = false;
		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
			msg.header.frame_id = tf.fixed_frame;
			error = true;
		}

		const pose = tf.absoluteTransforms[msg.header.frame_id];

		if(!pose){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			data = undefined;
			drawCells();
			return;
		}

		data = {};
		data.pose = pose;
		data.msg = msg		
		drawCells();

		if(!error){
			status.setOK();
		}
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
click_icon.addEventListener("click", loadTopics);

loadTopics();
resizeScreen();

console.log("Gridcells Widget Loaded {uniqueID}")