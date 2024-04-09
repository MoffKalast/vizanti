let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let view = viewModule.view;
let tf = tfModule.tf;
let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;

async function saveMap(save_path, topic) {
	const saveMapService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/save_map",
		serviceType: "vizanti/SaveMap",
	});

	const request = new ROSLIB.ServiceRequest({
		file_path: save_path,
		topic: topic
	});

	return new Promise((resolve, reject) => {
		saveMapService.callService(request, (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
		});
	});
}

async function loadMap(load_path, topic) {
	const loadMapService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/load_map",
		serviceType: "vizanti/LoadMap",
	});

	const request = new ROSLIB.ServiceRequest({
		file_path: load_path,
		topic: topic,
	});

	return new Promise((resolve, reject) => {
		loadMapService.callService(request, (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
		});
	});
}

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let icons = {};
icons["map"] = await imageToDataURL("assets/map.svg");
icons["costmap"] = await imageToDataURL("assets/costmap.svg");
icons["raw"] = await imageToDataURL("assets/rawmap.svg");

let listener = undefined;
let map_topic = undefined;
let map_data = undefined;
let new_map_data = undefined;

let received_msg = undefined;

//firefox bug workaround
const temp_canvas = document.createElement('canvas');

const worker_thread = new Worker(`${base_url}/templates/map/map_worker.js`);
const map_canvas = document.createElement('canvas');

//offscreen rendering is currently half broken in firefox
//https://bugzilla.mozilla.org/show_bug.cgi?id=1833496
const offscreen_canvas = map_canvas.transferControlToOffscreen();
worker_thread.postMessage({	canvas: offscreen_canvas}, [offscreen_canvas]);

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');

const loadPathBox = document.getElementById("{uniqueID}_loadpath");
const loadTopicBox = document.getElementById("{uniqueID}_loadtopic");
const savePathBox = document.getElementById("{uniqueID}_savepath");
const loadButton = document.getElementById('{uniqueID}_load');
const saveButton = document.getElementById('{uniqueID}_save');

//rendring colour modes: 0 = map, 1 = costmap, 2 = raw
const colourSchemeBox = document.getElementById('{uniqueID}_colour_scheme');
colourSchemeBox.selectedIndex = topic.includes("cost") ? 1 : 0;
colourSchemeBox.addEventListener('change', saveSettings);

const timestampCheckbox = document.getElementById('{uniqueID}_use_timestamp');
timestampCheckbox.addEventListener('change', saveSettings);

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

loadButton.addEventListener('click',  async () => {
	let path = loadPathBox.value;

	if (path.endsWith(".pgm")) {
		path = path.slice(0, -4) + ".yaml";
	} else if (!path.endsWith(".yaml")) {
		path += ".yaml";
	}

	loadPathBox.value = path;

	try {
		const result = await loadMap(path, loadTopicBox.value);
		alert(result.message);
	} catch (error) {
		alert(error);
	}
});

saveButton.addEventListener('click', async () => {
	let path = savePathBox.value;

	if (path.endsWith(".pgm")) {
		path = path.slice(0, -4);
	} else if (path.endsWith(".yaml")) {
		path = path.slice(0, -5);
	}

	savePathBox.value = path;

	try {
		const result = await saveMap(path, topic);
		alert(result.message);
	} catch (error) {
		alert(error);
	}
});

opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;

	if(loaded_data.costmap_mode !== undefined){
		colourSchemeBox.selectedIndex = loaded_data.costmap_mode ? 1 : 0;
	}else{
		colourSchemeBox.selectedIndex = loaded_data.colour_scheme > 0 ? loaded_data.colour_scheme: 0;
	}

	timestampCheckbox.checked = loaded_data.use_timestamp ?? false;
	throttle.value = loaded_data.throttle ?? 1000;

}else{
	saveSettings();
}

icon.src = icons[colourSchemeBox.value];

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		colour_scheme: colourSchemeBox.selectedIndex,
		throttle: throttle.value,
		use_timestamp: timestampCheckbox.checked
	}
	settings.save();
}

//Rendering

async function drawMap(){

	if(!map_data)
		return;

	const map_width = view.getMapUnitsInPixels(
		temp_canvas.width * map_data.info.resolution
	);

	const map_height = view.getMapUnitsInPixels(
		temp_canvas.height * map_data.info.resolution
	);

	let tf_pose = map_data.pose;

	if(!timestampCheckbox.checked){
		tf_pose = tf.transformPose(
			map_data.header.frame_id,
			tf.fixed_frame,
			map_data.info.origin.position,
			map_data.info.origin.orientation
		);
	}

	const pos = view.fixedToScreen({
		x: tf_pose.translation.x,
		y: tf_pose.translation.y,
	});

	const yaw = tf_pose.rotation.toEuler().h;

	ctx.clearRect(0, 0, canvas.width, canvas.height);
	ctx.imageSmoothingEnabled = false;

	ctx.save();
	ctx.globalAlpha = opacitySlider.value;
	ctx.translate(pos.x, pos.y);
	ctx.scale(1.0, -1.0);
	ctx.rotate(yaw);
	ctx.drawImage(temp_canvas, 0, 0, map_width, map_height);
	ctx.restore();
}

//Topic

function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(map_topic !== undefined){
		map_topic.unsubscribe(listener);
	}

	map_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'nav_msgs/OccupancyGrid',
		throttle_rate: parseInt(throttle.value), // throttle to once every second max
		compression: "cbor"
	});

	status.setWarn("No data received.");

	worker_thread.onmessage = (e) => {
		setTimeout(()=>{

			const img = e.data.image
			temp_canvas.width = img.width
			temp_canvas.height = img.height
			temp_canvas.getContext('2d').putImageData(img, 0, 0);
			map_data = new_map_data;
			drawMap();
			status.setOK();
		},12);
	};
	
	listener = map_topic.subscribe((msg) => {

		if(msg.info.width == 0 || msg.info.height == 0){
			status.setWarn("Received empty map.");
			return;
		}

		if(!tf.absoluteTransforms[msg.header.frame_id]){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

		queueWorkerMsg(msg);
		received_msg = msg;
	});

	saveSettings();
}

function queueWorkerMsg(msg){
	msg.pose = tf.transformPose(
		msg.header.frame_id,
		tf.fixed_frame,
		msg.info.origin.position,
		msg.info.origin.orientation
	);

	new_map_data = msg;
	map_data = undefined;

	worker_thread.postMessage({
		map_msg: msg,
		colour_scheme: colourSchemeBox.value,
	});
}

async function loadTopics(){
	let result = await rosbridge.get_topics("nav_msgs/OccupancyGrid");

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

colourSchemeBox.addEventListener("change", (event) => {
	icon.src = icons[colourSchemeBox.value];
	queueWorkerMsg(received_msg);
});

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;

	map_data = undefined;
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	ctx.imageSmoothingEnabled = false;

	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawMap();
}

window.addEventListener("tf_changed", drawMap);
window.addEventListener("view_changed", drawMap);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("Map Widget Loaded {uniqueID}")