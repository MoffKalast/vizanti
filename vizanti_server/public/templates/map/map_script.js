let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let paramsModule = await import(`${base_url}/ros_launch_params`);

let view = viewModule.view;
let tf = tfModule.tf;
let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;
let params = paramsModule.default;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let icons = {};
icons["map"] = await imageToDataURL("assets/map.svg");
icons["costmap"] = await imageToDataURL("assets/costmap.svg");
icons["raw"] = await imageToDataURL("assets/rawmap.svg");
icons["raw_transparent"] = await imageToDataURL("assets/rawmap_transparent_white.svg");
icons["raw_transparent_black"] = await imageToDataURL("assets/rawmap_transparent_black.svg");
icons["sonar"] = await imageToDataURL("assets/sonar.svg");

let listener = undefined;
let map_topic = undefined;
let retryTimer;
let loadingTopics = false;
let destroyed = false;
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

function setOpacityText(val){
	if(val == 0.0)
		opacityValue.textContent = "0.0 (Map rendering disabled)";
	else
		opacityValue.textContent = val;
}

opacitySlider.addEventListener('input', () =>  {
	setOpacityText(opacitySlider.value)
	saveSettings();
	drawMap();
});

const loadPathBox = document.getElementById("{uniqueID}_loadpath");
const loadTopicBox = document.getElementById("{uniqueID}_loadtopic");
const savePathBox = document.getElementById("{uniqueID}_savepath");

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

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic ?? topic;

	opacitySlider.value = loaded_data.opacity;
	setOpacityText(loaded_data.opacity);

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

	ctx.setTransform(1,0,0,1,0,0);
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	ctx.imageSmoothingEnabled = false;

	if(opacitySlider.value == 0.0)
		return;

	const map_width = view.getMapUnitsInPixels(
		temp_canvas.width * map_data.info.resolution
	);

	const map_height = view.getMapUnitsInPixels(
		temp_canvas.height * map_data.info.resolution
	);

	let tf_pose = map_data.pose;

	if(!timestampCheckbox.checked){
		tf_pose = tf.transformPoseStamped(
			map_data.header,
			map_data.info.origin.position,
			map_data.info.origin.orientation
		);
	}

	const pos = view.fixedToScreen({
		x: tf_pose.translation.x,
		y: tf_pose.translation.y,
	});

	const matrix = view.quaterionToProjectionMatrix(tf_pose.rotation);

	ctx.globalAlpha = opacitySlider.value;
	ctx.setTransform(matrix[0], matrix[1], matrix[2], matrix[3], pos.x, pos.y); //sx,0,0,sy,px,py
	ctx.scale(1.0, -1.0);
	ctx.drawImage(temp_canvas, 0, 0, map_width, map_height);
}

//Topic

// Retry while waiting for the first map. Rosbridge may subscribe before
// DDS discovers the publisher and miss its retained map.
function scheduleRetry(){
	clearTimeout(retryTimer);
	retryTimer = setTimeout(() => {
		if(!rosbridge.connected){
			scheduleRetry();
		}else if(topic){
			connect();
		}else{
			loadTopics();
		}
	}, 3000);
}

function connect(){
	clearTimeout(retryTimer);
	if(map_topic !== undefined){
		map_topic.unsubscribe(listener);
		map_topic = undefined;
	}

	if(!topic){
		status.setWarn("Waiting for an occupancy grid topic.");
		scheduleRetry();
		return;
	}

	map_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'nav_msgs/msg/OccupancyGrid',
		throttle_rate: parseInt(throttle.value), // throttle to once every two seconds max
		compression: rosbridge.compression,
		queue_length: 1
	});

	status.setWarn("No data received.");

	worker_thread.onmessage = (e) => {
		setTimeout(()=>{
			const img = e.data.image
			temp_canvas.width = img.width
			temp_canvas.height = img.height
			temp_canvas.getContext('2d', { colorSpace: 'srgb' }).putImageData(img, 0, 0);
			map_data = new_map_data;
			drawMap();
			status.setOK();
		},1);
	};
	
	listener = (msg) => {

		if(msg.info.width == 0 || msg.info.height == 0){
			status.setWarn("Received empty map.");
			return;
		}

		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
			msg.header.frame_id = tf.fixed_frame;
		}

		if(!tf.absoluteTransforms[msg.header.frame_id]){
			if(msg.header.frame_id == "map"){
				status.setWarn("Map transform not available yet, using identity transform.");

				//add a temporary transform so one can send an initialpose relative to it
				tf.absoluteTransforms[msg.header.frame_id] = {
					translation: {x: 0, y:0, z:0},
					rotation: new Quaternion()
				}
				tf.frame_list.add("map");

			}else{
				status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
				return;
			}
		}

		queueWorkerMsg(msg);
		received_msg = msg;
		clearTimeout(retryTimer);
	};
	map_topic.subscribe(listener);
	scheduleRetry();

	saveSettings();
}

function queueWorkerMsg(msg){
	msg.pose = tf.transformPoseStamped(
		msg.header,
		msg.info.origin.position, 
		msg.info.origin.orientation
	);

	new_map_data = msg;

	worker_thread.postMessage({
		map_msg: msg,
		colour_scheme: colourSchemeBox.value,
	});
}

function updateTopicOptions(topics){
	const options = [...topics];
	if(topic && !options.includes(topic))
		options.push(topic);

	selectionbox.replaceChildren(...options.map(name => new Option(name, name)));
	selectionbox.value = topic;
}

async function loadTopics(){
	if(loadingTopics)
		return;

	loadingTopics = true;
	try{
		const topics = await rosbridge.get_topics("nav_msgs/msg/OccupancyGrid");
		if(destroyed)
			return;

		if(!topic && topics.length){
			topic = topics[0];
			connect();
		}
		updateTopicOptions(topics);
	}finally{
		loadingTopics = false;
		if(!destroyed && !topic)
			scheduleRetry();
	}
}

colourSchemeBox.addEventListener("change", (event) => {
	icon.src = icons[colourSchemeBox.value];
	if(received_msg)
		queueWorkerMsg(received_msg);
});

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;

	map_data = undefined;
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	ctx.imageSmoothingEnabled = false;

	connect();
});

icon.addEventListener("click", (event) => {
	loadTopics();
});

updateTopicOptions([]);
connect();
if(rosbridge.connected)
	loadTopics();

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawMap();
}

window.addEventListener("tf_fixed_frame_changed", drawMap);
function onTFChanged(){
	if(received_msg && received_msg.header.frame_id != tf.fixed_frame){
		drawMap();
	}
}

window.addEventListener("tf_changed", onTFChanged);

window.addEventListener("view_changed", drawMap);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

window.addEventListener("{uniqueID}_destroy", () => {
	destroyed = true;
	clearTimeout(retryTimer);
	if(map_topic !== undefined)
		map_topic.unsubscribe(listener);

	worker_thread.terminate();

	window.removeEventListener("tf_fixed_frame_changed", drawMap);
	window.removeEventListener("tf_changed", onTFChanged);
	window.removeEventListener("view_changed", drawMap);
	window.removeEventListener("resize", resizeScreen);
	window.removeEventListener("orientationchange", resizeScreen);
}, {once: true});

console.log("Map Widget Loaded {uniqueID}")