let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);

let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;
let tf = tfModule.tf;

let offset_x = "-999";
let offset_y = "-999";

const clamp = (num, min, max) => Math.min(Math.max(num, min), max);
const vwToVh = vw => (vw * window.innerWidth) / window.innerHeight;

let topic = getTopic("{uniqueID}");

if(topic != "")
	topic += " (Imu)";

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

//persistent loading, so we don't re-fetch on every update
let stock_images = {};
stock_images["loading"] = await imageToDataURL("assets/tile_loading.png");
stock_images["error"] = await imageToDataURL("assets/tile_error.png");

let imu_topic = undefined;
let listener = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const canvas = document.getElementById('{uniqueID}_canvas');
const imgpreview = document.getElementById('{uniqueID}_imgpreview');

const text_pitch = document.getElementById("{uniqueID}_pitch");
const text_roll = document.getElementById("{uniqueID}_roll");
const text_yaw = document.getElementById("{uniqueID}_yaw");
const text_heading = document.getElementById("{uniqueID}_heading");
const text_quaternion = document.getElementById("{uniqueID}_quaternion");

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');
opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

const widthSlider = document.getElementById('{uniqueID}_width');
const widthValue = document.getElementById('{uniqueID}_width_value');
widthSlider.addEventListener('input', () =>  {
	widthValue.textContent = widthSlider.value;
	saveSettings();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

let mode = "" //see setMode()
let raw_target = "";

//Canvas setup
const ctx = canvas.getContext('2d', { willReadFrequently: true });

let textureLoaded = false;
let overlayLoaded = false;
let canvasSizeChanged = false;
let renderOnce = true;

//all angles summed need to move at least this much to trigger an update
const angleDelta = 0.001;

let prev_pitch = 0;
let prev_yaw = 0;
let prev_roll = 0;

let pitch = 0;
let yaw = 0; 
let roll = 0;

let quat = new Quaternion();
let quat_smooth = new Quaternion();

//precomputed pixels
let lut_px;
let lut_py;
let lut_pz;
let lut_index;
let lut_size = 0;
let imageData
let data;
let textureData;

function setupCanvas() {

	const width = canvas.width;
	const height = canvas.height;

	const radius = width / 2;
	const radiusSq = radius * radius;
	const invRadius = 1 / radius;
	
	imageData = ctx.createImageData(width, height);
	data = imageData.data;
	
	let pixelCount = 0;
	for (let y = 0; y < height; y++) {
		const dy = y - radius;
		for (let x = 0; x < width; x++) {
			const dx = x - radius;
			const distSq = dx * dx + dy * dy;
			if (distSq <= radiusSq * 0.82) {
				pixelCount++;
			}
		}
	}

	lut_size = pixelCount;
	lut_px = new Float32Array(lut_size);
	lut_py = new Float32Array(lut_size);
	lut_pz = new Float32Array(lut_size);
	lut_index = new Uint32Array(lut_size);
	
	let i = 0;
	for (let y = 0; y < height; y++) {
		const dy = y - radius;
		const dySq = dy * dy;
		for (let x = 0; x < width; x++) {
			const dx = x - radius;
			const distSq = dx * dx + dySq;
			
			if (distSq <= radiusSq * 0.82) {
				const dz = Math.sqrt(radiusSq - distSq);
				
				// Store in our new SoA structure
				lut_px[i] = dx * invRadius;
				lut_py[i] = dy * invRadius;
				lut_pz[i] = dz * invRadius;
				
				const pixelIndex = (y * width + x) * 4;
				lut_index[i] = pixelIndex;
				
				data[pixelIndex + 3] = 255; // Set alpha once
				i++;
			}
		}
	}

	canvasSizeChanged = false;
	renderOnce = true;
}

const texture = new Image();
texture.src = 'assets/navball_texture.jpg';
texture.onload = () => {
	const textureCanvas = document.createElement('canvas');
	textureCanvas.width = texture.width;
	textureCanvas.height = texture.height;

	const textureCtx = textureCanvas.getContext('2d');
	textureCtx.drawImage(texture, 0, 0);
	textureData = textureCtx.getImageData(0, 0, texture.width, texture.height);

	textureLoaded = true;
};

const overlay = new Image();
overlay.src = 'assets/navball_overlay.png';
overlay.onload = () => {
	overlayLoaded = true;
};

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	offset_x = loaded_data.offset_x;
	offset_y = loaded_data.offset_y;

	throttle.value = loaded_data.throttle;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
	canvas.style.opacity = loaded_data.opacity;

	widthSlider.value = loaded_data.width;
	widthValue.innerText = loaded_data.width;

	displayImageOffset(offset_x, offset_y);
	setMode();
}else{
	displayImageOffset(50, 95);
	saveSettings();
}

function saveSettings(){
	setMode();
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		throttle: throttle.value,
		width: widthSlider.value,
		offset_x: offset_x,
		offset_y: offset_y
	}
	settings.save();

	canvas.style.opacity = opacitySlider.value;
	displayImageOffset(offset_x, offset_y);
}

//Topic

function renderNavball() {

	if(!textureLoaded || !overlayLoaded)
		return;

	if(canvasSizeChanged){
		setupCanvas();
	}

	const interpolator = quat_smooth.slerp(quat);
	quat_smooth = interpolator(0.1);

	const euler = quat_smooth.toEuler();
	const pitch = euler.pitch;
	const yaw = euler.h; 
	const roll = euler.g;

 	const dp = Math.abs(prev_pitch - pitch);
	const dy = Math.abs(prev_yaw - yaw);
	const dr = Math.abs(prev_roll - roll);

	if (dp + dy + dr < angleDelta && !renderOnce)
		return;

	const width = canvas.width;
	const height = canvas.height;

	const texWidth = texture.width;
	const texHeight = texture.height;
	const texturePixels = textureData.data;
	const invPI = 1 / Math.PI;
	const texWidthMinus1 = texWidth - 1;
	const texHeightMinus1 = texHeight - 1;

	prev_pitch = pitch;
	prev_yaw = yaw;
	prev_roll = roll;

	const cosP = Math.cos(pitch);
	const sinP = Math.sin(pitch);
	const cosY = Math.cos(yaw - Math.PI/2);
	const sinY = Math.sin(yaw - Math.PI/2);
	const cosR = Math.cos(roll + Math.PI);
	const sinR = Math.sin(roll + Math.PI);
	
	// Pre-calculate the combined rotation matrix
	const m11 = cosY * cosR + sinY * sinP * sinR;
	const m12 = cosP * sinR;
	const m13 = -sinY * cosR + cosY * sinP * sinR;
	
	const m21 = -cosY * sinR + sinY * sinP * cosR;
	const m22 = cosP * cosR;
	const m23 = sinY * sinR + cosY * sinP * cosR;
	
	const m31 = sinY * cosP;
	const m32 = -sinP;
	const m33 = cosY * cosP;
	
	for (let i = 0; i < lut_size; i++) {
		const px = lut_px[i];
		const py = lut_py[i];
		const pz = lut_pz[i];
		
		// Apply the single combined rotation
		const rot_px = px * m11 + py * m21 + pz * m31;
		const rot_py = px * m12 + py * m22 + pz * m32;
		const rot_pz = px * m13 + py * m23 + pz * m33;
		
		// Convert to spherical coords
		const theta = Math.atan2(rot_px, rot_pz);
		const phi = Math.asin(rot_py);
		
		const u = (-theta * invPI + 1) * 0.5;
		const v = 0.5 - phi * invPI;
		
		const texX = (u * texWidthMinus1) | 0;
		const texY = (v * texHeightMinus1) | 0;
		const texIndex = (texY * texWidth + texX) * 4;
		
		const pixelIndex = lut_index[i];
		data[pixelIndex]     = texturePixels[texIndex];
		data[pixelIndex + 1] = texturePixels[texIndex + 1];
		data[pixelIndex + 2] = texturePixels[texIndex + 2];
	}
	
	ctx.putImageData(imageData, 0, 0);
	if (overlayLoaded) {
		ctx.drawImage(overlay, 0, 0, width, height);
	}

	renderOnce = false;
}

function updateData(){
	function radToDeg(radians){
		return radians * 180/Math.PI;
	}

	function enuYawToHeading(yawRad) {
		const heading = (90 - radToDeg(yawRad)) % 360;
		return (heading + 360) % 360;
	}

	text_quaternion.innerText = "Quaternion XYZW: "+quat.x.toFixed(3)+","+quat.y.toFixed(3)+","+quat.z.toFixed(3)+","+quat.w.toFixed(3);

	let data = quat.toEuler();
	const pitch = data.pitch;
	const yaw = data.h; 
	const roll = data.g;

	text_pitch.innerText = "Pitch: "+radToDeg(pitch).toFixed(0)+"°";
	text_roll.innerText = "Roll: "+radToDeg(roll).toFixed(0)+"°";
	text_yaw.innerText = "Yaw: "+radToDeg(yaw).toFixed(0)+"°";

	text_heading.innerText = "Heading: "+enuYawToHeading(yaw).toFixed(0)+"°";

	renderNavball();
}

let updateInteral = setInterval(renderNavball, 33);

function connect(){
	if(imu_topic !== undefined){
		imu_topic.unsubscribe(listener);
	}

	if(mode != "topic")
		return;

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	status.setWarn("No data received.");

	imu_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : raw_target,
		messageType : 'sensor_msgs/msg/Imu',
		throttle_rate: parseInt(throttle.value)
	});
	
	listener = imu_topic.subscribe(async (msg) => {  

		quat = new Quaternion([
			msg.orientation.w, 
			msg.orientation.x, 
			msg.orientation.y, 
			msg.orientation.z
		]);

		updateData();
		status.setOK();
	});

	saveSettings();
}

async function loadTopics(){
	let imu_array = await rosbridge.get_topics("sensor_msgs/msg/Imu");
	let tf_array = Array.from(tf.frame_list);

	let topiclist = "";
	imu_array.forEach(element => {
		topiclist += "<option value='"+element+" (Imu)'>"+element+" (Imu)</option>"
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
		if(imu_array.includes(raw_target) || tf_array.includes(raw_target)){
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
	saveSettings();
	connect();
});

selectionbox.addEventListener("click", connect);

icon.addEventListener("click", ()=> {
	loadTopics();
});

loadTopics();

function setMode(){
	if(topic.endsWith("(Frame)")){
		mode = "tf";
		raw_target = topic.replace(" (Frame)", "");
	}else if(topic.endsWith("(Imu)")){
		mode = "topic";
		raw_target = topic.replace(" (Imu)", "");
	}else{
		mode = "";
		raw_target = "";
	}
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

				
			quat = frame.rotation;
			updateData();

			status.setOK();
			tf_throttle_stamp = now;
		}
	}
});

//preview for definining position
let preview_active = false;

function onStart(event) {
	preview_active = true;
	document.addEventListener('mousemove', onMove);
	document.addEventListener('touchmove', onMove);
	document.addEventListener('mouseup', onEnd);
	document.addEventListener('touchend', onEnd);
}

function displayImageOffset(x, y){

	let canvas_width = widthSlider.value;
	let canvas_height = vwToVh(canvas_width);

	offset_x = clamp(x, canvas_width*0.3, 100 - canvas_width*0.3) ;
	offset_y = clamp(y, canvas_height*0.3, 100 - canvas_height*0.3) ;

	imgpreview.style.left = offset_x+"vw";
	imgpreview.style.top = offset_y+"vh";

	canvas.style.left = offset_x+"vw";
	canvas.style.top = offset_y+"vh";

	const new_size = Math.round((canvas_width/100.0) * window.innerWidth);

	if(canvas.width != new_size){
		canvas.style.width = canvas_width+"vw";
		canvas.style.height = canvas_height+"vh";
		canvas.width = new_size;
		canvas.height = new_size;
		canvasSizeChanged = true;
	}
}

window.addEventListener('resize', ()=>{
	displayImageOffset(offset_x, offset_y);
});

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
	
		let canvas_width = widthSlider.value/2;
		let canvas_height = vwToVh(canvas_width);
	
		offset_x = clamp(currentX/window.innerWidth * 100, canvas_width*0.3, 100 - canvas_width*0.3);
		offset_y = clamp(currentY/window.innerHeight * 100, canvas_height*0.3, 100 - canvas_height*0.3);

		saveSettings();
	}
}

function onEnd() {
	preview_active = false;
	document.removeEventListener('mousemove', onMove);
	document.removeEventListener('touchmove', onMove);
	document.removeEventListener('mouseup', onEnd);
	document.removeEventListener('touchend', onEnd);
}
  
imgpreview.addEventListener('mousedown', onStart);
imgpreview.addEventListener('touchstart', onStart);

displayImageOffset(offset_x, offset_y);

console.log("Navball Widget Loaded {uniqueID}")

