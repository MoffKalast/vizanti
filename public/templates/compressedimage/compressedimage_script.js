let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;

let img_offset_x = "-999";
let img_offset_y = "-999";
let last_natural_width = 400;
let last_natural_height = 250;

const clamp = (num, min, max) => Math.min(Math.max(num, min), max);
const vwToVh = vw => (vw * window.innerWidth) / window.innerHeight;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

//persistent loading, so we don't re-fetch on every update
let stock_images = {};
stock_images["loading"] = await imageToDataURL("assets/img_loading.png");
stock_images["error"] = await imageToDataURL("assets/img_error.png");

let image_topic = undefined;
let listener = undefined;
let peer = undefined;
let stats_timer = undefined;
let last_stats = undefined;
let connection_id = 0;
let active_stream = undefined;
let stream_failed = false;
let stream_received = false;
let active_throttle = undefined;
let throttle_timer = undefined;

let mode = "mjpeg";
let streams = [];

const PIPELINE_DESCRIPTIONS = {
	passthrough: "passthrough, images are sent exactly as published",
	repackage: "repackage, the video is rewrapped on the robot without re-encoding",
	transcode: "transcode, images are re-encoded on the robot, which uses CPU"
};

const list_service = new ROSLIB.Service({
	ros : rosbridge.ros,
	name : "/vizanti/image_streams/list",
	serviceType : "vizanti/ListImageStreams"
});

const start_service = new ROSLIB.Service({
	ros : rosbridge.ros,
	name : "/vizanti/image_streams/start",
	serviceType : "vizanti/StartImageStream"
});

const selectionbox = document.getElementById("{uniqueID}_topic");
const rotationbox = document.getElementById("{uniqueID}_rotation");

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const canvas = document.getElementById('{uniqueID}_image');
const video = document.getElementById('{uniqueID}_video');
const imgpreview = document.getElementById('{uniqueID}_imgpreview');

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

const text_resolution = document.getElementById("{uniqueID}_resolution");
const text_datasize = document.getElementById("{uniqueID}_datasize");
const text_rate = document.getElementById("{uniqueID}_rate");
const text_bandwidth = document.getElementById("{uniqueID}_bandwidth");
const text_compression = document.getElementById("{uniqueID}_compression");
const text_type = document.getElementById("{uniqueID}_type");
const text_frame = document.getElementById("{uniqueID}_frame");
const text_pipeline = document.getElementById("{uniqueID}_pipeline");

const COMPRESSION_TYPES = ["jpeg", "jpg", "png", "tiff", "webp", "rvl"];

let arrival_times = [];
let arrival_bytes = [];

function parseFormat(format){
	const lower = (format ?? "").toLowerCase();
	const encoding = lower.split(";")[0].trim();
	const compression = COMPRESSION_TYPES.find(type => lower.includes(type)) ?? "unknown";
	const is_depth = lower.includes("compresseddepth") || encoding.startsWith("16uc1") || encoding.startsWith("32fc1") || encoding.startsWith("16uc") && lower.includes("depth");
	return {encoding, compression, is_depth};
}

function formatBytes(bytes){
	if(bytes >= 1024 * 1024)
		return (bytes / (1024 * 1024)).toFixed(2)+" MB";

	if(bytes >= 1024)
		return (bytes / 1024).toFixed(1)+" kB";

	return bytes+" B";
}

function resetLiveData(){
	arrival_times = [];
	arrival_bytes = [];

	text_resolution.innerText = "Resolution: ?";
	text_datasize.innerText = "Data size: ?";
	text_rate.innerText = "Rate: ?";
	text_bandwidth.innerText = "Bandwidth: ?";
	text_compression.innerText = "Compression: ?";
	text_type.innerText = "Type: ?";
	text_frame.innerText = "Frame: ?";
}

function updateLiveData(msg){
	const info = parseFormat(msg.format);
	const bytes = msg.data?.byteLength ?? 0;

	arrival_times.push(performance.now());
	arrival_bytes.push(bytes);
	if(arrival_times.length > 20){
		arrival_times.shift();
		arrival_bytes.shift();
	}

	text_datasize.innerText = "Data size: "+formatBytes(bytes);
	text_compression.innerText = "Compression: "+info.compression.toUpperCase();
	text_type.innerText = "Type: "+(info.is_depth ? "Depth" : (info.encoding.startsWith("mono") || info.encoding == "8uc1" ? "Grayscale" : "Color"))+(info.encoding == "" ? "" : " ("+info.encoding+")");
	text_frame.innerText = "Frame: "+(msg.header?.frame_id == "" ? "(empty)" : msg.header?.frame_id ?? "?");

	if(arrival_times.length > 1){
		const seconds = (arrival_times[arrival_times.length - 1] - arrival_times[0]) / 1000;
		const rate = (arrival_times.length - 1) / seconds;
		const mean_bytes = arrival_bytes.reduce((a, b) => a + b, 0) / arrival_bytes.length;

		text_rate.innerText = "Rate: "+rate.toFixed(1)+" Hz";
		text_bandwidth.innerText = "Bandwidth: "+formatBytes(mean_bytes * rate)+"/s";
	}
}

const throttle = document.getElementById('{uniqueID}_throttle');

function throttleMs(){
	return Math.max(0, parseInt(throttle.value) || 0);
}

function applyThrottle(){
	clearTimeout(throttle_timer);

	if(throttleMs() == active_throttle)
		return;

	saveSettings();
	connect();
}

throttle.addEventListener("input", (event) =>{
	clearTimeout(throttle_timer);
	throttle_timer = setTimeout(applyThrottle, 1000);
});

throttle.addEventListener("change", (event) =>{
	throttle.value = throttleMs();
	applyThrottle();
});

rotationbox.addEventListener("change", (event) => {
	saveSettings();
});

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	mode = loaded_data.mode ?? "mjpeg";

	img_offset_x = loaded_data.img_offset_x;
	img_offset_y = loaded_data.img_offset_y;

	throttle.value = loaded_data.throttle;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
	canvas.style.opacity = loaded_data.opacity;

	widthSlider.value = loaded_data.width;
	widthValue.innerText = loaded_data.width;
	rotationbox.value = loaded_data.rotation;

	last_natural_width = loaded_data.last_natural_width ?? 400;
	last_natural_height = loaded_data.last_natural_height ?? 300;

	canvas.style.transform = `translate(-50%, -50%) rotate(${loaded_data.rotation}deg)`;
	displayImageOffset(img_offset_x, img_offset_y);
}else{
	displayImageOffset(0, 100);
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		mode: mode,
		opacity: opacitySlider.value,
		throttle: throttle.value,
		width: widthSlider.value,
		img_offset_x: img_offset_x,
		img_offset_y: img_offset_y,
		rotation: rotationbox.value,
		last_natural_width: last_natural_width,
		last_natural_height: last_natural_height
	}
	settings.save();

	[canvas, video].forEach(element => {
		element.style.opacity = opacitySlider.value;
		element.style.transform = `translate(-50%, -50%) rotate(${rotationbox.value}deg)`;
	});
	displayImageOffset(img_offset_x, img_offset_y);
}

//Topic

let blob_url = undefined;

function setCanvasSrc(src){
	if(blob_url !== undefined)
		URL.revokeObjectURL(blob_url);

	blob_url = src.startsWith("blob:") ? src : undefined;
	canvas.src = src;
}

async function getImage(src) {
    return new Promise((resolve, reject) => {
        let img = new Image();
        img.onload = () => resolve(src);
        img.onerror = () => reject(src);
        img.src = src;
    });
}

function callService(service, request){
	return new Promise((resolve, reject) => {
		service.callService(new ROSLIB.ServiceRequest(request), resolve, reject);
	});
}

function showVideo(visible){
	video.style.display = visible ? "" : "none";
	canvas.style.display = visible ? "none" : "";
}

function showStockImage(name){
	showVideo(false);
	canvas.onload = null;
	setCanvasSrc(stock_images[name]);
	displayImageOffset(img_offset_x, img_offset_y);
}

function updatePipeline(){
	const stream = streams.find(s => s.topic == topic && s.mode == mode);
	text_pipeline.innerText = "Pipeline: "+(stream === undefined ? "?" : PIPELINE_DESCRIPTIONS[stream.pipeline] ?? stream.pipeline);
}

function fail(message){
	stream_failed = true;
	status.setError(message);
}

function streamRunning(){
	if(stream_failed || active_stream != mode+":"+topic)
		return false;

	if(mode == "webrtc")
		return peer !== undefined && ["new", "connecting", "connected"].includes(peer.connectionState);

	return stream_received;
}

function disconnect(){
	connection_id++;
	active_stream = undefined;

	if(image_topic !== undefined){
		image_topic.unsubscribe(listener);
		image_topic = undefined;
	}

	if(peer !== undefined){
		peer.close();
		peer = undefined;
	}

	if(stats_timer !== undefined){
		clearInterval(stats_timer);
		stats_timer = undefined;
	}

	last_stats = undefined;
	video.srcObject = null;
}

function connect(){
	disconnect();
	active_stream = mode+":"+topic;
	active_throttle = throttleMs();
	stream_failed = false;
	stream_received = false;
	resetLiveData();
	updatePipeline();
	showStockImage("loading");
	saveSettings();

	if(topic == ""){
		fail("Empty topic.");
		return;
	}

	status.setWarn("Starting stream...");

	if(mode == "webrtc")
		connectWebrtc(connection_id);
	else
		connectMjpeg(connection_id);
}

async function connectMjpeg(id){
	let stream_topic = topic;

	try{
		const result = await callService(start_service, {topic: topic, mode: "mjpeg", throttle: throttleMs(), sdp: ""});

		if(!result.success){
			if(id == connection_id)
				fail(result.message);
			return;
		}

		stream_topic = result.stream_topic;
	}catch(error){
		console.warn("Image stream service unavailable, subscribing directly:", error);
	}

	if(id != connection_id)
		return;

	status.setWarn("No data received.");

	image_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : stream_topic,
		messageType : 'sensor_msgs/CompressedImage',
		throttle_rate: throttleMs(),
		queue_length: 1,
		compression: 'cbor'
	});
	
	listener = image_topic.subscribe(async (msg) => {  
		const mime = parseFormat(msg.format).compression == "png" ? "image/png" : "image/jpeg";
		const src = URL.createObjectURL(new Blob([msg.data], {type: mime}));

		updateLiveData(msg);

		getImage(src)
			.then((img) => {
			    canvas.onload = () => {
			        last_natural_width = canvas.naturalWidth;
			        last_natural_height = canvas.naturalHeight;
			        text_resolution.innerText = "Resolution: "+canvas.naturalWidth+" x "+canvas.naturalHeight;
			        if(!stream_received){
			            displayImageOffset(img_offset_x, img_offset_y);
			            status.setOK();
			            stream_received = true;
			        }
			    };
			    setCanvasSrc(src);
			})
			.catch((e) => {
				URL.revokeObjectURL(src);
				showStockImage("error");
				fail("Failed to decode image.");
			});
	});
}

function waitForIceGathering(pc){
	return new Promise((resolve) => {
		if(pc.iceGatheringState == "complete"){
			resolve();
			return;
		}

		const timeout = setTimeout(resolve, 2000);
		pc.addEventListener("icegatheringstatechange", () => {
			if(pc.iceGatheringState == "complete"){
				clearTimeout(timeout);
				resolve();
			}
		});
	});
}

async function connectWebrtc(id){
	const pc = new RTCPeerConnection();
	peer = pc;

	pc.addTransceiver("video", {direction: "recvonly"});

	pc.addEventListener("track", (event) => {
		video.srcObject = new MediaStream([event.track]);
	});

	pc.addEventListener("connectionstatechange", () => {
		if(id != connection_id)
			return;

		if(pc.connectionState == "failed"){
			showStockImage("error");
			fail("WebRTC connection failed, the robot may not be reachable over UDP.");
		}else if(pc.connectionState == "disconnected"){
			status.setWarn("WebRTC connection interrupted.");
		}
	});

	try{
		await pc.setLocalDescription(await pc.createOffer());
		await waitForIceGathering(pc);

		if(id != connection_id)
			return;

		const result = await callService(start_service, {topic: topic, mode: "webrtc", throttle: throttleMs(), sdp: pc.localDescription.sdp});

		if(id != connection_id)
			return;

		if(!result.success){
			showStockImage("error");
			fail(result.message);
			return;
		}

		await pc.setRemoteDescription({type: "answer", sdp: result.sdp});
	}catch(error){
		if(id != connection_id)
			return;

		showStockImage("error");
		fail("Failed to start WebRTC stream: "+error);
		return;
	}

	if(id != connection_id)
		return;

	text_compression.innerText = "Compression: H264";
	text_type.innerText = "Type: Video";
	status.setWarn("Waiting for video...");
	stats_timer = setInterval(() => updateWebrtcStats(pc).catch(() => {}), 1000);
}

async function updateWebrtcStats(pc){
	const stats = await pc.getStats();

	stats.forEach(report => {
		if(report.type != "inbound-rtp" || report.kind != "video")
			return;

		if(last_stats !== undefined){
			const seconds = (report.timestamp - last_stats.timestamp) / 1000;
			const frames = report.framesDecoded - last_stats.framesDecoded;
			const bytes = report.bytesReceived - last_stats.bytesReceived;

			text_rate.innerText = "Rate: "+(frames / seconds).toFixed(1)+" Hz";
			text_bandwidth.innerText = "Bandwidth: "+formatBytes(bytes / seconds)+"/s";
			text_datasize.innerText = "Data size: "+(frames > 0 ? formatBytes(bytes / frames)+" per frame" : "?");
		}

		last_stats = report;
	});
}

video.addEventListener("resize", () => {
	if(video.videoWidth == 0)
		return;

	last_natural_width = video.videoWidth;
	last_natural_height = video.videoHeight;
	text_resolution.innerText = "Resolution: "+video.videoWidth+" x "+video.videoHeight;

	showVideo(true);
	displayImageOffset(img_offset_x, img_offset_y);
	status.setOK();
	stream_received = true;
	saveSettings();
});

async function getStreams(){
	try{
		const result = await callService(list_service, {});
		return result.topics.map((stream_topic, i) => ({topic: stream_topic, type: result.types[i], mode: result.modes[i], pipeline: result.pipelines[i]}));
	}catch(error){
		const topics = await rosbridge.get_topics("sensor_msgs/CompressedImage");
		return topics.filter(t => !t.startsWith("/vizanti/")).map(t => ({topic: t, type: "sensor_msgs/CompressedImage", mode: "mjpeg", pipeline: "passthrough"}));
	}
}

function optionHTML(stream_topic, stream_mode){
	return "<option value='"+stream_mode+":"+stream_topic+"'>"+stream_topic+" ("+stream_mode+")</option>";
}

async function loadTopics(){
	streams = await getStreams();

	let topiclist = "";
	streams.forEach(stream => {
		topiclist += optionHTML(stream.topic, stream.mode);
	});

	if(topic == "" && streams.length > 0){
		topic = streams[0].topic;
		mode = streams[0].mode;
	}else if(topic != "" && !streams.some(s => s.topic == topic && s.mode == mode)){
		topiclist += optionHTML(topic, mode);
	}

	selectionbox.innerHTML = topiclist;
	selectionbox.value = mode+":"+topic;

	if(!streamRunning())
		connect();
}

selectionbox.addEventListener("change", (event) => {
	const separator = selectionbox.value.indexOf(":");
	mode = selectionbox.value.substring(0, separator);
	topic = selectionbox.value.substring(separator + 1);
	connect();
});

icon.addEventListener("click", ()=> {
	loadTopics();
});

loadTopics();

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

	if(canvas.naturalWidth == 0)
		return;

	const rotation = ((parseFloat(rotationbox.value) % 360) + 360) % 360;
	const isSideways = (rotation === 90 || rotation === 270);

	let img_width, img_height;
	if(isSideways) {
	    img_height = parseFloat(widthSlider.value);
	    img_width = (img_height * last_natural_width) / (vwToVh(last_natural_height));
	} else {
	    img_width = parseFloat(widthSlider.value);
	    img_height = (vwToVh(img_width) * last_natural_height) / last_natural_width;
	}

	const img_height_vw = img_height * window.innerHeight / window.innerWidth;
	const visual_width  = isSideways ? img_height_vw : img_width;
	const visual_height = isSideways ? img_width * window.innerWidth / window.innerHeight : img_height;

	[canvas, video].forEach(element => {
		element.style.width  = img_width  + "vw";
		element.style.height = img_height + "vh";
	});

	// Clamp position using the actual visual extents, not the pre-rotation ones
	let offset_x = clamp(x, visual_width  / 2, 100 - visual_width  / 2);
	let offset_y = clamp(y, visual_height / 2, 100 - visual_height / 2);

	imgpreview.style.left = offset_x + "vw";
	imgpreview.style.top  = offset_y + "vh";

	[canvas, video].forEach(element => {
		element.style.left = offset_x + "vw";
		element.style.top  = offset_y + "vh";
	});
}

function onResize(){
	displayImageOffset(img_offset_x, img_offset_y);
}

window.addEventListener('resize', onResize);

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
		img_offset_x = currentX / window.innerWidth  * 100;
		img_offset_y = currentY / window.innerHeight * 100;
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

displayImageOffset(img_offset_x, img_offset_y);

window.addEventListener("{uniqueID}_destroy", () => {
	clearTimeout(throttle_timer);
	disconnect();

	if(blob_url !== undefined)
		URL.revokeObjectURL(blob_url);

	window.removeEventListener("resize", onResize);
	document.removeEventListener("mousemove", onMove);
	document.removeEventListener("touchmove", onMove);
	document.removeEventListener("mouseup", onEnd);
	document.removeEventListener("touchend", onEnd);
}, {once: true});

console.log("Image Widget Loaded {uniqueID}")