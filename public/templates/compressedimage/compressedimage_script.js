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

const clamp = (num, min, max) => Math.min(Math.max(num, min), max);
const vwToVh = vw => (vw * window.innerWidth) / window.innerHeight;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

//persistent loading, so we don't re-fetch on every update
let stock_images = {};
stock_images["loading"] = await imageToDataURL("assets/tile_loading.png");
stock_images["error"] = await imageToDataURL("assets/tile_error.png");

let image_topic = undefined;
let listener = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const rotationbox = document.getElementById("{uniqueID}_rotation");

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const canvas = document.getElementById('{uniqueID}_image');
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

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

rotationbox.addEventListener("change", (event) => {
	saveSettings();
});

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	img_offset_x = loaded_data.img_offset_x;
	img_offset_y = loaded_data.img_offset_y;

	throttle.value = loaded_data.throttle;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
	canvas.style.opacity = loaded_data.opacity;

	widthSlider.value = loaded_data.width;
	widthValue.innerText = loaded_data.width;
	rotationbox.value = loaded_data.rotation;

	canvas.style.transform = `translate(-50%, -50%) rotate(${loaded_data.rotation}deg)`;
	displayImageOffset(img_offset_x, img_offset_y);
}else{
	displayImageOffset(0, 100);
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		throttle: throttle.value,
		width: widthSlider.value,
		img_offset_x: img_offset_x,
		img_offset_y: img_offset_y,
		rotation: rotationbox.value
	}
	settings.save();

	canvas.style.opacity = opacitySlider.value;
	canvas.style.transform = `translate(-50%, -50%) rotate(${rotationbox.value}deg)`;
	displayImageOffset(img_offset_x, img_offset_y);
}

//Topic

async function getImage(src) {
    return new Promise((resolve, reject) => {
        let img = new Image();
        img.onload = () => resolve(src);
        img.onerror = () => reject(src);
        img.src = src;
    });
}

function connect(){

	canvas.src = stock_images["loading"];
	displayImageOffset(img_offset_x, img_offset_y);

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(image_topic !== undefined){
		image_topic.unsubscribe(listener);
	}

	status.setWarn("No data received.");

	image_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/CompressedImage',
		throttle_rate: parseInt(throttle.value)
	});
	
	let received = false;
	listener = image_topic.subscribe(async (msg) => {  
		const src = 'data:image/jpeg;base64,' + msg.data

		getImage(src)
			.then(() => {
				canvas.src = src;
				if(!received){
					displayImageOffset(img_offset_x, img_offset_y);
					status.setOK();
				}
			})
			.catch((e) => {
				canvas.src = stock_images["error"];
				displayImageOffset(img_offset_x, img_offset_y);
				status.setError(e.message);
			});
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/CompressedImage");
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
	connect();
});

selectionbox.addEventListener("click", connect);

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

	let img_width = widthSlider.value;
	let img_height = (vwToVh(img_width) * canvas.naturalHeight)/canvas.naturalWidth;

	canvas.style.width = img_width+"vw";
	canvas.style.height = img_height+"vh";

	let offset_x = clamp(x, img_width/2, 100 - img_width/2);
	let offset_y = clamp(y, img_height/2, 100 - img_height/2);

	imgpreview.style.left = offset_x+"vw";
	imgpreview.style.top = offset_y+"vh";

	canvas.style.left = offset_x+"vw";
	canvas.style.top = offset_y+"vh";
}

window.addEventListener('resize', ()=>{
	displayImageOffset(img_offset_x, img_offset_y);
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
	
		let img_width = widthSlider.value/2;
		let img_height = (vwToVh(img_width) * canvas.naturalHeight)/canvas.naturalWidth;
	
		img_offset_x = clamp(currentX/window.innerWidth * 100, img_width, 100 - img_width);
		img_offset_y = clamp(currentY/window.innerHeight * 100, img_height, 100 - img_height);

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

console.log("Image Widget Loaded {uniqueID}")

