let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let pathsModule = await import(`${base_url}/assets/robot_model/paths`);

let view = viewModule.view;
let tf = tfModule.tf;
let settings = persistentModule.settings;
let Status = StatusModule.Status;
let paths = pathsModule.default;

let models = {};
paths.map(file => {
	const name = file.split('.png')[0].split("_")[1];
	models[name] = new Image();
	models[name].src = `${base_url}/assets/robot_model/${file}`;
});

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const frameSelector = document.getElementById("{uniqueID}_frame");
const spriteSelector = document.getElementById("{uniqueID}_sprite");
const lengthSelector = document.getElementById("{uniqueID}_length");
const previewImg = document.getElementById("{uniqueID}_previewimg");

let frame = "";
let sprite = "4wd";

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	frame = loaded_data.frame;
	lengthSelector.value = loaded_data.length;

	sprite = loaded_data.sprite;
	spriteSelector.value = sprite;
	previewImg.src = models[sprite].src;
}else{

	if(frame == ""){
		frame = "base_link";
		status.setWarn("No frame found, defaulting to base_link");
	}

	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		frame: frame,
		sprite: sprite,
		length: lengthSelector.value
	}
	settings.save();
}

async function drawRobot() {

	const unit = view.getMapUnitsInPixels(lengthSelector.value);

    const wid = canvas.width;
    const hei = canvas.height;

    ctx.clearRect(0, 0, wid, hei);

	const robotframe = tf.absoluteTransforms[frame];
	const modelimg = models[sprite];

	if(robotframe){
		let pos = view.fixedToScreen({
			x: robotframe.translation.x,
			y: robotframe.translation.y,
		});
	
		let yaw = robotframe.rotation.toEuler().h;

		let ratio = modelimg.naturalHeight/modelimg.naturalWidth;

		ctx.save();
		ctx.translate(pos.x, pos.y);
		ctx.scale(1.0, 1.0);
		ctx.rotate(Math.PI-yaw);
		ctx.drawImage(modelimg, -unit/2, -(unit*ratio)/2, unit, unit*ratio);
		ctx.restore();
		status.setOK();
	}else{
		status.setError("Required transform frame \""+frame+"\" not found.");
	}
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawRobot();
}

window.addEventListener("tf_changed", drawRobot);
window.addEventListener("view_changed", drawRobot);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

// TF frame list
function setFrameList(){

	let framelist = "";
	for (const key of tf.frame_list.values()) {
		framelist += "<option value='"+key+"'>"+key+"</option>"
	}
	frameSelector.innerHTML = framelist;

	if(tf.transforms.hasOwnProperty(frame)){
		frameSelector.value = frame;
	}else{
		framelist += "<option value='"+frame+"'>"+frame+"</option>"
		frameSelector.innerHTML = framelist
		frameSelector.value = frame;
	}

	let spritelist = "";
	for (const [key, value] of Object.entries(models)) {
		spritelist += "<option value='"+key+"'>"+key+"</option>"
	}

	spriteSelector.innerHTML = spritelist;
	spriteSelector.value = sprite;
}

frameSelector.addEventListener("change", (event) => {
	frame = frameSelector.value;
	saveSettings();
});

spriteSelector.addEventListener("change", (event) => {
	sprite = spriteSelector.value;
	previewImg.src = models[sprite].src;
	saveSettings();
});

lengthSelector.addEventListener("input", saveSettings);

frameSelector.addEventListener("click", setFrameList);
icon.addEventListener("click", setFrameList);

frameSelector.addEventListener("change", (event) =>{
	frame = frameSelector.value;
	drawRobot();
	saveSettings();
});

resizeScreen();

console.log("Model Widget Loaded {uniqueID}")