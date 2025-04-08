let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let tf = tfModule.tf;
let settings = persistentModule.settings;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;

let base_link_frame = "base_link";
let fixed_frame = tf.fixed_frame;
let units = "m/s";

let min_speed = Infinity;
let max_speed = 0;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const baseLinkFrameBox = document.getElementById("{uniqueID}_frame");
const fixedFrameBox = document.getElementById("{uniqueID}_fixed_frame");
const unitBox = document.getElementById("{uniqueID}_units");

const icondiv = document.getElementById("{uniqueID}_icon");
const icon = icondiv.getElementsByTagName('img')[0];
const icon_dial = icondiv.getElementsByTagName('img')[1];

const text_spd = document.getElementById("{uniqueID}_spd");
const text_spd_min = document.getElementById("{uniqueID}_spd_min");
const text_spd_max = document.getElementById("{uniqueID}_spd_max");
const text_dial = document.getElementById("{uniqueID}_dialtext");
const text_unit = document.getElementById("{uniqueID}_unittext");

const reset_button = document.getElementById("{uniqueID}_reset");
reset_button.addEventListener("click",resetTracking);

function resetTracking(){
	speed_samples = [];
	min_speed = Infinity;
	max_speed = 0;
}


if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	base_link_frame = loaded_data.frame;
	fixed_frame = loaded_data.fixed_frame;
	units = loaded_data.units;

	baseLinkFrameBox.value = base_link_frame;
	fixedFrameBox.value = fixed_frame;
	unitBox.value = units;

}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		frame: baseLinkFrameBox.value,
		fixed_frame: fixedFrameBox.value,
		units: unitBox.value,
	}
	settings.save();
}

let prev = undefined;
let prev_stamp = undefined;
let speed_samples = [];
const sample_size = 10;  // Number of samples to average
const smoothing_factor = 0.5;  // Low-pass filter smoothing factor (0.0 - 1.0)

function writeText(speed_ms, minspeed_ms, maxspeed_ms){
	let mult = 1.0;

	text_unit.innerText = units;

	switch(units){
		case 'km/h': mult = 3.6; break;
		case 'kts': mult = 1.943844; break;
		case 'mph': mult = 2.236936; break;
		case 'ft/s': mult = 3.28084; break;
		case 'b/s': mult = 6.56167; break;
		default: break;
	}

	function formatDial(num) {
		if (num >= 10000) return "9999";
		if (num >= 1000) return Math.floor(num).toString();
		if (num >= 100) return num.toFixed(2);
		if (num >= 10) return num.toFixed(2);
		return num.toFixed(3);
	}

	text_spd.innerText = "Speed: " + (speed_ms * mult).toFixed(2) + " " + units;
    text_spd_min.innerText = "Min Speed: " + (minspeed_ms * mult).toFixed(2) + " " + units;
    text_spd_max.innerText = "Max Speed: " + (maxspeed_ms * mult).toFixed(2) + " " + units;
    text_dial.innerText = formatDial(speed_ms * mult);
}

function calculateSpeed() {

    if (!tf.absoluteTransforms[base_link_frame] || !tf.frame_headerstamps[base_link_frame]) {
        status.setError("Required transform frame \"" + base_link_frame + "\" not found.");
        return;
    }

    if (!tf.absoluteTransforms[fixed_frame]) {
        status.setError("Required transform frame \"" + fixed_frame + "\" not found.");
        return;
    }

    const transformed = tf.transformPose(
        base_link_frame,
        fixed_frame,
        { x: 0, y: 0, z: 0 },
        new Quaternion()
    );

    status.setOK();

    if (prev === undefined) {
        prev = transformed;
        prev_stamp = tf.frame_headerstamps[base_link_frame];
        return;
    }

    const time_now = tf.frame_headerstamps[base_link_frame];
    const deltaTimeSec = tf.getTimeStampDelta(prev_stamp, time_now);

    // Skip duplicated calls
    if (deltaTimeSec < 0.01){
		prev = transformed;
    	prev_stamp = time_now;
		return;
	}

	const dist = Math.hypot(
        prev.translation.x - transformed.translation.x,
        prev.translation.y - transformed.translation.y,
        prev.translation.z - transformed.translation.z
    );

    let current_speed = dist / deltaTimeSec;

    // Low-pass filter
    if (speed_samples.length > 0) {
        current_speed = (current_speed * smoothing_factor) + (speed_samples[speed_samples.length - 1] * (1.0 - smoothing_factor));
    }

    // Add new sample
    speed_samples.push(current_speed);
    if (speed_samples.length > sample_size) {
        speed_samples.shift();
    }

    const average_speed = speed_samples.reduce((a, b) => a + b, 0) / speed_samples.length;
    min_speed = Math.min(min_speed, average_speed);
    max_speed = Math.max(max_speed, average_speed);

	//dial needle
	const angle = ((average_speed - min_speed) / (max_speed - min_speed)) * 195;
	icon_dial.style.transform = "rotate(" + angle + "deg)";

	writeText(average_speed, min_speed, max_speed);

    prev = transformed;
    prev_stamp = time_now;
}

window.addEventListener("tf_fixed_frame_changed", calculateSpeed);
window.addEventListener("tf_changed", calculateSpeed);

// TF frame list
function setFrameList(){
	//find world frames
	let framelist = "";
	for (const key of tf.frame_list.values()) {
		framelist += "<option value='"+key+"'>"+key+"</option>"
	}
	baseLinkFrameBox.innerHTML = framelist;
	fixedFrameBox.innerHTML = framelist;

	if(tf.frame_list.has(fixed_frame)){
		fixedFrameBox.value = fixed_frame;
	}else{
		fixedFrameBox.innerHTML = framelist + "<option value='"+fixed_frame+"'>"+fixed_frame+"</option>";
		fixedFrameBox.value = fixed_frame;
	}

	if(tf.frame_list.has(base_link_frame)){
		baseLinkFrameBox.value = base_link_frame;
	}else{
		baseLinkFrameBox.innerHTML = framelist + "<option value='"+base_link_frame+"'>"+base_link_frame+"</option>";
		baseLinkFrameBox.value = base_link_frame;
	}
}

baseLinkFrameBox.addEventListener("change", (event) => {
	base_link_frame = baseLinkFrameBox.value;
	saveSettings();
	resetTracking();
});

fixedFrameBox.addEventListener("change", (event) => {
	fixed_frame = fixedFrameBox.value;
	saveSettings();
	resetTracking();
});

unitBox.addEventListener("change", (event) => {
	units = unitBox.value;
	saveSettings();
});

baseLinkFrameBox.addEventListener("click", setFrameList);
fixedFrameBox.addEventListener("click", setFrameList);
icon.addEventListener("click", setFrameList);

setFrameList();

console.log("Speedometer Widget Loaded {uniqueID}")
