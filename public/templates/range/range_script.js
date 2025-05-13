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

let data = {};

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');

const text_range = document.getElementById("{uniqueID}_rangetext");
const text_min = document.getElementById("{uniqueID}_rangemintext");
const text_max = document.getElementById("{uniqueID}_rangemaxtext");
const text_fov = document.getElementById("{uniqueID}_fovtext");
const text_type = document.getElementById("{uniqueID}_typetext");

opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

const decay = document.getElementById('{uniqueID}_decay');
decay.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;

	decay.value = loaded_data.decay ?? 2000;
	throttle.value = loaded_data.throttle ?? 100;
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		decay: decay.value,
		throttle: throttle.value
	}
	settings.save();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

async function drawRanges() {

	function drawCircle(min_size, max_size) {
        ctx.beginPath();
        ctx.arc(0, 0, (min_size+max_size)/2, 0, 2 * Math.PI);
        ctx.closePath();
		ctx.lineWidth = max_size - min_size;
        ctx.stroke();
		ctx.lineWidth = 1;
	  }

	function drawPizza(start_angle, end_angle, min_len, max_len){

		if(min_len <= 0)
			min_len = 1;

		if(max_len <= 0)
			max_len = min_len+1;

        ctx.beginPath();
        ctx.arc(0, 0, min_len, start_angle, end_angle);
        ctx.lineTo(max_len * Math.cos(end_angle), max_len * Math.sin(end_angle));
        ctx.arc(0, 0, max_len, end_angle, start_angle, true);
        ctx.lineTo(min_len * Math.cos(start_angle), min_len * Math.sin(start_angle));
        ctx.closePath();
        ctx.fill();
	} 

	const unit = view.getMapUnitsInPixels(1.0);

	const wid = canvas.width;
	const hei = canvas.height;

	ctx.setTransform(1,0,0,1,0,0);
	ctx.clearRect(0, 0, wid, hei);
	ctx.globalAlpha = opacitySlider.value;

	for (const [key, sample] of Object.entries(data)) {

		//skip old messages
		if(decay.value > 0 && new Date() - sample.stamp > decay.value)
			continue;

		if(sample.max_range == 0)
			continue;

		const pos = view.fixedToScreen({
			x: sample.pose.translation.x,
			y: sample.pose.translation.y,
		});

		const start_angle = -sample.field_of_view/2;
		const end_angle = sample.field_of_view/2;

		ctx.setTransform(1,0,0,-1,pos.x, pos.y); //sx,0,0,sy,px,py
		ctx.rotate(sample.yaw);

		if(sample.cone_half_width < sample.max_range)
		{
			ctx.fillStyle = "#33414e96";
			drawPizza(start_angle, end_angle, unit*sample.min_range, unit*sample.max_range, unit*sample.cone_half_width)

			ctx.fillStyle = "#5eb4ffff";
			let minarc = unit*sample.range-10;
	
			drawPizza(start_angle, end_angle, minarc, unit*sample.range, unit*sample.cone_half_width)
			
		}
		else
		{
			ctx.strokeStyle = "#33414e96";
			const scale = sample.cone_half_width / sample.max_range;
			const min_range = sample.min_range * scale * unit;
			const range = sample.range * scale * unit;

			drawCircle(min_range, unit*sample.cone_half_width);

			ctx.strokeStyle = "#5eb4ffff";
			let minarc = range-10;
			if(minarc < 0)
				minarc = 1;

			drawCircle(minarc, range)

		}

		yieldToMainThread();

	}
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawRanges();
}

window.addEventListener("tf_fixed_frame_changed", drawRanges);
window.addEventListener("view_changed", drawRanges);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

//Topic

const RADIATION_TYPE = {
	0: "Ultrasound",
	1: "Infrared"
}

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
		messageType : 'sensor_msgs/Range',
		throttle_rate: parseInt(throttle.value)
	});

	status.setWarn("No data received.");	
	listener = range_topic.subscribe((msg) => {

		let error = false;
		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
			msg.header.frame_id = tf.fixed_frame;
			error = true;
		}

		const pose = tf.absoluteTransforms[msg.header.frame_id];

		if(!pose){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

		text_range.innerText = "Range: "+msg.range.toFixed(3)+" m";
		text_min.innerText = "Min: "+msg.min_range.toFixed(3)+" m";
		text_max.innerText = "Max: "+msg.max_range.toFixed(3)+" m";
		text_fov.innerText = "Field of view: "+(msg.field_of_view * (180/Math.PI)).toFixed(2)+"°";
		text_type.innerText = "Type: "+RADIATION_TYPE[msg.radiation_type];

		const front_vector = tfModule.applyRotation(
			{
				x: msg.max_range, 
				y: 0,
				z: 0 
			}, 
			pose.rotation, 
			false
		);

		//calculate the new values for displaying the cone in a rotated projection
		const yaw = Math.atan2(front_vector.y, front_vector.x);
		const ratio = Math.hypot(front_vector.y, front_vector.x) / msg.max_range;

		const cone_half_width = Math.tan(msg.field_of_view * 0.5) * msg.max_range;
		const ratio_fov = 2 * Math.atan(cone_half_width / (ratio * msg.max_range));

		data[msg.header.frame_id] = {
			yaw: yaw,
			cone_half_width: cone_half_width,
			field_of_view: ratio_fov,
			min_range: ratio * msg.min_range,
			max_range: ratio * msg.max_range,
			range: ratio * msg.range,
			type: msg.radiation_type,
			pose: pose,
			stamp: new Date()
		}
		drawRanges();

		if(!error){
			status.setOK();
		}
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/Range");
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
	text_range.innerText = "Range: ?";
	text_min.innerText = "Min: ?";
	text_max.innerText = "Max: ?";
	text_fov.innerText = "Field of view: ?";
	text_type.innerText = "Type: ?";

	topic = selectionbox.value;
	connect();
});

selectionbox.addEventListener("click", connect);
icon.addEventListener("click", loadTopics);

loadTopics();
resizeScreen();

console.log("Range Widget Loaded {uniqueID}")