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

opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

const decay = document.getElementById('{uniqueID}_decay');
decay.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;

	decay.value = loaded_data.decay ?? 2000;
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		decay: decay.value
	}
	settings.save();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

async function drawRanges() {

	function drawCircle(min_size, max_size) {
		ctx.save();
        ctx.beginPath();
        ctx.arc(0, 0, (min_size+max_size)/2, 0, 2 * Math.PI);
        ctx.closePath();
		ctx.lineWidth = max_size - min_size;
        ctx.stroke();
		ctx.restore();
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

	ctx.clearRect(0, 0, wid, hei);
	ctx.globalAlpha = opacitySlider.value;

	for (const [key, sample] of Object.entries(data)) {

		//skip old messages
		if(new Date() - sample.stamp > decay.value)
			continue;

		if(sample.max_range == 0)
			continue;

		const pos = view.fixedToScreen({
			x: sample.pose.translation.x,
			y: sample.pose.translation.y,
		});

		const start_angle = -sample.field_of_view/2;
		const end_angle = sample.field_of_view/2;

		ctx.save();
		ctx.translate(pos.x, pos.y);
		ctx.scale(1.0, -1.0);
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



		ctx.restore();

		yieldToMainThread();

	}
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawRanges();
}

window.addEventListener("tf_changed", drawRanges);
window.addEventListener("view_changed", drawRanges);
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
		messageType : 'sensor_msgs/Range'
	});

	status.setWarn("No data received.");
	
	listener = range_topic.subscribe((msg) => {		
		const pose = tf.absoluteTransforms[msg.header.frame_id];

		if(!pose){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

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
		status.setOK();
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
	topic = selectionbox.value;
	connect();
});

selectionbox.addEventListener("click", connect);
icon.addEventListener("click", loadTopics);

loadTopics();
resizeScreen();

console.log("Range Widget Loaded {uniqueID}")