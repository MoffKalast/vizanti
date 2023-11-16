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
let data = undefined;

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');
opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

const thicknessSlider = document.getElementById('{uniqueID}_thickness');
const thicknessValue = document.getElementById('{uniqueID}_thickness_value');
thicknessSlider.addEventListener('input', () =>  {
	thicknessValue.textContent = thicknessSlider.value;
	saveSettings();
});

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	saveSettings();
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

	thicknessSlider.value = loaded_data.thickness;
	thicknessValue.innerText = loaded_data.thickness;

	colourpicker.value = loaded_data.color;
	throttle.value = loaded_data.throttle;
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		thickness: thicknessSlider.value,
		color: colourpicker.value,
		throttle: throttle.value
	};
	settings.save();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

async function drawCloud() {

	const pixel = view.getMapUnitsInPixels(thicknessSlider.value);

	const wid = canvas.width;
	const hei = canvas.height;

	ctx.clearRect(0, 0, wid, hei);
	ctx.globalAlpha = opacitySlider.value;
	ctx.fillStyle = colourpicker.value;

	if(data == undefined){
		return;
	}

	let delta = parseInt(pixel/2);

	data.points.forEach((transform) => {
		const screenpos = view.fixedToScreen(transform.translation);
		ctx.fillRect(
			screenpos.x - delta,
			screenpos.y - delta,
			pixel,
			pixel
		);
	});
	
	ctx.restore();
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawCloud();
}

window.addEventListener("tf_changed", drawCloud);
window.addEventListener("view_changed", drawCloud);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

function bytes_to_datatype(view, offset, type, littleEndian){
	switch(type){
		case 1: return parseFloat(view.getInt8(offset)); //INT8    = 1
		case 2: return parseFloat(view.getUint8(offset)); //UINT8   = 2
		case 3: return parseFloat(view.getInt16(offset, littleEndian)); //INT16   = 3
		case 4: return parseFloat(view.getUInt16(offset, littleEndian)); //UINT16  = 4
		case 5: return parseFloat(view.getInt32(offset, littleEndian)); //INT32   = 5
		case 6: return parseFloat(view.getUInt32(offset, littleEndian)); //UINT32  = 6
		case 7: return view.getFloat32(offset, littleEndian); //FLOAT32 = 7
		case 8: return view.getFloat64(offset, littleEndian); //FLOAT64 = 8
		default: return 0;
	}	
}

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
		messageType : 'sensor_msgs/PointCloud2',
		throttle_rate: parseInt(throttle.value),
		compression: "cbor"
	});

	status.setWarn("No data received.");

	listener = range_topic.subscribe((msg) => {	

		const pose = tf.absoluteTransforms[msg.header.frame_id];

		if(!pose){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

		/* if(msg.width * msg.height != msg.data.length / msg.point_step){
			status.setError("Invalid cloud data, point count is inconsistent with binary blob length.");
			return;
		} */

		const xData = msg.fields.find(field => field.name === 'x');
		const yData = msg.fields.find(field => field.name === 'y');
		const zData = msg.fields.find(field => field.name === 'z');

		if(xData === undefined || yData === undefined || zData === undefined){
			status.setError("XYZ coordinate data not found in cloud.");
			return;
		}

		const littleEndian = !msg.is_bigendian;

		const buffer = new ArrayBuffer(msg.data.length);
		const dataview = new DataView(buffer);

		//for some reason these two aren't equal, so we have to throw each byte in separately like cavemen, not great but works for now
		//console.log(buffer.slice(0, msg.point_step))
		//console.log(msg.data.buffer.slice(0, msg.point_step))
		for(let i = 0; i < msg.data.length; i++){
			dataview.setUint8(i, msg.data[i]);
		}

		let pointarray = [];
		for(let i = 0; i < msg.data.length; i += msg.point_step){
			//const dataview = new DataView(msg.data.buffer.slice(i, i+msg.point_step));
			
			const point = {
				x: bytes_to_datatype(dataview, i+xData.offset, xData.datatype, littleEndian),
				y: bytes_to_datatype(dataview, i+yData.offset, yData.datatype, littleEndian),
				z: bytes_to_datatype(dataview, i+zData.offset, zData.datatype, littleEndian)
			};
			pointarray.push(tf.transformPose(msg.header.frame_id, tf.fixed_frame, point, new Quaternion()));
		}

		if(pointarray.length > 0){
			data = {};
			data.pose = pose;
			data.points = pointarray;
			status.setOK();
			drawCloud();
		}
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/PointCloud2");
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
icon.addEventListener("click", loadTopics);

loadTopics();
resizeScreen();

console.log("Point Cloud Widget Loaded {uniqueID}")