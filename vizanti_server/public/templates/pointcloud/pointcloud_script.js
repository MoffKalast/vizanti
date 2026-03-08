let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);

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
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

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

const maxPointsSlider = document.getElementById('{uniqueID}_max_points');
const maxPointsValue = document.getElementById('{uniqueID}_max_points_value');
maxPointsSlider.addEventListener('input', () =>  {
	maxPointsValue.textContent = maxPointsSlider.value;
	saveSettings();
});

const colourCountSlider = document.getElementById('{uniqueID}_colour_count');
const colourCountValue = document.getElementById('{uniqueID}_colour_count_value');
colourCountSlider.addEventListener('input', () =>  {
	colourCountValue.textContent = colourCountSlider.value;
	saveSettings();
});

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
});

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

const colourOverrideCheckbox = document.getElementById('{uniqueID}_color_override');
colourOverrideCheckbox.addEventListener('change', saveSettings);

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;

	thicknessSlider.value = loaded_data.thickness;
	thicknessValue.innerText = thicknessSlider.value;

	maxPointsSlider.value = loaded_data.max_points ?? 15000;
	maxPointsValue.innerText = maxPointsSlider.value;

	colourCountSlider.value = loaded_data.colour_count ?? 5;
	colourCountValue.innerText = colourCountSlider.value;

	colourpicker.value = loaded_data.color;
	throttle.value = loaded_data.throttle;

	colourOverrideCheckbox.checked = loaded_data.colour_override ?? false;
	
}else{
	saveSettings();
}

//update the icon colour when it's loaded or when the image source changes
icon.onload = () => {
	utilModule.setIconColor(icon, colourpicker.value);
};
if (icon.contentDocument) {
	utilModule.setIconColor(icon, colourpicker.value);
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		opacity: opacitySlider.value,
		thickness: thicknessSlider.value,
		color: colourpicker.value,
		throttle: throttle.value,
		max_points: maxPointsSlider.value,
		colour_count: colourCountSlider.value,
		colour_override: colourOverrideCheckbox.checked
	};
	settings.save();
	drawCloud();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

async function drawCloud() {
    const pixel = view.getMapUnitsInPixels(thicknessSlider.value);
    const wid = canvas.width;
    const hei = canvas.height;

    ctx.clearRect(0, 0, wid, hei);
    ctx.globalAlpha = opacitySlider.value;

    if(data == undefined)
		return;

    const delta = parseInt(pixel / 2);

    function drawGroup(points) {
        for(const pt of points){
            const screenpos = view.fixedToScreen(pt);
            const x = screenpos.x - delta;
            const y = screenpos.y - delta;
            ctx.moveTo(x, y);
            ctx.lineTo(x + pixel, y);
            ctx.lineTo(x + pixel, y + pixel);
            ctx.lineTo(x, y + pixel);
            ctx.lineTo(x, y);
        }
    }

    if(data.use_rgb){
		if(colourOverrideCheckbox.checked){
			ctx.fillStyle = colourpicker.value;
			ctx.beginPath();
			for(const group of data.groups){
            	drawGroup(group.points);
        	}
			ctx.fill();
		}else{
			for(const group of data.groups){
				ctx.fillStyle = group.color;
				ctx.beginPath();
            	drawGroup(group.points);
				ctx.fill();
        	}
		}
	}else{
		ctx.fillStyle = colourpicker.value;
        ctx.beginPath();
		drawGroup(data.points);
		ctx.fill();
	}

    ctx.restore();
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawCloud();
}

window.addEventListener("tf_fixed_frame_changed", drawCloud);
window.addEventListener("view_changed", drawCloud);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

function bytes_to_datatype(view, offset, type, littleEndian){
	switch(type){
		case 1: return parseFloat(view.getInt8(offset));
		case 2: return parseFloat(view.getUint8(offset));
		case 3: return parseFloat(view.getInt16(offset, littleEndian));
		case 4: return parseFloat(view.getUInt16(offset, littleEndian)); 
		case 5: return parseFloat(view.getInt32(offset, littleEndian)); 
		case 6: return parseFloat(view.getUInt32(offset, littleEndian)); 
		case 7: return view.getFloat32(offset, littleEndian); 
		case 8: return view.getFloat64(offset, littleEndian);
		default: return 0;
	}	
}


function histogramCut(points, numBuckets) {
	if(points.length === 0) return [];

	// Find range of each channel
	let minR = 255;
	let maxR = 0;
	
	let minG = 255;
	let maxG = 0;

	let minB = 255;
	let maxB = 0;
	for(const p of points){
		if(p.r < minR)
			minR = p.r;
		if(p.r > maxR)
			maxR = p.r;
		if(p.g < minG)
			minG = p.g;
		if(p.g > maxG)
			maxG = p.g;
		if(p.b < minB)
			minB = p.b;
		if(p.b > maxB)
			maxB = p.b;
	}

	// Pick the channel with the widest range to bin along
	const rRange = maxR - minR;
	const gRange = maxG - minG;
	const bRange = maxB - minB;
	const axis = rRange >= gRange && rRange >= bRange ? 'r' : gRange >= bRange ? 'g' : 'b';
	const axisMin = axis === 'r' ? minR : axis === 'g' ? minG : minB;
	const axisMax = axis === 'r' ? maxR : axis === 'g' ? maxG : maxB;
	const axisRange = axisMax - axisMin || 1;

	// Assign each point to a histogram bucket
	const buckets = Array.from({ length: numBuckets }, () => []);
	for(const p of points){
		const idx = Math.min(
			Math.floor(((p[axis] - axisMin) / axisRange) * numBuckets),
			numBuckets - 1
		);
		buckets[idx].push(p);
	}

	// Convert each non-empty bucket to a group with averaged colour
	const groups = [];
	for(const bucket of buckets){
		const n = bucket.length;

		if(n === 0)
			continue;

		let r = 0;
		let g = 0;
		let b = 0;

		for(const p of bucket){
			r += p.r;
			g += p.g;
			b += p.b;
		}
		
		const hex = '#' + [r/n, g/n, b/n].map(v => Math.round(v).toString(16).padStart(2, '0')).join('');
		groups.push({
			color: hex,
			points: bucket
		});
	}

	return groups;
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
		messageType : 'sensor_msgs/msg/PointCloud2',
		throttle_rate: parseInt(throttle.value),
		compression: rosbridge.compression
	});

	status.setWarn("No data received.");

	listener = range_topic.subscribe((msg) => {	

		let error = false;
		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame.");
			msg.header.frame_id = tf.fixed_frame;
			error = true;
		}

		const pose = tf.absoluteTransforms[msg.header.frame_id];
		if(!pose){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

		const numPoints = msg.width * msg.height;
		const xData = msg.fields.find(field => field.name === 'x');
		const yData = msg.fields.find(field => field.name === 'y');
		const zData = msg.fields.find(field => field.name === 'z');
		const rgbData = msg.fields.find(field => field.name === 'rgb');

		if(!xData || !yData || !zData){
			status.setError("XYZ coordinate data not found in cloud.");
			return;
		}

		const MAX_POINTS = maxPointsSlider.value;
		const COLOUR_COUNT = colourCountSlider.value;
		const sampleStep = numPoints <= MAX_POINTS ? 1 : Math.floor(numPoints / MAX_POINTS);
		const sampledCount = Math.ceil(numPoints / sampleStep);

		// Build sampled byte array — only copy bytes for points we'll actually use
		const sampledBytes = new Uint8Array(sampledCount * msg.point_step);

		if(typeof msg.data === 'string'){
			const binaryString = atob(msg.data);
			for(let i = 0, s = 0; i < numPoints; i += sampleStep, s++){
				const srcOffset = i * msg.point_step;
				for(let b = 0; b < msg.point_step; b++){
					sampledBytes[s * msg.point_step + b] = binaryString.charCodeAt(srcOffset + b);
				}
			}
		} else {
			let src;
			if(msg.data instanceof Uint8Array){
				src = msg.data;
			} else if(msg.data instanceof ArrayBuffer){
				src = new Uint8Array(msg.data);
			} else if(typeof msg.data === 'object' && msg.data[0] !== undefined){
				src = Uint8Array.from(msg.data);
			} else {
				status.setError("Cloud in unknown data type: " + typeof msg.data);
				return;
    }
			for(let i = 0, s = 0; i < numPoints; i += sampleStep, s++){
				sampledBytes.set(src.subarray(i * msg.point_step, (i + 1) * msg.point_step), s * msg.point_step);
			}
		}

		const littleEndian = !msg.is_bigendian;
		const dataview = new DataView(sampledBytes.buffer);
		let pointarray = [];

		if(!rgbData){

			for(let s = 0; s < sampledCount; s++){
				const byteOffset = s * msg.point_step;
				const point = {
					x: bytes_to_datatype(dataview, byteOffset + xData.offset, xData.datatype, littleEndian),
					y: bytes_to_datatype(dataview, byteOffset + yData.offset, yData.datatype, littleEndian),
					z: bytes_to_datatype(dataview, byteOffset + zData.offset, zData.datatype, littleEndian)
				};
				const transformed = tf.transformPose(msg.header.frame_id, tf.fixed_frame, point, new Quaternion()).translation;
				pointarray.push(transformed);
			}

			if(pointarray.length > 0){
				data = {
					use_rgb: false,
					points: pointarray
				};
				drawCloud();
				if(!error){
					status.setOK();
				}
			}

		}else{

			for(let s = 0; s < sampledCount; s++){
				const byteOffset = s * msg.point_step;
				const point = {
					x: bytes_to_datatype(dataview, byteOffset + xData.offset, xData.datatype, littleEndian),
					y: bytes_to_datatype(dataview, byteOffset + yData.offset, yData.datatype, littleEndian),
					z: bytes_to_datatype(dataview, byteOffset + zData.offset, zData.datatype, littleEndian)
				};
				const transformed = tf.transformPose(msg.header.frame_id, tf.fixed_frame, point, new Quaternion()).translation;

				if(rgbData){
					const bits = dataview.getUint32(byteOffset + rgbData.offset, littleEndian);
					transformed.r = (bits >> 16) & 0xFF;
					transformed.g = (bits >> 8)  & 0xFF;
					transformed.b = bits & 0xFF;
				}
				pointarray.push(transformed);
			}

			if(pointarray.length > 0){
				data = {
					use_rgb: true,
					groups: histogramCut(pointarray, COLOUR_COUNT)
				};

				drawCloud();
				if(!error){
					status.setOK();
				}
			}

		}

		
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/msg/PointCloud2");
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
click_icon.addEventListener("click", loadTopics);

loadTopics();
resizeScreen();

console.log("Point Cloud Widget Loaded {uniqueID}")