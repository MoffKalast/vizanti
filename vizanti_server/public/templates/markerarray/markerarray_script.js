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

let listener = undefined;
let marker_topic = undefined;

let markers = {};
let z_sorted_keys = [];

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	throttle.value = loaded_data.throttle ?? 100;
}else{
	saveSettings();
}


function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		throttle: throttle.value
	}
	settings.save();
}



//Rendering

/* 
Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id                           # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type                         # Type of object
int32 action                         # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials */

function rgbaToFillColor(rosColorRGBA) {

	if(rosColorRGBA === undefined)
		return `white`;

	// Clamp the RGBA values between 0 and 1
	const r = Math.min(Math.max(rosColorRGBA.r, 0), 1);
	const g = Math.min(Math.max(rosColorRGBA.g, 0), 1);
	const b = Math.min(Math.max(rosColorRGBA.b, 0), 1);
	const a = Math.min(Math.max(rosColorRGBA.a, 0), 1);
  
	// Convert the RGBA values from the range [0, 1] to the range [0, 255]
	const r255 = Math.round(r * 255);
	const g255 = Math.round(g * 255);
	const b255 = Math.round(b * 255);
  
	// Return the RGBA color string for HTML canvas context
	return `rgba(${r255}, ${g255}, ${b255}, ${a})`;
}

function getContrastingColor(rosColorRGBA) {
	// Calculate luminance (per WCAG)
	return (0.299 * rosColorRGBA.r + 0.587 * rosColorRGBA.g + 0.114 * rosColorRGBA.b) > 0.5 ? '#161B21' : '#FFFFFF';
}

async function drawMarkers(){

	function drawCircle(marker, size){
		ctx.scale(marker.scale.x, marker.scale.y);
		ctx.beginPath();
		ctx.arc(0, 0, size/2, 0, 2 * Math.PI, false);
		ctx.fill();
	}

	function drawCube(marker, size){
		ctx.scale(marker.scale.x, marker.scale.y);
		ctx.fillRect(-size/2, -size/2, size, size);
	}

	function drawCubeList(marker, size) {
		ctx.scale(marker.scale.x, marker.scale.y);

		const sizeHalf = size / 2;
		const sizeDouble = size * 2;
		const topMap = new Map();
		
		// Z-culling with numeric keys and index storage
		marker.points.forEach((point, index) => {
			const keyX = Math.round(point.x * 2);
			const keyY = Math.round(point.y * 2);
			const key = keyX * 100000 + keyY;
			const existing = topMap.get(key);
			if (!existing || point.z > marker.points[existing].z) {
				topMap.set(key, index);
			}
		});
		
		const groups = new Map();
		for (const index of topMap.values()) {
			const color = rgbaToFillColor(marker.colors[index]);

			if (!groups.has(color))
				groups.set(color, []);

			groups.get(color).push(marker.points[index]);
		}

		groups.forEach((points, color) => {
			ctx.fillStyle = color;
			ctx.beginPath();
			points.forEach(point => {
				const x = point.x * sizeDouble - sizeHalf;
				const y = -point.y * sizeDouble - sizeHalf;
				ctx.moveTo(x, y);
				ctx.lineTo(x + size, y);
				ctx.lineTo(x + size, y + size);
				ctx.lineTo(x, y + size);
				ctx.closePath();
			});
			ctx.fill();
		});
	}

	function drawArrow(marker, size){
		const height = parseInt(size*marker.scale.x);
		const width = parseInt(size*0.2*marker.scale.y)+1;
		const tip = parseInt(size*0.3*marker.scale.x)+1;
		const tipwidth = parseInt(size*0.6*marker.scale.y)+1;

		ctx.beginPath();
		ctx.moveTo(0, -width);
		ctx.lineTo(height - tip, -width);
		ctx.lineTo(height - tip, -tipwidth);
		ctx.lineTo(height, 0);
		ctx.lineTo(height - tip, tipwidth);
		ctx.lineTo(height - tip, width);
		ctx.lineTo(0, width);
		ctx.lineTo(0, -width);
		ctx.fill();
	}

	function drawLine(marker, size){

		if(!marker.hasOwnProperty("colors") || marker.colors.length == 0)
			ctx.strokeStyle = rgbaToFillColor(marker.color);
		else
			ctx.strokeStyle = rgbaToFillColor(marker.colors[0]); // for now

		ctx.lineWidth = parseInt(marker.scale.x*size);

		ctx.beginPath();
		marker.points.forEach((point, index) => {
			const x = point.x * size;
			const y = -point.y * size;
			if (index === 0) {
				ctx.moveTo(x, y);
			} else {
				ctx.lineTo(x, y);
			}
		});

		ctx.stroke();
	}

	function drawLineList(marker, size){
		ctx.lineWidth = parseInt(marker.scale.x*size);

		// Draw lines between pairs of points: 0-1, 2-3, 4-5, etc.
		for(let i = 0; i < marker.points.length - 1; i += 2){
			const point1 = marker.points[i];
			const point2 = marker.points[i + 1];
			
			// Set color for this line segment
			if(!marker.hasOwnProperty("colors") || marker.colors.length === 0){
				ctx.strokeStyle = rgbaToFillColor(marker.color);
			} else if(marker.colors.length > i){
				// Create gradient from start to end point for per-vertex color
				const x1 = point1.x * size;
				const y1 = -point1.y * size;
				const x2 = point2.x * size;
				const y2 = -point2.y * size;
				
				const gradient = ctx.createLinearGradient(x1, y1, x2, y2);
				gradient.addColorStop(0, rgbaToFillColor(marker.colors[i]));
				gradient.addColorStop(1, rgbaToFillColor(marker.colors[i + 1] || marker.colors[i]));
				ctx.strokeStyle = gradient;
			}

			ctx.beginPath();
			ctx.moveTo(point1.x * size, -point1.y * size);
			ctx.lineTo(point2.x * size, -point2.y * size);
			ctx.stroke();
		}
	}

	function drawSphereList(marker, size) {
		const radius = (size * marker.scale.x) / 2; // Use scale.x for sphere diameter
		
		marker.points.forEach((point, index) => {
			// Set color for this sphere
			if (marker.hasOwnProperty("colors") && marker.colors.length > index) {
				ctx.fillStyle = rgbaToFillColor(marker.colors[index]);
			} else {
				ctx.fillStyle = rgbaToFillColor(marker.color);
			}
			
			// Draw the circle at the point position
			ctx.beginPath();
			ctx.arc(point.x * size, -point.y * size, radius, 0, 2 * Math.PI, false);
			ctx.fill();
		});
	}

	function drawText(marker, size) {
		ctx.scale(0.1, 0.1);
		const scale = size * marker.scale.z
		ctx.font = `${10 * scale}px Monospace`;
		ctx.textAlign = "center";
		ctx.fillStyle = rgbaToFillColor(marker.color);
		ctx.strokeStyle = getContrastingColor(marker.color)
		ctx.lineWidth = 2.5 * scale;
		ctx.lineJoin = 'round';
		ctx.miterLimit = 2;

		const lines = marker.text.split('\n');
		let maxAscent = 0;
		let maxDescent = 0;
		let lineHeight = 0;

		for (const line of lines) {
			const metrics = ctx.measureText(line);
			maxAscent = Math.max(maxAscent, metrics.actualBoundingBoxAscent);
			maxDescent = Math.max(maxDescent, metrics.actualBoundingBoxDescent);
		}
		lineHeight = (maxAscent + maxDescent) * 1.2; // 1.2 = line spacing factor

		// Compute vertical offset so that the text block is vertically centered
		const totalHeight = lines.length * lineHeight;
		ctx.translate(0, totalHeight / -2 + maxAscent);

		// Render each line
		for (let i = 0; i < lines.length; i++) {
			const y = i * lineHeight;
			ctx.strokeText(lines[i], 0, y);
			ctx.fillText(lines[i], 0, y);
		}
	}



	const unit = view.getMapUnitsInPixels(1.0);

	const wid = canvas.width;
    const hei = canvas.height;

	ctx.setTransform(1,0,0,1,0,0);
	ctx.clearRect(0, 0, wid, hei);

	let current_time = new Date();

	for (const key of z_sorted_keys) {
		const marker = markers[key];
		
		ctx.fillStyle = rgbaToFillColor(marker.color);

		const frame = tf.absoluteTransforms[marker.header.frame_id];

		if(!frame)
			continue;

		//skip old markers
		if((current_time - marker.stamp) / 1000.0 > marker.lifetime.sec + marker.lifetime.nanosec*1e-9)
			continue;

		const pos = view.fixedToScreen({
			x: marker.transformed.translation.x,
			y: marker.transformed.translation.y
		});

		const matrix = view.quaterionToProjectionMatrix(marker.transformed.rotation);
		ctx.setTransform(matrix[0], matrix[1], matrix[2], matrix[3], pos.x, pos.y); //sx,0,0,sy,px,py

		switch(marker.type)
		{
			case 0: drawArrow(marker, unit); break;//ARROW=0
			case 1: drawCube(marker, unit);break;//CUBE=1
			case 2: 
			case 3: drawCircle(marker, unit); break; //SPHERE=2 CYLINDER=3
			case 4: drawLine(marker, unit); break; //LINE_STRIP=4
			case 5: drawLineList(marker, unit); break; //LINE_LIST=5
			case 6:	drawCubeList(marker, unit); break; //CUBE_LIST=6
			case 7: drawSphereList(marker, unit); break; //SPHERE_LIST=7
			case 8: status.setWarn("POINTS markers are not supported yet."); break; //POINTS=8
			case 9: drawText(marker, unit); break;//TEXT_VIEW_FACING=9
			case 10: status.setWarn("MESH_RESOURCE markers are not supported yet."); break; //MESH_RESOURCE=10
			case 11: status.setWarn("TRIANGLE_LIST markers are not supported yet."); break; //TRIANGLE_LIST=11
		}
	}
}

//Topic
function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(marker_topic !== undefined){
		marker_topic.unsubscribe(listener);
	}

	marker_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'visualization_msgs/msg/MarkerArray',
		compression: rosbridge.compression,
		throttle_rate: parseInt(throttle.value)
	});

	status.setWarn("No data received.");
	
	listener = marker_topic.subscribe((msg) => {

		let error = false;
		msg.markers.forEach(m => {
			if(m.action == 3){
				markers = {};
				z_sorted_keys = [];
				return;
			}
			const id = m.ns + m.id;
			if(m.action == 2){
				if(markers.hasOwnProperty(id)){
					delete markers[id];
				}
				return;
			}

			const q = m.pose.orientation;
			if(q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0){
				m.pose.orientation = new Quaternion();
			}

			if(m.header.frame_id == ""){
				status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
				m.header.frame_id = tf.fixed_frame;
				error = true;
			}
		
			m.transformed = tf.transformPose(
				m.header.frame_id, 
				tf.fixed_frame, 
				m.pose.position, 
				m.pose.orientation
			);

			m.stamp = new Date();	
			markers[id] = m;
		});

		z_sorted_keys = Object.keys(markers).sort((a, b) => {
			const markerA = markers[a];
			const markerB = markers[b];
			return markerA.transformed.translation.z - markerB.transformed.translation.z;
		});

		if(!error){
			status.setOK();
		}
		drawMarkers();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("visualization_msgs/msg/MarkerArray");

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
	markers = {};
	z_sorted_keys = [];
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawMarkers();
}

window.addEventListener("tf_fixed_frame_changed", drawMarkers);
window.addEventListener("view_changed", drawMarkers);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("MarkerArray Widget Loaded {uniqueID}")