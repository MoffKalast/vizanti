import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';
import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';



let topic = "";
let listener = undefined;
let marker_topic = undefined;

let markers = {};

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic
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

function rgbaToCanvasColor(rosColorRGBA) {

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

function drawMarkers(){

	function drawCircle(size){
		ctx.beginPath();
		ctx.arc(0, 0, size, 0, 2 * Math.PI, false);
		ctx.fill();
	}

	function drawCube(size){
		ctx.fillRect(-size/2, -size/2, size, size);
	}

	const unit = view.getMapUnitsInPixels(1.0);

	const wid = canvas.width;
    const hei = canvas.height;

	ctx.clearRect(0, 0, wid, hei);

	for (const [key, marker] of Object.entries(markers)) {
		ctx.fillStyle = rgbaToCanvasColor(marker.color);

		const pos = view.mapToScreen({
			x: marker.pose.position.x,
			y: marker.pose.position.y
		});

		const yaw = (new Quaternion(
			marker.pose.orientation.w,
			marker.pose.orientation.x,
			marker.pose.orientation.y,
			marker.pose.orientation.z
		)).toEuler().h;

		ctx.save();
		ctx.translate(pos.x, pos.y);
		ctx.scale(marker.scale.x, marker.scale.y);
		ctx.rotate(yaw);

		switch(marker.type)
		{
			case 0: break;//ARROW=0
			case 1: drawCube(unit);break;//CUBE=1
			case 2: 
			case 3: drawCircle(unit); break; //SPHERE=2 CYLINDER=3
			case 4: break; //LINE_STRIP=4
			case 5: break; //LINE_LIST=5
			case 6: break; //CUBE_LIST=6
			case 7: break; //SPHERE_LIST=7
			case 8: break; //POINTS=8
			case 9: break; //TEXT_VIEW_FACING=9
			case 10: break; //MESH_RESOURCE=10
			case 11: break; //TRIANGLE_LIST=11
		}
		ctx.restore();
	}
}

//Topic
function connect(){

	if(topic == "")
		return;

	if(marker_topic !== undefined){
		marker_topic.unsubscribe(listener);
	}

	marker_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'visualization_msgs/MarkerArray',
		throttle_rate: 40
	});
	
	listener = marker_topic.subscribe((msg) => {
		msg.markers.forEach(m => {
			if(m.action == 3){
				markers = {};
				return;
			}
			const id = m.ns + m.id;
			if(m.action == 2){
				if(markers.hasOwnProperty(id)){
					delete markers[id];
				}
				return;
			}
		
			markers[id] = m;
		});
		drawMarkers();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("visualization_msgs/MarkerArray");

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

window.addEventListener("tf_changed", drawMarkers);
window.addEventListener("view_changed", drawMarkers);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("MarkerArray Widget Loaded {uniqueID}")

