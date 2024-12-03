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
let imageToDataURL = utilModule.imageToDataURL;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const icon_pose = 'assets/pose.svg'//await imageToDataURL('assets/pose.svg');
const icon_pose_with_covariance = 'assets/posewithcovariancestamped.svg'//await imageToDataURL('assets/posewithcovariancestamped.svg');

let typedict = {};
let listener = undefined;
let marker_topic = undefined;

let posemsg = undefined;
let frame = "";

const selectionbox = document.getElementById("{uniqueID}_topic");
const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];

const scaleSlider = document.getElementById('{uniqueID}_scale');
const scaleSliderValue = document.getElementById('{uniqueID}_scale_value');
scaleSlider.addEventListener('change', saveSettings);
scaleSlider.addEventListener('input', function () {
	scaleSliderValue.textContent = this.value;
	saveSettings();
	drawMarkers();
});

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
colourpicker.addEventListener("input", (event) =>{
	utilModule.setIconColor(icon, colourpicker.value);
	saveSettings();
	drawMarkers();
});

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;

	colourpicker.value = loaded_data.color ?? "#f74127";

	scaleSlider.value = loaded_data.scale;
	scaleSliderValue.textContent = scaleSlider.value;

	typedict = loaded_data.typedict ?? {};

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
		scale: parseFloat(scaleSlider.value),
		typedict: typedict,
		color: colourpicker.value
	}
	settings.save();
}

async function drawMarkers(){

	function drawCircle(size){
		ctx.fillStyle = colourpicker.value;
		ctx.beginPath();
		ctx.arc(0, 0, size/2, 0, 2 * Math.PI, false);
		ctx.fill();
	}

	function drawArrow(size){
		ctx.fillStyle = colourpicker.value;
		const height = parseInt(size*2.0);
		const width = parseInt(size*0.1*0.6)+1;
		const tip = parseInt(size*0.24)+1;
		const tipwidth = parseInt(size*0.3*0.6)+1;

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
	function drawPizza(start_angle, end_angle, min_len, max_len){
        ctx.beginPath();
        ctx.arc(0, 0, min_len, start_angle, end_angle);
        ctx.lineTo(max_len * Math.cos(end_angle), max_len * Math.sin(end_angle));
        ctx.arc(0, 0, max_len, end_angle, start_angle, true);
        ctx.lineTo(min_len * Math.cos(start_angle), min_len * Math.sin(start_angle));
        ctx.closePath();
        ctx.fill();
	}
	
	function drawTranslationalCovariance(eigenvalues, size) {
		if (eigenvalues === undefined)
			return;
	
		const radiusX = Math.sqrt(eigenvalues.lambda1) * size;
		const radiusY = Math.sqrt(eigenvalues.lambda2) * size;
		const theta = Math.atan2(eigenvalues.eigenvector1[1], eigenvalues.eigenvector1[0]);

		ctx.fillStyle = 'rgba(204, 51, 204, 0.4)';
		ctx.rotate(theta);
		ctx.beginPath();
		ctx.ellipse(0, 0, radiusX, radiusY, 0, 0, 2 * Math.PI);
		ctx.fill();
	}

	function drawAngularCovariance(covariance, size) {

		if(covariance === undefined)
			return;

		// Calculate the 2-sigma standard deviation 
		let angleUncertainty = 2 * Math.sqrt(covariance[35]);

		if(angleUncertainty > Math.PI*2)
			angleUncertainty = Math.PI*2;

		const halfAngleZ = angleUncertainty * 0.5;

		ctx.fillStyle = 'rgba(255, 255, 0, 0.4)'; // Yellow, semi-transparent
		drawPizza(-halfAngleZ, halfAngleZ, 0, size*2);
	}

	const unit = view.getMapUnitsInPixels(1.0);

	const wid = canvas.width;
    const hei = canvas.height;

	ctx.setTransform(1,0,0,1,0,0);
	ctx.clearRect(0, 0, wid, hei);

	if(!posemsg)
		return;

	if(frame === tf.fixed_frame){

		const screenpos = view.fixedToScreen(posemsg);
		const scale = unit*parseFloat(scaleSlider.value);

		ctx.setTransform(1,0,0,-1,screenpos.x, screenpos.y); //sx,0,0,sy,px,py
		
		drawTranslationalCovariance(posemsg.eigenvalues, unit);
		ctx.setTransform(1,0,0,-1,screenpos.x, screenpos.y);

		if(!posemsg.rotation_invalid)
			ctx.rotate(posemsg.yaw);

		if(!posemsg.rotation_invalid){
			drawAngularCovariance(posemsg.covariance, scale);
			drawArrow(scale);
		}else{
			drawCircle(scale*0.4);
		}
	}
}

function calculateEigen(covariance){
	//2x2 covariance submatrix for x and y.
	const covMatrix = [
		[covariance[0], covariance[1]],
		[covariance[6], covariance[7]]
	];

	const a = covMatrix[0][0];
	const b = covMatrix[0][1];
	const c = covMatrix[1][1];
	const trace = a + c;
	const det = a * c - b * b;
	const lambda1 = trace / 2 + Math.sqrt(trace * trace / 4 - det);
	const lambda2 = trace / 2 - Math.sqrt(trace * trace / 4 - det);

	let eigenvector1, eigenvector2;

	if (b !== 0) {
		eigenvector1 = [lambda1 - c, b];
		eigenvector2 = [lambda2 - c, b];
	} else {
		eigenvector1 = [1, 0];  // If off-diagonal is 0, the eigenvectors are aligned with the axes.
		eigenvector2 = [0, 1];
	}

	const norm1 = Math.sqrt(eigenvector1[0] * eigenvector1[0] + eigenvector1[1] * eigenvector1[1]);
	const norm2 = Math.sqrt(eigenvector2[0] * eigenvector2[0] + eigenvector2[1] * eigenvector2[1]);

	eigenvector1 = [eigenvector1[0] / norm1, eigenvector1[1] / norm1];
	eigenvector2 = [eigenvector2[0] / norm2, eigenvector2[1] / norm2];

	return {
		eigenvector1: eigenvector1,
		eigenvector2: eigenvector2,
		lambda1: lambda1,
		lambda2: lambda2
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
		messageType : typedict[topic]
	});

	status.setWarn("No data received.");

	const skip_covariance = typedict[topic] == "geometry_msgs/msg/PoseStamped";
	icon.data = skip_covariance ? icon_pose : icon_pose_with_covariance;
	
	listener = marker_topic.subscribe((msg) => {

		let error = false;
		if(msg.header.frame_id == ""){
			status.setWarn("Transform frame is an empty string, falling back to fixed frame. Fix your publisher ;)");
			msg.header.frame_id = tf.fixed_frame;
			error = true;
		}
		
		if(!tf.absoluteTransforms[msg.header.frame_id]){
			status.setError("Required transform frame \""+msg.header.frame_id+"\" not found.");
			return;
		}

		frame = tf.fixed_frame;

		let pose = skip_covariance ? msg.pose : msg.pose.pose;

		let q = pose.orientation;
		const rotation_invalid = q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0

		if(rotation_invalid){
			status.setWarn("Received invalid rotation, defaulting to indentity quat.");
			q = new Quaternion();
		}

		const transformed = tf.transformPose(
			msg.header.frame_id, 
			tf.fixed_frame, 
			pose.position, 
			q
		);

		//Todo: transform covariance
		posemsg = {
			x: transformed.translation.x,
			y: transformed.translation.y,
			yaw: transformed.rotation.toEuler().h,
			rotation_invalid: rotation_invalid,
			covariance: skip_covariance ? undefined : msg.pose.covariance,
			eigenvalues: skip_covariance ? undefined : calculateEigen(msg.pose.covariance)
		};
	
		drawMarkers();
		
		if(!error){
			status.setOK();
		}
	});

	saveSettings();
}

async function loadTopics(){
	let pose_topics = await rosbridge.get_topics("geometry_msgs/msg/PoseStamped");
	let posecov_topics = await rosbridge.get_topics("geometry_msgs/msg/PoseWithCovarianceStamped");

	let topiclist = "";
	pose_topics.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+" (PoseStamped)</option>"
		typedict[element] = "geometry_msgs/msg/PoseStamped";
	});
	posecov_topics.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+" (PoseWithCovarianceStamped)</option>"
		typedict[element] = "geometry_msgs/msg/PoseWithCovarianceStamped";
	});
	selectionbox.innerHTML = topiclist

	if(topic == "")
		topic = selectionbox.value;
	else{
		if(pose_topics.includes(topic) || posecov_topics.includes(topic)){
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
	posemsg = undefined;
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

click_icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawMarkers();
}

window.addEventListener("tf_fixed_frame_changed", drawMarkers);
window.addEventListener("tf_changed", ()=>{
	if(frame != tf.fixed_frame){
		drawMarkers();
	}
});

window.addEventListener("view_changed", drawMarkers);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("Pose Widget Loaded {uniqueID}")

