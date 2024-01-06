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

let fixed_frame = "";
let base_link_frame = "";
let seq = 0;
let active = false;
let points = [];

const icon = document.getElementById("{uniqueID}_icon");

const startButton = document.getElementById("{uniqueID}_start");
const stopButton = document.getElementById("{uniqueID}_stop");

startButton.addEventListener('click', ()=>{
	console.log("Points:",points.slice(getStartIndex()))
	if(startCheckbox.checked)
		sendMessage(points.slice(getStartIndex()))
	else
		sendMessage(points)
});

stopButton.addEventListener('click', ()=>{
	sendMessage([])
});

const flipButton = document.getElementById("{uniqueID}_flip");
const deleteButton = document.getElementById("{uniqueID}_delete");

flipButton.addEventListener('click', ()=>{
	points.reverse();
	drawWaypoints();
	saveSettings();
});

deleteButton.addEventListener('click', async ()=>{
	if(await confirm("Are you sure you want to delete all waypoints?")){
		points = [];
		drawWaypoints();
		saveSettings();
	}
});

const startCheckbox = document.getElementById('{uniqueID}_startclosest');
startCheckbox.addEventListener('change', saveSettings);

// Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	points = loaded_data.points;
	fixed_frame = loaded_data.fixed_frame;
	base_link_frame = loaded_data.base_link_frame ?? "base_link";

	startCheckbox.checked = loaded_data.start_closest;
}else{
	saveSettings();
}

if(topic == ""){
	topic = "/move_base_simple/waypoints";
	status.setWarn("No topic found, defaulting to /move_base_simple/waypoints");
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		fixed_frame: fixed_frame,
		base_link_frame: base_link_frame,
		points: points,
		start_closest: startCheckbox.checked
	}
	settings.save();
}

// Message sending

function getStamp(){
	const currentTime = new Date();
	const currentTimeSecs = Math.floor(currentTime.getTime() / 1000);
	const currentTimeNsecs = (currentTime.getTime() % 1000) * 1e6;

	return {
		secs: currentTimeSecs,
		nsecs: currentTimeNsecs
	}
}

function sendMessage(pointlist){
	let timeStamp = getStamp();
	let poseList = [];

	if(pointlist.length > 0)
	{
		if(pointlist.length  == 1){
			poseList.push(new ROSLIB.Message({
				header: {
					seq: 0,
					stamp: timeStamp,
					frame_id: fixed_frame
				},
				pose: {
					position: {
						x: pointlist[0].x,
						y: pointlist[0].y,
						z: 0.0
					},
					orientation: new Quaternion()
				}
			}));
		}
		else
		{
			pointlist.forEach((point, index) => {
				let p0;
				let p1;

				if(index < pointlist.length-1){
					p0 = point;
					p1 = pointlist[index+1];
				}else{
					p0 = pointlist[index-1];
					p1 = point;
				}

				poseList.push(new ROSLIB.Message({
					header: {
						seq: index,
						stamp: timeStamp,
						frame_id: fixed_frame
					},
					pose: {
						position: {
							x: point.x,
							y: point.y,
							z: 0.0
						},
						orientation: Quaternion.fromEuler(Math.atan2(p0.y - p1.y, -(p0.x - p1.x)), 0, 0, 'ZXY')
					}
				}));
			});
		}
	}

	const publisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: 'nav_msgs/Path',
		latched: true
	});

	const pathMessage = new ROSLIB.Message({
		header: {
			seq: seq++,
			stamp: timeStamp,
			frame_id: fixed_frame
		},
		poses: poseList
	});
	publisher.publish(pathMessage);

	status.setOK();
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

const view_container = document.getElementById("view_container");

function getStartIndex(){

	if(base_link_frame == ""){
		status.setError("Base link frame not selected or the TF data is missing.");
		return 0;
	}

	let link = tf.transformPose(
		base_link_frame, 
		fixed_frame, 
		{x: 0, y: 0, z: 0}, 
		new Quaternion()
	);

    let minDistance = Number.POSITIVE_INFINITY;
    let minIndex = 0;

    for (let i = 0; i < points.length; i++) {
        let distance = 0;

		distance += Math.pow((link.translation.x - points[i].x), 2);
		distance += Math.pow((link.translation.y - points[i].y), 2);
		distance += Math.pow((link.translation.z - points[i].z), 2);

        if (distance < minDistance) {
            minDistance = distance;
            minIndex = i;
        }
    }
    return minIndex;
}

function pointToScreen(point){
	let transformed = tf.transformPose(
		fixed_frame, 
		tf.fixed_frame, 
		point, 
		new Quaternion()
	);

	return view.fixedToScreen({
		x: transformed.translation.x,
		y: transformed.translation.y
	});
}

function screenToPoint(click){
	let transformed = view.screenToFixed(click);
	return tf.transformPose(
		tf.fixed_frame, 
		fixed_frame, 
		transformed, 
		new Quaternion()
	).translation;
}

function drawWaypoints() {

    const wid = canvas.width;
    const hei = canvas.height;

    ctx.clearRect(0, 0, wid, hei);
	ctx.lineWidth = 3;
	ctx.strokeStyle = "#EBCE00"; 
	ctx.fillStyle = active ? "white" : "#EBCE00";

	const frame = tf.absoluteTransforms[fixed_frame];

	if(!frame){
		status.setError("Fixed transform frame not selected or the TF data is missing.");
		return;
	}

	const startIndex = getStartIndex();
	const viewPoints = points.map((point) =>
		pointToScreen(point)
	);

	if(startCheckbox.checked)
		ctx.strokeStyle = "#4a4a4a";
	else
		ctx.strokeStyle = "#EBCE00";

	ctx.beginPath();
	viewPoints.forEach((pos, index) => {

		if(index == startIndex && startCheckbox.checked){
			ctx.lineTo(pos.x, pos.y);
			ctx.stroke();
			ctx.strokeStyle = "#EBCE00"; 
			ctx.beginPath();
		}

		if (index === 0) {
			ctx.moveTo(pos.x, pos.y);
		} else {
			ctx.lineTo(pos.x, pos.y);
		}
	});
	ctx.stroke();

	viewPoints.forEach((pos, index) => {		
		ctx.save();
		ctx.translate(pos.x, pos.y);

		ctx.fillStyle = "#292929";
		ctx.beginPath();
		ctx.arc(0, 0, 12, 0, 2 * Math.PI, false);
		ctx.fill();

		if(index < startIndex && startCheckbox.checked){
			ctx.fillStyle = active ? "white" : "#827c52";
		}else{
			ctx.fillStyle = active ? "white" : "#EBCE00";
		}

		ctx.beginPath();
		ctx.arc(0, 0, 9, 0, 2 * Math.PI, false);
		ctx.fill();

		ctx.restore();
	});

	ctx.font = "bold 13px Monospace";
	ctx.textAlign = "center";
	ctx.fillStyle = "#212E4A";

	viewPoints.forEach((pos, index) => {
		ctx.fillText(index, pos.x, pos.y+5);
	});

	status.setOK();
}

let start_point = undefined;
let delta = undefined;
let drag_point = -1;

function findPoint(newpoint){
	let i = -1;
	points.forEach((point, index) => {
		const screenpoint = pointToScreen(point);
		const dist = Math.hypot(
			screenpoint.x - newpoint.x,
			screenpoint.y - newpoint.y,
		)
		if(dist < 15){
			i = index;
		}
	});
	return i;
}

function startDrag(event){
	const { clientX, clientY } = event.touches ? event.touches[0] : event;
	start_point = {
		x: clientX,
		y: clientY
	};

	drag_point = findPoint(start_point);
	if(drag_point >= 0){
		view.setInputMovementEnabled(false);
	}
}

function drag(event){
	const { clientX, clientY } = event.touches ? event.touches[0] : event;
	if(drag_point >= 0){
		points[drag_point] = screenToPoint({
			x: clientX,
			y: clientY
		})
		drawWaypoints();
	}

	if (start_point === undefined) 
		return;

	delta = {
		x: start_point.x - clientX,
		y: start_point.y - clientY,
	};
}

function distancePointToLineSegment(px, py, x1, y1, x2, y2) {
	const dx = x2 - x1;
	const dy = y2 - y1;
	const lengthSquared = dx * dx + dy * dy;

	let t = ((px - x1) * dx + (py - y1) * dy) / lengthSquared;
	t = Math.max(0, Math.min(1, t));

	const closestX = x1 + t * dx;
	const closestY = y1 + t * dy;

	const distanceSquared = (px - closestX) * (px - closestX) + (py - closestY) * (py - closestY);

	return Math.sqrt(distanceSquared);
}

function endDrag(event){

	if(drag_point >= 0){
		view.setInputMovementEnabled(true);
		drag_point = -1;
	}

	let moveDist = 0;

	if(delta !== undefined){
		moveDist = Math.hypot(delta.x,delta.y);
	}

	if(moveDist < 10){

		const { clientX, clientY } = event.touches ? event.touches[0] : event;
		const newpoint = {
			x: clientX,
			y: clientY
		};

		let index = findPoint(newpoint);

		if(index >= 0)
			points.splice(index, 1);
		else
		{
			let after = -1;
			for (let i = 0; i < points.length - 1; i++) {
				const p0 = pointToScreen(points[i]);
				const p1 = pointToScreen(points[i+1]);

				const distance = distancePointToLineSegment(
					newpoint.x, newpoint.y,
					p0.x, p0.y,
					p1.x, p1.y
				);

				if (distance <= 10) {
					after = i+1;
					break;
				}
			}
		
			if(after > 0){
				points.splice(after, 0, screenToPoint(newpoint));
			}else{
				points.push(screenToPoint(newpoint));
			}
		}
		saveSettings();
	}

	drawWaypoints();

	start_point = undefined;
	delta = undefined;
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawWaypoints();
}

window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);
window.addEventListener("tf_changed", drawWaypoints);
window.addEventListener("view_changed", drawWaypoints);

function addListeners(){
	view_container.addEventListener('mousedown', startDrag);
	view_container.addEventListener('mousemove', drag);
	view_container.addEventListener('mouseup', endDrag);

	view_container.addEventListener('touchstart', startDrag);
	view_container.addEventListener('touchmove', drag);
	view_container.addEventListener('touchend', endDrag);	
}

function removeListeners(){
	view_container.removeEventListener('mousedown', startDrag);
	view_container.removeEventListener('mousemove', drag);
	view_container.removeEventListener('mouseup', endDrag);

	view_container.removeEventListener('touchstart', startDrag);
	view_container.removeEventListener('touchmove', drag);
	view_container.removeEventListener('touchend', endDrag);	
}

function setActive(value){
	active = value;

	if(active){
		addListeners();
		icon.style.backgroundColor = "rgba(255, 255, 255, 1.0)";
		view_container.style.cursor = "pointer";
	}else{
		removeListeners()
		icon.style.backgroundColor = "rgba(124, 124, 124, 0.3)";
		view_container.style.cursor = "";
	}
}

// Topics

const selectionbox = document.getElementById("{uniqueID}_topic");
const fixedFrameBox = document.getElementById("{uniqueID}_fixed_frame");
const baseLinkFrameBox = document.getElementById("{uniqueID}_base_link_frame");

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	saveSettings();
	status.setOK();baseLinkFrameBox
});

fixedFrameBox.addEventListener("change", (event) => {
	fixed_frame = fixedFrameBox.value;
	saveSettings();
});

baseLinkFrameBox.addEventListener("change", (event) => {
	base_link_frame = baseLinkFrameBox.value;
	saveSettings();
});

async function loadTopics(){
	let result = await rosbridge.get_topics("nav_msgs/Path");

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

	//find world frames
	let framelist = "";
	for (const key of tf.frame_list.values()) {
		framelist += "<option value='"+key+"'>"+key+"</option>"
	}
	fixedFrameBox.innerHTML = framelist;

	if(tf.frame_list.has(fixed_frame)){
		fixedFrameBox.value = fixed_frame;
	}else{
		framelist += "<option value='"+fixed_frame+"'>"+fixed_frame+"</option>"
		fixedFrameBox.innerHTML = framelist;
		fixedFrameBox.value = fixed_frame;
	}

	//find base frames
	let baselist = "";
	for (const key of tf.frame_list.values()) {
		if (key.includes("base")) {
			baselist += "<option value='"+key+"'>"+key+"</option>"
		}
	}

	//if there's no base frame, it may have a nonstandard name so let's put any frame up for selection
	if(baselist == ""){
		for (const key of tf.frame_list.values()) {
			baselist += "<option value='"+key+"'>"+key+"</option>"
		}
	}

	baseLinkFrameBox.innerHTML = baselist;
	
	if(tf.frame_list.has(base_link_frame)){
		baseLinkFrameBox.value = base_link_frame;
	}else{
		baselist += "<option value='"+fixed_frame+"'>"+fixed_frame+"</option>"
		baseLinkFrameBox.innerHTML = baselist;
		baseLinkFrameBox.value = base_link_frame;
	}
}

loadTopics();

// Long press modal open stuff

let longPressTimer;
let isLongPress = false;

icon.addEventListener("click", (event) =>{
	if(!isLongPress)
		setActive(!active);
	else
		isLongPress = false;
});

icon.addEventListener("mousedown", startLongPress);
icon.addEventListener("touchstart", startLongPress);

icon.addEventListener("mouseup", cancelLongPress);
icon.addEventListener("mouseleave", cancelLongPress);
icon.addEventListener("touchend", cancelLongPress);
icon.addEventListener("touchcancel", cancelLongPress);

icon.addEventListener("contextmenu", (event) => {
	event.preventDefault();
});

function startLongPress(event) {
	isLongPress = false;
	longPressTimer = setTimeout(() => {
		isLongPress = true;
		loadTopics();
		openModal("{uniqueID}_modal");
	}, 500);
}

function cancelLongPress(event) {
	clearTimeout(longPressTimer);
}

resizeScreen();

console.log("Waypoints Widget Loaded {uniqueID}")
