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

let typedict = {};
let fixed_frame = tf.fixed_frame;
let base_link_frame = find_base_frame();
let seq = 0;
let mode = "IDLE";
let points = [];
let shift_pressed = false;

const icon_bar = document.getElementById("icon_bar");
const icon = document.getElementById("{uniqueID}_icon");
const dropdown = document.getElementById("{uniqueID}_dropdown");

const buttontext = document.getElementById("{uniqueID}_buttontext");
const margin = document.getElementById("{uniqueID}_margin");
const startCheckbox = document.getElementById('{uniqueID}_startclosest');

const flipButton = document.getElementById("{uniqueID}_flip");
const zSetButton = document.getElementById("{uniqueID}_z_set");
const deleteButton = document.getElementById("{uniqueID}_delete");

flipButton.addEventListener('click', ()=>{
	points.reverse();
	drawWaypoints();
	saveSettings();
});

zSetButton.addEventListener('click', async ()=>{
	let zval = await prompt("Set the height of all points to this value:", "0");
	if (zval != null) {
		const newz = parseFloat(zval);
		for (let i = 0; i < points.length; i++) {
			points[i].z = newz;
		}
	}
});

deleteButton.addEventListener('click', async ()=>{
	if(await confirm("Are you sure you want to delete all waypoints?")){
		points = [];
		drawWaypoints();
		saveSettings();
	}
});

startCheckbox.addEventListener('change', ()=>{
	drawWaypoints();
	saveSettings();
});

// Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	points = loaded_data.points;
	fixed_frame = loaded_data.fixed_frame ?? tf.fixed_frame;
	base_link_frame = loaded_data.base_link_frame ?? "base_link";

	margin.value = loaded_data.margin ?? 0.8;
	startCheckbox.checked = loaded_data.start_closest;

	if(loaded_data.topic_type != undefined)
		typedict[topic] = loaded_data.topic_type;

	for (let i = 0; i < points.length; i++) {
		if (points[i].z == null || points[i].z == undefined)
			points[i].z = 0;
	}

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
		topic_type: typedict[topic],
		fixed_frame: fixed_frame,
		base_link_frame: base_link_frame,
		points: points,
		start_closest: startCheckbox.checked,
		margin: margin.value
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

function getPoseStamped(index, timeStamp, x, y, z, quat){
	return new ROSLIB.Message({
		header: {
			seq: index,
			stamp: timeStamp,
			frame_id: fixed_frame
		},
		pose: {
			position: {
				x: x,
				y: y,
				z: z
			},
			orientation: quat
		}
	});
}

function getPose(x, y, z, quat){
	return new ROSLIB.Message({
		position: {
			x: x,
			y: y,
			z: z
		},
		orientation: quat
	});
}

function sendMessage(pointlist){
	let timeStamp = getStamp();
	let poseList = [];
	let stamped = typedict[topic] == "nav_msgs/Path";

	if(pointlist.length > 0)
	{
		if(pointlist.length  == 1){
			if(stamped){
				poseList.push(getPoseStamped(0, timeStamp, pointlist[0].x, pointlist[0].y, pointlist[0].z, new Quaternion()));
			}else{
				poseList.push(getPose(pointlist[0].x, pointlist[0].y, pointlist[0].z, new Quaternion()));
			}
		}else{
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

				const rotation = Quaternion.fromEuler(Math.atan2(p1.y - p0.y, p1.x - p0.x), 0, 0, 'ZXY');

				if(stamped){
					poseList.push(getPoseStamped(index, timeStamp, point.x, point.y, point.z, rotation));
				}else{
					poseList.push(getPose(point.x, point.y, point.z, rotation));
				}
			});
		}
	}

	const publisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: stamped ? 'nav_msgs/Path' : 'geometry_msgs/PoseArray',
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

	setMode("IDLE");
	closeModal("{uniqueID}_modal");
}

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

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
	return tf.transformPose(
		tf.fixed_frame, 
		fixed_frame, 
		view.screenToFixed(click), 
		new Quaternion()
	).translation;
}

function drawOffsetPath(viewPoints, offset, startIndex) {
	ctx.lineWidth = offset;
	ctx.strokeStyle = "rgba(20,20,20,0.35)";
	ctx.lineCap = "round";

	ctx.beginPath();
	for (let i = 0; i < viewPoints.length - 1; i++) {

		if(startCheckbox.checked && i < startIndex)
			continue;

        const p1 = viewPoints[i];
        const p2 = viewPoints[i + 1];

		ctx.moveTo(p1.x, p1.y);
		ctx.lineTo(p2.x, p2.y);
	}

	ctx.stroke();
}

const LIGHT_YELLOW = [255, 248, 199];
const PURE_YELLOW =  [235, 206, 0];
const DARK_YELLOW = [54, 47, 0];

const LIGHT_BLUE = [181, 209, 255];
const PURE_BLUE = [105, 162, 255];
const DARK_BLUE = [0, 25, 69];

function drawWaypoints() {

	const active = mode != "IDLE";
    const wid = canvas.width;
    const hei = canvas.height;

    ctx.clearRect(0, 0, wid, hei);

	const frame = tf.absoluteTransforms[fixed_frame];
	if(!frame){
		status.setError("Fixed transform frame not selected or the TF data is missing.");
		return;
	}

	const color = mode != "Z" ? "#EBCE00" : "#abcbff";
	const OUTLINE_PX = mode != "Z" ? 13 : 18;
	const INNER_PX = mode != "Z" ? 10 : 15;

	const startIndex = getStartIndex();
	const viewPoints = points.map((point) =>
		pointToScreen(point)
	);

	if(margin.value > 0){
		drawOffsetPath(viewPoints, margin.value * view.getMapUnitsInPixels(1.0), startIndex);
	}

	ctx.lineWidth = 3;
	ctx.fillStyle = active ? "white" : color
	if(startCheckbox.checked)
		ctx.strokeStyle = "#4a4a4a";
	else
		ctx.strokeStyle = color;

	const minZ = Math.min(...points.map(p => p.z));
	const maxZ = Math.max(...points.map(p => p.z));

	//draw path gradients
	if(minZ != maxZ)
	{
		function scale_color(x, y, z, scale){
			let r = y[0];
			let g = y[1];
			let b = y[2];
			if(scale < 0.5){
				scale = scale*2;
				const scaleinv = 1.0 - scale;
				r = scaleinv*x[0] + scale*r;
				g = scaleinv*x[1] + scale*g;
				b = scaleinv*x[2] + scale*b;
			}else if(scale > 0.5){
				scale = (scale-0.5)*2;
				const scaleinv = 1.0 - scale;
				r = scale*z[0] + scaleinv*r;
				g = scale*z[1] + scaleinv*g;
				b = scale*z[2]+ scaleinv*b;
			}
			return `rgba(${r},${g},${b},1.0)`;
		}

		for (let i = 0; i < viewPoints.length-1; i++) {
			const pos = viewPoints[i];
			const next = viewPoints[i+1];

			const grad = ctx.createLinearGradient(pos.x, pos.y, next.x, next.y)
			const start_scale = (points[i].z - minZ) / (maxZ - minZ);
			const end_scale = (points[i+1].z - minZ) / (maxZ - minZ);
			const mid_scale = (start_scale + end_scale) * 0.5;

			if(mode != "Z"){
				grad.addColorStop(0.0, scale_color(DARK_YELLOW, PURE_YELLOW, LIGHT_YELLOW, start_scale));
				grad.addColorStop(0.5, scale_color(DARK_YELLOW, PURE_YELLOW, LIGHT_YELLOW, mid_scale));
				grad.addColorStop(1.0, scale_color(DARK_YELLOW, PURE_YELLOW, LIGHT_YELLOW, end_scale));
			}else{//blue
				grad.addColorStop(0.0, scale_color(DARK_BLUE, PURE_BLUE, LIGHT_BLUE, start_scale));
				grad.addColorStop(0.5, scale_color(DARK_BLUE, PURE_BLUE, LIGHT_BLUE, mid_scale));
				grad.addColorStop(1.0, scale_color(DARK_BLUE, PURE_BLUE, LIGHT_BLUE, end_scale));
			}

			if(startCheckbox.checked && i < startIndex){
				ctx.strokeStyle = "#545454";
			}else{
				ctx.strokeStyle = grad;
			}
				
			ctx.beginPath();
			ctx.moveTo(pos.x, pos.y);
			ctx.lineTo(next.x, next.y);
			ctx.stroke();
		}
		
	}
	else //draw monocolour path
	{
		ctx.beginPath();
		for (let i = 0; i < viewPoints.length; i++) {
			const pos = viewPoints[i];
	
			if(i == startIndex && startCheckbox.checked){
				ctx.lineTo(pos.x, pos.y);
				ctx.stroke();
				ctx.strokeStyle = color; 
				ctx.beginPath();
			}
	
			if (i === 0) {
				ctx.moveTo(pos.x, pos.y);
			} else {
				ctx.lineTo(pos.x, pos.y);
			}
		};
		ctx.stroke();
	}

	function drawCircles(){
		//circle outlines
		ctx.fillStyle = "#292929";
		ctx.beginPath();
		for (let i = 0; i < viewPoints.length; i++) {
			const pos = viewPoints[i];
			ctx.moveTo(pos.x+OUTLINE_PX, pos.y);
			ctx.arc(pos.x, pos.y, OUTLINE_PX, 0, 2 * Math.PI, false);
		};
		ctx.fill();

		//circle middle
		if(startCheckbox.checked)
		{
			ctx.fillStyle = active ? "white" : "#827c52";
			ctx.beginPath();
			for (let i = 0; i < startIndex; i++) {
				const pos = viewPoints[i];
				ctx.moveTo(pos.x+INNER_PX, pos.y);
				ctx.arc(pos.x, pos.y, INNER_PX, 0, 2 * Math.PI, false);
			}
			ctx.fill();

			ctx.fillStyle = active ? "white" : color;
			ctx.beginPath();
			for (let i = startIndex; i < viewPoints.length; i++) {
				const pos = viewPoints[i];
				ctx.moveTo(pos.x+INNER_PX, pos.y);
				ctx.arc(pos.x, pos.y, INNER_PX, 0, 2 * Math.PI, false);
			}
			ctx.fill();
		}
		else
		{
			ctx.fillStyle = active ? "white" : color;
			ctx.beginPath();
			for (let i = 0; i < viewPoints.length; i++) {
				const pos = viewPoints[i];
				ctx.moveTo(pos.x+INNER_PX, pos.y);
				ctx.arc(pos.x, pos.y, INNER_PX, 0, 2 * Math.PI, false);
			}
			ctx.fill();
		}
	}

	function drawRectangles(){

		function traceRect(pos, width, height){
			const x = pos.x - width/2;
			const y = pos.y - height/2;
			ctx.moveTo(x, y);
			ctx.lineTo(x + width, y);
			ctx.lineTo(x + width, y + height);
			ctx.lineTo(x, y + height);
			ctx.lineTo(x, y);
		}

		const BORDER_PX = (OUTLINE_PX - INNER_PX) * 2;

		//rect outlines
		ctx.lineWidth = 1;
		ctx.fillStyle = "#292929";
		ctx.beginPath();
		for (let i = 0; i < viewPoints.length; i++) {
			traceRect(viewPoints[i], INNER_PX*3.5+BORDER_PX, INNER_PX*1.3+BORDER_PX);
		}
		ctx.fill();

		//rect middle
		ctx.fillStyle = active ? "white" : color;
		ctx.beginPath();
		for (let i = 0; i < viewPoints.length; i++) {
			traceRect(viewPoints[i], INNER_PX*3.5, INNER_PX*1.3);
		}
		ctx.fill();

		//draw depth scale
		if(drag_point >= 0){
			const p = viewPoints[drag_point];

			const grad = ctx.createLinearGradient(p.x-60, p.y, p.x+25, p.y)
			grad.addColorStop(0.0, "rgba(0, 0, 0, 0.75)");
			grad.addColorStop(1.0, "transparent");
			ctx.fillStyle = grad;
			ctx.fillRect(p.x-60, icon_bar.offsetHeight, 85, window.innerHeight-icon_bar.offsetHeight)

			ctx.lineWidth = 2;
			ctx.strokeStyle = "white";
			ctx.beginPath();

			//0
			ctx.moveTo(p.x-60, p.y);
			ctx.lineTo(p.x-30, p.y);

			const steps = [1, 10, 100, 1000, 10000]
			for(const i of steps){
				const scaled = stepToLinearScale(i) / 1.25;
				ctx.moveTo(p.x-60, p.y+scaled);
				ctx.lineTo(p.x, p.y+scaled);

				ctx.moveTo(p.x-60, p.y-scaled);
				ctx.lineTo(p.x, p.y-scaled);
			}
			ctx.stroke();

			ctx.lineJoin = 'round';
			ctx.miterLimit = 2;
			ctx.font = (12)+"px Monospace";
			ctx.textAlign = "left";
			ctx.fillStyle = "white";

			for(const i of steps){
				const scaled = stepToLinearScale(i) / 1.25;

				const text = Math.round(drag_point_z+i).toFixed(0);
				const text_neg = Math.round(drag_point_z-i).toFixed(0);

				ctx.fillText(text_neg, p.x-25, p.y+scaled-5);
				ctx.fillText(text, p.x-25, p.y-scaled-5);
			}

			ctx.lineWidth = 1;
			ctx.strokeStyle = "lightgray";
			ctx.beginPath();
			const micro_steps = [
				0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,
				1, 2, 3, 4, 5, 6, 7, 8, 9,
				10, 20, 30, 40, 50, 60, 70, 80, 90, 
				100, 200, 300, 400, 500, 600, 700, 800, 900,
				1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000
			]

			for(const i of micro_steps){
				const scaled = stepToLinearScale(i) / 1.25;
				ctx.moveTo(p.x-60, p.y+scaled);
				ctx.lineTo(p.x-30, p.y+scaled);

				ctx.moveTo(p.x-60, p.y-scaled);
				ctx.lineTo(p.x-30, p.y-scaled);
			}

			ctx.stroke();

			ctx.lineWidth = 5;
			ctx.strokeStyle = "#446294";
			ctx.beginPath();
			ctx.moveTo(p.x-60, icon_bar.offsetHeight);
			ctx.lineTo(p.x-60, window.innerHeight);

			//up arrow
			ctx.moveTo(p.x-65, icon_bar.offsetHeight+10);
			ctx.lineTo(p.x-60, icon_bar.offsetHeight);
			ctx.lineTo(p.x-55, icon_bar.offsetHeight+10);

			//down arrow
			ctx.moveTo(p.x-65, window.innerHeight-10);
			ctx.lineTo(p.x-60, window.innerHeight);
			ctx.lineTo(p.x-55, window.innerHeight-10);
			ctx.stroke();
		}
		
	}

	if(mode == "Z")
		drawRectangles();
	else
		drawCircles();

	ctx.font = "bold 12px Monospace";
	ctx.textAlign = "center";
	ctx.fillStyle = "#21252b";

	function formatZ(num) {
		if(num > 0){
			if (num >= 10000) return "9999";
			if (num >= 100) return Math.floor(num).toString();
			return num.toFixed(1);
		}
		const absnum = Math.abs(num);
		if (absnum >= 10000) return "-9999";
		if (absnum >= 100) return Math.floor(num).toString();
		return num.toFixed(1);
	}
	viewPoints.forEach((pos, index) => {
		if(mode == "Z")
			ctx.fillText(formatZ(points[index].z)+"m", pos.x, pos.y+5);
		else
			ctx.fillText(index, pos.x, pos.y+5);
	});

	status.setOK();
}

let start_stamp = undefined;
let start_point = undefined;
let delta = undefined;
let drag_point = -1;
let drag_point_z = 0;

function findPoint(newpoint){
	let i = -1;
	points.forEach((point, index) => {
		const screenpoint = pointToScreen(point);
		const dist = Math.hypot(
			screenpoint.x - newpoint.x,
			screenpoint.y - newpoint.y,
		)
		if(mode == "XY" && dist < 15){
			i = index;
		}else if(mode == "Z" && dist < 20){
			i = index;
		}
	});
	return i;
}

const Z_SCALE_MULT = 170;

function stepToLinearScale(x) {
    const absX = Math.abs(x);
    let result;
    if (absX <= 1) {
        result = absX;
    } else if (absX <= 10) {
        result = 1 + (absX - 1) / 9;
    } else if (absX <= 100) {
        result = 2 + (absX - 10) / 90;
    } else if (absX <= 1000) {
        result = 3 + (absX - 100) / 900;
    } else if (absX <= 10000) {
        result = 4 + (absX - 1000) / 9000;
    } else {
        result = 5;
    }
    return result * Math.sign(x) * Z_SCALE_MULT;
}

function linearToStepScale(y) {
	y/=Z_SCALE_MULT;
    const absY = Math.abs(y);
    let result;
    if (absY <= 1) {
        result = absY;
    } else if (absY <= 2) {
        result = 1 + (absY - 1) * 9;
    } else if (absY <= 3) {
        result = 10 + (absY - 2) * 90;
    } else if (absY <= 4) {
        result = 100 + (absY - 3) * 900;
	} else if (absY <= 5) {
        result = 1000 + (absY - 4) * 9000;
	}else{
		result = 10000;
	}
    return result * Math.sign(y);
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
		drag_point_z = points[drag_point].z;
	}

	start_stamp = new Date();
}

function drag(event){
	let { clientX, clientY } = event.touches ? event.touches[0] : event;

	if(shift_pressed){
		clientX = Math.round(clientX/20) * 20;
		clientY = Math.round(clientY/20) * 20;
	}

	if(mode == "XY"){
		if(drag_point >= 0){
			const newpos = screenToPoint({
				x: clientX,
				y: clientY
			})
	
			points[drag_point].x = newpos.x;
			points[drag_point].y = newpos.y;
			drawWaypoints();
		}
	} 

	if (start_point === undefined) 
		return;

	delta = {
		x: start_point.x - clientX,
		y: start_point.y - clientY,
	};

	if(mode == "Z" && drag_point >= 0){	
		points[drag_point].z = drag_point_z + linearToStepScale(delta.y * 1.25); 

		if(points[drag_point].z > 9999.99)
			points[drag_point].z = 9999;
		else if(points[drag_point].z < -9999.99)
			points[drag_point].z = -9999;

		if (Math.abs(points[drag_point].z) >= 100)
			points[drag_point].z = parseInt(points[drag_point].z);
		else
			points[drag_point].z = parseInt(points[drag_point].z*10)/10;

		drawWaypoints();
	}
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

	if(moveDist < 10 && new Date() - start_stamp  < 300 && mode == "XY"){

		start_stamp = new Date("2010-3-2"); //debounce, and also when ROS box turtle was released

		let { clientX, clientY } = event.touches ? event.touches[0] : event;

		if(shift_pressed){
			clientX = Math.round(clientX/20) * 20;
			clientY = Math.round(clientY/20) * 20;
		}

		const newpoint = {
			x: clientX,
			y: clientY
		};

		let index = findPoint(newpoint);

		if(index >= 0){ // remove point
			points.splice(index, 1);
		}else{
			let before = -1;
			for (let i = 0; i < points.length - 1; i++) {
				const p0 = pointToScreen(points[i]);
				const p1 = pointToScreen(points[i+1]);

				const distance = distancePointToLineSegment(
					newpoint.x, newpoint.y,
					p0.x, p0.y,
					p1.x, p1.y
				);

				if (distance <= 10) {
					before = i+1;
					break;
				}
			}
		
			if(before > 0){
				// insert new point between two others
				const p = screenToPoint(newpoint);
				const p0 = points[before-1];
				const p1 = points[before];

				// Calculate the weight as the ratio of distances
				const distP0P1 = Math.hypot(p1.x - p0.x, p1.y - p0.y);
				const distP0P = Math.hypot(p.x - p0.x, p.y - p0.y);
				p.z = p0.z + distP0P / distP0P1 * (p1.z - p0.z);
				points.splice(before, 0, p);
			}else{
				// add point to the end
				const p = screenToPoint(newpoint);

				if (points.length > 0)
					p.z = points[points.length-1].z;

				points.push(p);
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
window.addEventListener("view_changed", drawWaypoints);

window.addEventListener("tf_fixed_frame_changed", drawWaypoints);
window.addEventListener("tf_changed", ()=>{
	if(fixed_frame != tf.fixed_frame){
		drawWaypoints();
	}
});

view_container.addEventListener("mouseleave", (event) => {
	delta = undefined;
	endDrag(event);
});

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

function setMode(newmode){
	mode = newmode;

	switch(mode){
		case "IDLE":
			removeListeners()
			icon.style.backgroundColor = "rgba(124, 124, 124, 0.3)";
			view_container.style.cursor = "";
			buttontext.innerText = "";
			canvas.style.zIndex = "2";
			break;

		case "XY":
			addListeners();
			icon.style.backgroundColor = "rgba(255, 255, 255, 1.0)";
			view_container.style.cursor = "pointer";
			buttontext.innerText = "X,Y";
			canvas.style.zIndex = "999";
			break;

		case "Z":
			addListeners();
			icon.style.backgroundColor = "rgba(255, 255, 255, 1.0)";
			view_container.style.cursor = "pointer";
			buttontext.innerText = "Z";
			canvas.style.zIndex = "999";
			break;
	}

	drawWaypoints();
}

// Shift clamp to axis
function handleKeyDown(event) {
	if (event.key === "Shift") {
		shift_pressed = true;
	}
}

function handleKeyUp(event) {
	if (event.key === "Shift") {
		shift_pressed = false;
	}
}

window.addEventListener("keydown", handleKeyDown);
window.addEventListener("keyup", handleKeyUp);

// Topics
const selectionbox = document.getElementById("{uniqueID}_topic");
const fixedFrameBox = document.getElementById("{uniqueID}_fixed_frame");
const baseLinkFrameBox = document.getElementById("{uniqueID}_base_link_frame");

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	saveSettings();
	status.setOK();
});

fixedFrameBox.addEventListener("change", (event) => {
	fixed_frame = fixedFrameBox.value;
	saveSettings();
});

baseLinkFrameBox.addEventListener("change", (event) => {
	base_link_frame = baseLinkFrameBox.value;
	saveSettings();
});

margin.addEventListener("input", (event) =>{
	drawWaypoints();
	saveSettings();
});

function find_base_frame(){
	//try base_link first
	for (const key of tf.frame_list.values()) {
		if (key.includes("base_link")) {
			return key
		}
	}

	//maybe footprint?
	for (const key of tf.frame_list.values()) {
		if (key.includes("base_footprint")) {
			return key
		}
	}

	//ok just base then...?
	for (const key of tf.frame_list.values()) {
		if (key.includes("base")) {
			return key
		}
	}

	//eh screw it
	return "base_link";
}

async function loadTopics(){
	const result_path = await rosbridge.get_topics("nav_msgs/Path");
	const result_array = await rosbridge.get_topics("geometry_msgs/PoseArray");

	let topiclist = "";
	result_path.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+" (Path)</option>";
		typedict[element] = "nav_msgs/Path";
	});
	result_array.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+" (PoseArray)</option>";
		typedict[element] = "geometry_msgs/PoseArray";
	});
	selectionbox.innerHTML = topiclist

	if(topic == "")
		topic = selectionbox.value;
	else{
		if(result_path.includes(topic) || result_array.includes(topic)){
			selectionbox.value = topic;
		}else{
			topiclist += "<option value='"+topic+"'>"+topic+"</option>"
			selectionbox.innerHTML = topiclist
			selectionbox.value = topic;
		}
	}

	//find frames
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

	baseLinkFrameBox.innerHTML = framelist;
	
	if(tf.frame_list.has(base_link_frame)){
		baseLinkFrameBox.value = base_link_frame;
	}else{
		framelist += "<option value='"+fixed_frame+"'>"+fixed_frame+"</option>"
		baseLinkFrameBox.innerHTML = framelist;
		baseLinkFrameBox.value = base_link_frame;
	}
}

loadTopics();

//dropdown stuff

function dropdown_visibility(open){
	if(open)
		dropdown.style.display = "block";
	else
		dropdown.style.display = "none";
}

// Toggle dropdown on click
icon.addEventListener("click", (event) => {
	event.stopPropagation();

	if(mode != "IDLE"){
		setMode("IDLE");
	}else{
		const rect = icon.getBoundingClientRect();
		const dropdownWidth = 90;
		let top = rect.bottom + 5; // Default: below the icon
		let left = rect.left;

		if (left + dropdownWidth > window.innerWidth) {
			left = window.innerWidth - dropdownWidth - 5;
		}

		if (left < 5) {
			left = 5;
		}

		dropdown.style.top = `${top}px`;
		dropdown.style.left = `${left}px`;
	
		dropdown_visibility(dropdown.style.display == "none")
	}
});

// Close dropdown when clicking outside
document.addEventListener("click", (event) => {
	if (!dropdown.contains(event.target) && !icon.contains(event.target)) {
		dropdown_visibility(false);
	}
});

const drop_start = document.getElementById("{uniqueID}_sendAction");
const drop_stop = document.getElementById("{uniqueID}_stopAction");
const drop_xy = document.getElementById("{uniqueID}_editXY");
const drop_z = document.getElementById("{uniqueID}_editZ");
const drop_config = document.getElementById("{uniqueID}_config");

const startButton = document.getElementById("{uniqueID}_start");
const stopButton = document.getElementById("{uniqueID}_stop");

drop_start.addEventListener("click", (event) => {
	if(startCheckbox.checked)
		sendMessage(points.slice(getStartIndex()))
	else
		sendMessage(points)
	dropdown_visibility(false);
});

drop_stop.addEventListener("click", (event) => {
	sendMessage([]);
	dropdown_visibility(false);
});

drop_xy.addEventListener("click", (event) => {
	setMode("XY");
	dropdown_visibility(false);
});

drop_z.addEventListener("click", (event) => {
	setMode("Z");
	dropdown_visibility(false);
});

drop_config.addEventListener("click", (event) => {
	loadTopics();
	openModal("{uniqueID}_modal");
	dropdown_visibility(false);
});

resizeScreen();

console.log("Waypoints Widget Loaded {uniqueID}")