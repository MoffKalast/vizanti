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
let path_publisher = undefined;
let mode = "IDLE";
let shift_pressed = false;

let polygon = [];
let start_marker = null;
let end_marker = null;
let survey_points = [];
let transect_labels = [];

let last_vertex_count = 0;
let survey_error = null;

let survey_seq = 0;
let survey_pending = null;
let survey_busy = false;

const INITIAL_MAX_LINES = 30;
const INITIAL_MIN_LINES = 5;
const INITIAL_TARGET_LINES = 10;

const worker_thread = new Worker(`${base_url}/templates/survey/survey_worker.js`);

const icon_bar = document.getElementById("icon_bar");
const icon = document.getElementById("{uniqueID}_icon");
const dropdown = document.getElementById("{uniqueID}_dropdown");
const buttontext = document.getElementById("{uniqueID}_buttontext");

const spacingBox = document.getElementById("{uniqueID}_spacing");
const angleBox = document.getElementById("{uniqueID}_angle");
const turnaroundBox = document.getElementById("{uniqueID}_turnaround");
const crosshatchCheckbox = document.getElementById("{uniqueID}_crosshatch");
const directTransitCheckbox = document.getElementById("{uniqueID}_directtransit");

const zSetButton = document.getElementById("{uniqueID}_z_set");
const deleteButton = document.getElementById("{uniqueID}_delete");

zSetButton.addEventListener('click', async ()=>{
	let zval = await prompt("Set the height of all polygon vertices to this value:", "0");
	if (zval != null) {
		const newz = parseFloat(zval);
		for (let i = 0; i < polygon.length; i++) {
			polygon[i].z = newz;
		}
		if(start_marker) start_marker.z = newz;
		if(end_marker) end_marker.z = newz;
		update();
	}
});

deleteButton.addEventListener('click', async ()=>{
	if(await confirm("Are you sure you want to delete the survey polygon?")){
		polygon = [];
		start_marker = null;
		end_marker = null;
		update();
	}
});

// Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data = settings["{uniqueID}"];
	topic = loaded_data.topic;
	polygon = loaded_data.polygon ?? [];
	start_marker = loaded_data.start_marker ?? null;
	end_marker = loaded_data.end_marker ?? null;
	fixed_frame = loaded_data.fixed_frame ?? tf.fixed_frame;
	base_link_frame = loaded_data.base_link_frame ?? "base_link";

	spacingBox.value = loaded_data.spacing ?? 1.0;
	angleBox.value = loaded_data.angle ?? 0;
	turnaroundBox.value = loaded_data.turnaround ?? 1.0;
	crosshatchCheckbox.checked = loaded_data.crosshatch ?? false;
	directTransitCheckbox.checked = loaded_data.direct_transit ?? false;

	if(loaded_data.topic_type != undefined)
		typedict[topic] = loaded_data.topic_type;

	for (let i = 0; i < polygon.length; i++) {
		if (polygon[i].z == null || polygon[i].z == undefined){
			polygon[i].z = 0;
		}
	}

	last_vertex_count = polygon.length;
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
		polygon: polygon,
		start_marker: start_marker,
		end_marker: end_marker,
		spacing: spacingBox.value,
		angle: angleBox.value,
		turnaround: turnaroundBox.value,
		crosshatch: crosshatchCheckbox.checked,
		direct_transit: directTransitCheckbox.checked
	}
	settings.save();
}

// Survey geometry

function getCCWPolygon(poly){
	let area = 0;
	for (let i = 0; i < poly.length; i++) {
		const a = poly[i];
		const b = poly[(i+1) % poly.length];
		area += a.x * b.y - b.x * a.y;
	}

	if(area < 0)
		return poly.slice().reverse();
	return poly.slice();
}

function offsetPolygon(poly, dist){
	if(dist <= 0){
		return poly.map(p => ({
			x: p.x,
			y: p.y,
			z: p.z
		}));
	}

	// outward miter offset, assumes CCW winding so outward normal of edge (a->b) is (dy, -dx)
	const count = poly.length;
	const result = [];

	for(let i = 0; i < count; i++){
		const prev = poly[(i - 1 + count) % count];
		const cur = poly[i];
		const next = poly[(i + 1) % count];

		let dir0X = cur.x - prev.x;
		let dir0Y = cur.y - prev.y;
		let dir1X = next.x - cur.x;
		let dir1Y = next.y - cur.y;

		const len0 = Math.hypot(dir0X, dir0Y) || 1;
		const len1 = Math.hypot(dir1X, dir1Y) || 1;

		dir0X /= len0;
		dir0Y /= len0;
		dir1X /= len1;
		dir1Y /= len1;

		const norm0 = {
			x: dir0Y,
			y: -dir0X
		};

		const norm1 = {
			x: dir1Y,
			y: -dir1X
		};

		let miterX = norm0.x + norm1.x;
		let miterY = norm0.y + norm1.y;
		const miterLen = Math.hypot(miterX, miterY);

		if(miterLen < 1e-9){
			result.push({
				x: cur.x + norm0.x * dist,
				y: cur.y + norm0.y * dist,
				z: cur.z
			});
			continue;
		}

		miterX /= miterLen;
		miterY /= miterLen;

		const dot = norm0.x * norm1.x + norm0.y * norm1.y;
		let scale = dist / Math.sqrt((1 + dot) * 0.5);

		if(scale > dist * 3)
			scale = dist * 3;

		result.push({
			x: cur.x + miterX * scale,
			y: cur.y + miterY * scale,
			z: cur.z
		});
	}

	return result;
}

function roundSpacing(value){
	if(value >= 100)
		return Math.round(value);

	if(value >= 10)
		return Math.round(value * 10) / 10;

	return Math.round(value * 100) / 100;
}

function transectExtent(poly, angleRad){
	const nrm = {x: -Math.sin(angleRad), y: Math.cos(angleRad)};

	let min = Infinity, max = -Infinity;
	for (const p of poly) {
		const c = p.x * nrm.x + p.y * nrm.y;
		if(c < min) min = c;
		if(c > max) max = c;
	}

	const extent = max - min;
	return extent > 0 ? extent : 0;
}

function countTransects(poly, angleRad, spacing){
	const extent = transectExtent(poly, angleRad);
	if(extent == 0 || !(spacing > 0))
		return 0;

	const lines = Math.ceil((extent - spacing * 0.5) / spacing);
	return lines > 0 ? lines : 0;
}

function countAllTransects(poly, angleRad, spacing){
	if(!crosshatchCheckbox.checked)
		return countTransects(poly, angleRad, spacing);
	return countTransects(poly, angleRad, spacing) + countTransects(poly, angleRad + Math.PI/2, spacing);
}


function requestSurvey(){

	if(polygon.length < 3 || !start_marker || !end_marker){
		survey_points = [];
		transect_labels = [];
		survey_error = null;
		survey_pending = null;
		return;
	}

	survey_pending = {
		polygon: polygon,
		start_marker: start_marker,
		end_marker: end_marker,
		spacing: Math.max(parseFloat(spacingBox.value) || 1.0, 0.05),
		angle: (parseInt(angleBox.value) || 0) * Math.PI / 180.0,
		turnaround: Math.max(parseFloat(turnaroundBox.value) || 0, 0),
		crosshatch: crosshatchCheckbox.checked,
		direct_transit: directTransitCheckbox.checked
	};

	dispatchSurvey();
}

function dispatchSurvey(){

	if(survey_busy || !survey_pending)
		return;

	const job = survey_pending;
	survey_pending = null;
	survey_busy = true;
	job.seq = ++survey_seq;

	worker_thread.postMessage(job);
}

worker_thread.onmessage = (e) => {

	survey_busy = false;

	if(e.data.seq == survey_seq){
		survey_points = e.data.survey_points;
		transect_labels = e.data.transect_labels;
		survey_error = e.data.error;
	}

	dispatchSurvey();
	drawSurvey();
};

function autosizeSpacing(){
	const poly = getCCWPolygon(polygon);
	const angle = (parseInt(angleBox.value) || 0) * Math.PI / 180.0;
	const spacing = Math.max(parseFloat(spacingBox.value) || 1.0, 0.05);

	const lines = countAllTransects(poly, angle, spacing);
	if(lines >= INITIAL_MIN_LINES && lines <= INITIAL_MAX_LINES)
		return;

	const extent = transectExtent(poly, angle);
	if(extent == 0)
		return;

	const newspacing = Math.max(0.05, roundSpacing(extent / INITIAL_TARGET_LINES));
	if(newspacing == spacing)
		return;

	spacingBox.value = newspacing;
	turnaroundBox.value = newspacing;
}

function update(){

	const was_polygon = last_vertex_count >= 3;
	last_vertex_count = polygon.length;

	if(polygon.length >= 3 && !was_polygon)
		autosizeSpacing();

	if(polygon.length >= 3 && (!start_marker || !end_marker)){
		let link = {
			translation: {
				x: 0, 
				y: 0,
				z: 0
			}
		};

		if(base_link_frame != ""){
			link = tf.transformPose(base_link_frame, fixed_frame, {x: 0, y: 0, z: 0}, new Quaternion());
		}

		const marker_offset = Math.max(parseFloat(spacingBox.value) || 2, 0.05);

		if(!start_marker){
			start_marker = {
				x: link.translation.x - marker_offset,
				y: link.translation.y,
				z: link.translation.z
			};
		}
		if(!end_marker){
			end_marker = {
				x: link.translation.x + marker_offset,
				y: link.translation.y, 
				z: link.translation.z
			};
		}
	}

	requestSurvey();
	drawSurvey();
	saveSettings();
}

// Message sending

function sendMessage(pointlist){

	function getStamp(){
		const currentTime = new Date();
		return {
			secs: Math.floor(currentTime.getTime() / 1000),
			nsecs: (currentTime.getTime() % 1000) * 1e6
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

	let timeStamp = getStamp();
	let poseList = [];
	let stamped = typedict[topic] == "nav_msgs/Path";

	if(pointlist.length > 0)
	{
		if(pointlist.length == 1){
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
	
	if(path_publisher !== undefined){
		path_publisher.unadvertise();
	}

	path_publisher = new ROSLIB.Topic({
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

	path_publisher.publish(pathMessage);
	status.setOK();

	setMode("IDLE");
	closeModal("{uniqueID}_modal");
}

// Rendering

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });
const view_container = document.getElementById("view_container");

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

const POLY_COLOR = "#69A2FF";
const POLY_EDGE_COLOR = "rgba(90, 130, 200, 0.95)";
const SURVEY_COLOR = "#6FA8DC";
const SURVEY_DARK = "#2c4a6e";
const TRANSIT_COLOR = "rgba(200, 200, 200, 0.75)";
const START_COLOR = "#3ecf5e";
const END_COLOR = "#e0483e";

function polygonCentroid(pts){
	let area = 0, cx = 0, cy = 0;

	for (let i = 0; i < pts.length; i++) {
		const a = pts[i];
		const b = pts[(i+1) % pts.length];
		const cross = a.x * b.y - b.x * a.y;
		area += cross;
		cx += (a.x + b.x) * cross;
		cy += (a.y + b.y) * cross;
	}

	if(Math.abs(area) < 1e-9){
		let sx = 0, sy = 0;
		for (const p of pts) {
			sx += p.x;
			sy += p.y;
		}
		return {x: sx / pts.length, y: sy / pts.length};
	}

	return {x: cx / (3 * area), y: cy / (3 * area)};
}

function drawHourglass(pos){

	ctx.fillStyle = "rgba(41, 41, 41, 0.85)";
	ctx.beginPath();
	ctx.arc(pos.x, pos.y, 15, 0, 2 * Math.PI, false);
	ctx.fill();

	ctx.fillStyle = "#e8e8e8";
	ctx.fillRect(pos.x - 6, pos.y - 8, 12, 1.5);
	ctx.fillRect(pos.x - 6, pos.y + 6.5, 12, 1.5);

	ctx.beginPath();
	ctx.moveTo(pos.x - 5, pos.y - 6.5);
	ctx.lineTo(pos.x + 5, pos.y - 6.5);
	ctx.lineTo(pos.x, pos.y);
	ctx.closePath();
	ctx.fill();

	ctx.beginPath();
	ctx.moveTo(pos.x - 5, pos.y + 6.5);
	ctx.lineTo(pos.x + 5, pos.y + 6.5);
	ctx.lineTo(pos.x, pos.y);
	ctx.closePath();
	ctx.fill();
}

function drawSurvey(){
	const active = mode != "IDLE";
	const wid = canvas.width;
	const hei = canvas.height;

	ctx.clearRect(0, 0, wid, hei);

	const frame = tf.absoluteTransforms[fixed_frame];
	if(!frame){
		status.setError("Fixed transform frame not selected or the TF data is missing.");
		return;
	}

	const OUTLINE_PX = mode != "Z" ? 13 : 18;
	const INNER_PX = mode != "Z" ? 10 : 15;

	const viewPoly = polygon.map(pointToScreen);

	// stacked translucent fills: one layer for the turnaround area, another on the polygon itself
	const turnaround = Math.max(parseFloat(turnaroundBox.value) || 0, 0);
	if(polygon.length >= 3){
		ctx.fillStyle = "rgba(255, 255, 255, 0.1)";

		if(turnaround > 0){
			const outer = offsetPolygon(getCCWPolygon(polygon), turnaround).map(pointToScreen);
			ctx.beginPath();
			outer.forEach((p, i) => i == 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y));
			ctx.closePath();
			ctx.fill();
		}

		ctx.beginPath();
		viewPoly.forEach((p, i) => i == 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y));
		ctx.closePath();
		ctx.fill();
	}

	// polygon edges, dark blue dashed
	if(viewPoly.length >= 2){
		ctx.strokeStyle = POLY_EDGE_COLOR;
		ctx.lineWidth = 3;
		ctx.setLineDash([10, 8]);
		ctx.beginPath();
		viewPoly.forEach((p, i) => i == 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y));
		if(viewPoly.length >= 3)
			ctx.closePath();
		ctx.stroke();
	}
	ctx.setLineDash([]);

	if(survey_points.length >= 2){
		const viewSurvey = survey_points.map(pointToScreen);

		const pathColor = active ? "rgba(255,255,255,0.9)" : SURVEY_COLOR;
		ctx.lineWidth = 2.5;
		ctx.lineJoin = "round";
		for (let i = 0; i < viewSurvey.length - 1; i++) {
			ctx.strokeStyle = survey_points[i+1].transit ? TRANSIT_COLOR : pathColor;
			ctx.beginPath();
			ctx.moveTo(viewSurvey[i].x, viewSurvey[i].y);
			ctx.lineTo(viewSurvey[i+1].x, viewSurvey[i+1].y);
			ctx.stroke();
		}

		// direction ticks halfway along each segment
		for (let i = 0; i < viewSurvey.length - 1; i++) {
			ctx.fillStyle = survey_points[i+1].transit ? TRANSIT_COLOR : (active ? "white" : SURVEY_COLOR);
			const a = viewSurvey[i];
			const b = viewSurvey[i+1];
			const len = Math.hypot(b.x - a.x, b.y - a.y);
			if(len < 50)
				continue;

			const t = 0.7;
			const mx = a.x + (b.x - a.x) * t;
			const my = a.y + (b.y - a.y) * t;

			const ux = (b.x - a.x) / len, uy = (b.y - a.y) / len;
			ctx.beginPath();
			ctx.moveTo(mx + ux * 6, my + uy * 6);
			ctx.lineTo(mx - ux * 4 - uy * 5, my - uy * 4 + ux * 5);
			ctx.lineTo(mx - ux * 4 + uy * 5, my - uy * 4 - ux * 5);
			ctx.closePath();
			ctx.fill();
		}

		// waypoint dots
		ctx.fillStyle = SURVEY_DARK;
		ctx.beginPath();
		for (const p of viewSurvey) {
			ctx.moveTo(p.x + 4.5, p.y);
			ctx.arc(p.x, p.y, 4.5, 0, 2 * Math.PI, false);
		}
		ctx.fill();
		ctx.fillStyle = active ? "white" : SURVEY_COLOR;
		ctx.beginPath();
		for (const p of viewSurvey) {
			ctx.moveTo(p.x + 3, p.y);
			ctx.arc(p.x, p.y, 3, 0, 2 * Math.PI, false);
		}
		ctx.fill();
	}

	function drawNode(pos, fill, label, sizeScale = 1.0){
		if(mode == "Z"){
			const w = INNER_PX * 3.5, h = INNER_PX * 1.3;
			const bw = (OUTLINE_PX - INNER_PX) * 2;
			ctx.fillStyle = "#292929";
			ctx.fillRect(pos.x - (w+bw)/2, pos.y - (h+bw)/2, w+bw, h+bw);
			ctx.fillStyle = fill;
			ctx.fillRect(pos.x - w/2, pos.y - h/2, w, h);
		}else{
			ctx.fillStyle = "#292929";
			ctx.beginPath();
			ctx.arc(pos.x, pos.y, OUTLINE_PX * sizeScale, 0, 2 * Math.PI, false);
			ctx.fill();
			ctx.fillStyle = fill;
			ctx.beginPath();
			ctx.arc(pos.x, pos.y, INNER_PX * sizeScale, 0, 2 * Math.PI, false);
			ctx.fill();
		}

		ctx.font = "bold 12px Monospace";
		ctx.textAlign = "center";
		ctx.fillStyle = "#21252b";
		ctx.fillText(label, pos.x, pos.y + 5);
	}

	function formatZ(num){
		if (Math.abs(num) >= 100) return Math.floor(num).toString();
		return num.toFixed(1);
	}

	// transect execution order labels
	if(mode != "Z"){
		ctx.font = "bold 11px Monospace";
		ctx.textAlign = "center";
		transect_labels.forEach((label, index) => {
			const pos = pointToScreen(label);
			ctx.fillStyle = SURVEY_DARK;
			ctx.beginPath();
			ctx.arc(pos.x, pos.y, 9, 0, 2 * Math.PI, false);
			ctx.fill();
			ctx.fillStyle = active ? "white" : SURVEY_COLOR;
			ctx.fillText(index + 1, pos.x, pos.y + 4);
		});
	}

	const nodeColor = active ? "white" : POLY_COLOR;
	viewPoly.forEach((pos, index) => {
		drawNode(pos, nodeColor, mode == "Z" ? formatZ(polygon[index].z)+"m" : "", 0.6);
	});

	if(polygon.length >= 3 && start_marker && end_marker){
		const sm = pointToScreen(start_marker);
		const em = pointToScreen(end_marker);
		drawNode(sm, active ? "#c8ffd4" : START_COLOR, mode == "Z" ? formatZ(start_marker.z)+"m" : "", 0.7);
		drawNode(em, active ? "#ffd2ce" : END_COLOR, mode == "Z" ? formatZ(end_marker.z)+"m" : "", 0.7);
	}

	if((survey_busy || survey_pending) && polygon.length >= 3){
		drawHourglass(polygonCentroid(viewPoly));
		status.setWarn("Generating path...");
	}else if(survey_error){
		status.setError(survey_error);
	}else{
		status.setOK();
	}
}

// Input handling

let start_stamp = undefined;
let start_point = undefined;
let delta = undefined;
let drag_target = null;
let drag_point_z = 0;

function getEditablePoints(){
	const list = polygon.map((p, i) => ({point: p, type: "poly", index: i}));
	if(polygon.length >= 3 && start_marker && end_marker){
		list.push({point: start_marker, type: "start", index: -1});
		list.push({point: end_marker, type: "end", index: -1});
	}
	return list;
}

function findTarget(screenpos){
	let found = null;
	const radius = mode == "Z" ? 20 : 15;
	for (const entry of getEditablePoints()) {
		const sp = pointToScreen(entry.point);
		if(Math.hypot(sp.x - screenpos.x, sp.y - screenpos.y) < radius)
			found = entry;
	}
	return found;
}

const Z_SCALE_MULT = 170;

function linearToStepScale(y){
	y /= Z_SCALE_MULT;
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
	start_point = {x: clientX, y: clientY};

	drag_target = findTarget(start_point);
	if(drag_target){
		view.setInputMovementEnabled(false);
		drag_point_z = drag_target.point.z;
	}

	start_stamp = new Date();
}

function drag(event){
	let { clientX, clientY } = event.touches ? event.touches[0] : event;

	if(shift_pressed){
		clientX = Math.round(clientX/20) * 20;
		clientY = Math.round(clientY/20) * 20;
	}

	if(mode == "XY" && drag_target){
		const newpos = screenToPoint({x: clientX, y: clientY});
		drag_target.point.x = newpos.x;
		drag_target.point.y = newpos.y;
		requestSurvey();
		drawSurvey();
	}

	if (start_point === undefined)
		return;

	delta = {
		x: start_point.x - clientX,
		y: start_point.y - clientY,
	};

	if(mode == "Z" && drag_target){
		let z = drag_point_z + linearToStepScale(delta.y * 1.25);

		if(z > 9999.99)
			z = 9999;
		else if(z < -9999.99)
			z = -9999;

		if (Math.abs(z) >= 100)
			z = parseInt(z);
		else
			z = parseInt(z*10)/10;

		drag_target.point.z = z;
		requestSurvey();
		drawSurvey();
	}
}

function endDrag(event){

	if(drag_target){
		view.setInputMovementEnabled(true);
		drag_target = null;
		saveSettings();
	}

	let moveDist = 0;
	if(delta !== undefined){
		moveDist = Math.hypot(delta.x, delta.y);
	}

	if(moveDist < 10 && new Date() - start_stamp < 300 && mode == "XY"){

		start_stamp = new Date("2010-3-2"); //debounce

		const touch = event.changedTouches?.[0] ?? event.touches?.[0] ?? event;
		let { clientX, clientY } = touch;

		if(shift_pressed){
			clientX = Math.round(clientX/20) * 20;
			clientY = Math.round(clientY/20) * 20;
		}

		const newpoint = {x: clientX, y: clientY};
		const target = findTarget(newpoint);

		if(target){
			// start/end markers can be dragged but not deleted
			if(target.type == "poly")
				polygon.splice(target.index, 1);
		}else{
			const p = screenToPoint(newpoint);

			if(polygon.length >= 3){
				// insert at the edge where the detour is smallest, so far away clicks
				// extend the nearest side instead of appending and self-intersecting
				let before = 1, bestCost = Infinity;
				for (let i = 0; i < polygon.length; i++) {
					const a = polygon[i];
					const b = polygon[(i+1) % polygon.length];
					const cost = Math.hypot(p.x - a.x, p.y - a.y) + Math.hypot(b.x - p.x, b.y - p.y) - Math.hypot(b.x - a.x, b.y - a.y);
					if(cost < bestCost){
						bestCost = cost;
						before = i + 1;
					}
				}

				const p0 = polygon[before-1];
				const p1 = polygon[before % polygon.length];
				const distP0P1 = Math.hypot(p1.x - p0.x, p1.y - p0.y);
				const distP0P = Math.hypot(p.x - p0.x, p.y - p0.y);
				p.z = distP0P1 > 0 ? p0.z + Math.min(distP0P / distP0P1, 1) * (p1.z - p0.z) : p0.z;
				polygon.splice(before, 0, p);
			}else{
				if (polygon.length > 0)
					p.z = polygon[polygon.length-1].z;
				polygon.push(p);
			}
		}
		update();
	}else{
		drawSurvey();
	}

	start_point = undefined;
	delta = undefined;
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawSurvey();
}

window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);
window.addEventListener("view_changed", drawSurvey);

window.addEventListener("tf_fixed_frame_changed", drawSurvey);
window.addEventListener("tf_changed", ()=>{
	if(fixed_frame != tf.fixed_frame){
		drawSurvey();
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
			buttontext.innerText = "X,Y ";
			canvas.style.zIndex = "999";
			break;

		case "Z":
			addListeners();
			icon.style.backgroundColor = "rgba(255, 255, 255, 1.0)";
			view_container.style.cursor = "pointer";
			buttontext.innerText = "Z ";
			canvas.style.zIndex = "999";
			break;
	}

	drawSurvey();
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
	update();
});

baseLinkFrameBox.addEventListener("change", (event) => {
	base_link_frame = baseLinkFrameBox.value;
	saveSettings();
});

for (const el of [spacingBox, angleBox, turnaroundBox]) {
	el.addEventListener("input", update);
}
crosshatchCheckbox.addEventListener("change", update);
directTransitCheckbox.addEventListener("change", update);

function find_base_frame(){
	for (const key of tf.frame_list.values()) {
		if (key.includes("base_link")) {
			return key
		}
	}
	for (const key of tf.frame_list.values()) {
		if (key.includes("base_footprint")) {
			return key
		}
	}
	for (const key of tf.frame_list.values()) {
		if (key.includes("base")) {
			return key
		}
	}
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
		framelist += "<option value='"+base_link_frame+"'>"+base_link_frame+"</option>"
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

icon.addEventListener("click", (event) => {
	event.stopPropagation();

	if(mode != "IDLE"){
		setMode("IDLE");
	}else{
		const rect = icon.getBoundingClientRect();
		const dropdownWidth = 90;
		let top = rect.bottom + 5;
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

drop_start.addEventListener("click", (event) => {
	if(survey_busy || survey_pending){
		status.setWarn("Path is still generating, try again in a moment.");
	}else if(survey_points.length == 0){
		status.setWarn("No survey path to send, define a polygon with at least 3 vertices.");
	}else{
		sendMessage(survey_points);
	}
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

requestSurvey();
resizeScreen();

console.log("Survey Widget Loaded {uniqueID}")