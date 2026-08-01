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
let shift_pressed = false;

let polygon = [];
let start_marker = null;
let end_marker = null;
let survey_points = [];
let transect_labels = [];

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
		if (polygon[i].z == null || polygon[i].z == undefined)
			polygon[i].z = 0;
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

// ---- Survey geometry ----

function polygonSignedArea(poly){
	let area = 0;
	for (let i = 0; i < poly.length; i++) {
		const a = poly[i];
		const b = poly[(i+1) % poly.length];
		area += a.x * b.y - b.x * a.y;
	}
	return area * 0.5;
}

function getCCWPolygon(poly){
	if(polygonSignedArea(poly) < 0)
		return poly.slice().reverse();
	return poly.slice();
}

function offsetPolygon(poly, dist){
	if(dist <= 0)
		return poly.map(p => ({x: p.x, y: p.y, z: p.z}));

	// outward miter offset, assumes CCW winding so outward normal of edge (a->b) is (dy, -dx)
	const n = poly.length;
	const result = [];
	for (let i = 0; i < n; i++) {
		const prev = poly[(i-1+n) % n];
		const cur = poly[i];
		const next = poly[(i+1) % n];

		let d0x = cur.x - prev.x, d0y = cur.y - prev.y;
		let d1x = next.x - cur.x, d1y = next.y - cur.y;
		const l0 = Math.hypot(d0x, d0y) || 1;
		const l1 = Math.hypot(d1x, d1y) || 1;
		d0x /= l0; d0y /= l0;
		d1x /= l1; d1y /= l1;

		const n0 = {x: d0y, y: -d0x};
		const n1 = {x: d1y, y: -d1x};

		// miter direction as normalized sum of adjacent edge normals, scaled by 1/(1+dot) to hit the true offset corner
		let mx = n0.x + n1.x, my = n0.y + n1.y;
		const ml = Math.hypot(mx, my);
		if(ml < 1e-9){
			result.push({x: cur.x + n0.x * dist, y: cur.y + n0.y * dist, z: cur.z});
			continue;
		}
		mx /= ml; my /= ml;
		const dot = n0.x * n1.x + n0.y * n1.y;
		let scale = dist / Math.sqrt((1 + dot) * 0.5);
		if(scale > dist * 3)
			scale = dist * 3;
		result.push({x: cur.x + mx * scale, y: cur.y + my * scale, z: cur.z});
	}
	return result;
}

function lineIntersections(poly, p0, dir){
	// intersections of infinite line (p0 + t*dir) with polygon edges, z lerped along the edge
	const hits = [];
	for (let i = 0; i < poly.length; i++) {
		const a = poly[i];
		const b = poly[(i+1) % poly.length];
		const ex = b.x - a.x, ey = b.y - a.y;
		const denom = dir.x * ey - dir.y * ex;
		if(Math.abs(denom) < 1e-12)
			continue;
		const s = (dir.y * (a.x - p0.x) - dir.x * (a.y - p0.y)) / denom;
		if(s < 0 || s >= 1)
			continue;
		const t = Math.abs(dir.x) > Math.abs(dir.y) ? (a.x + s * ex - p0.x) / dir.x : (a.y + s * ey - p0.y) / dir.y;
		hits.push({t: t, x: a.x + s * ex, y: a.y + s * ey, z: a.z + s * (b.z - a.z)});
	}
	hits.sort((u, v) => u.t - v.t);
	return hits;
}

function closestOnPolygon(poly, p){
	let best = null;
	for (let i = 0; i < poly.length; i++) {
		const a = poly[i];
		const b = poly[(i+1) % poly.length];
		const dx = b.x - a.x, dy = b.y - a.y;
		const lsq = dx * dx + dy * dy;
		let t = lsq > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / lsq : 0;
		t = Math.max(0, Math.min(1, t));
		const cx = a.x + t * dx, cy = a.y + t * dy;
		const dist = Math.hypot(p.x - cx, p.y - cy);
		if(best == null || dist < best.dist){
			best = {edge: i, t: t, x: cx, y: cy, z: a.z + t * (b.z - a.z), dist: dist};
		}
	}
	return best;
}

function tracePerimeter(poly, from, to){
	// intermediate vertices between two boundary locations, along the shorter perimeter direction
	const n = poly.length;
	const edgeLen = [];
	let perimeter = 0;
	for (let i = 0; i < n; i++) {
		const a = poly[i];
		const b = poly[(i+1) % n];
		edgeLen.push(Math.hypot(b.x - a.x, b.y - a.y));
		perimeter += edgeLen[i];
	}
	if(perimeter < 1e-9)
		return [];

	function arclength(loc){
		let s = 0;
		for (let i = 0; i < loc.edge; i++)
			s += edgeLen[i];
		return s + edgeLen[loc.edge] * loc.t;
	}

	const sFrom = arclength(from);
	const sTo = arclength(to);
	const forward = (sTo - sFrom + perimeter) % perimeter;
	const backward = perimeter - forward;

	const verts = [];
	if(forward <= backward){
		let i = (from.edge + 1) % n;
		while(true){
			const sv = arclength({edge: i, t: 0});
			if(((sv - sFrom + perimeter) % perimeter) >= forward)
				break;
			verts.push({x: poly[i].x, y: poly[i].y, z: poly[i].z});
			i = (i + 1) % n;
			if(verts.length > n) break;
		}
	}else{
		let i = from.edge;
		while(true){
			const sv = arclength({edge: i, t: 0});
			if(((sFrom - sv + perimeter) % perimeter) >= backward)
				break;
			verts.push({x: poly[i].x, y: poly[i].y, z: poly[i].z});
			i = (i - 1 + n) % n;
			if(verts.length > n) break;
		}
	}
	return verts;
}

function pointInPolygon(poly, p){
	let inside = false;
	for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
		const a = poly[i], b = poly[j];
		if(((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x))
			inside = !inside;
	}
	return inside;
}

function generateTransects(poly, angleRad, spacing, turnaround){
	const dir = {x: Math.cos(angleRad), y: Math.sin(angleRad)};
	const nrm = {x: -dir.y, y: dir.x};

	let min = Infinity, max = -Infinity;
	for (const p of poly) {
		const c = p.x * nrm.x + p.y * nrm.y;
		if(c < min) min = c;
		if(c > max) max = c;
	}

	const lines = [];
	for (let c = min + spacing * 0.5; c < max; c += spacing) {
		const p0 = {x: nrm.x * c, y: nrm.y * c};
		let hits = lineIntersections(poly, p0, dir);

		// drop duplicate hits from lines passing exactly through a vertex shared by two edges
		hits = hits.filter((h, i) => i == 0 || h.t - hits[i-1].t > 1e-9);

		// keep only spans between consecutive hits whose midpoint lies inside the polygon,
		// so concave shapes get one transect per interior span instead of garbage pairing
		const segments = [];
		for (let i = 0; i + 1 < hits.length; i++) {
			const a = hits[i];
			const b = hits[i+1];
			if(!pointInPolygon(poly, {x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5}))
				continue;
			segments.push({
				a: {x: a.x - dir.x * turnaround, y: a.y - dir.y * turnaround, z: a.z},
				b: {x: b.x + dir.x * turnaround, y: b.y + dir.y * turnaround, z: b.z}
			});
		}
		if(segments.length > 0)
			lines.push(segments);
	}
	return lines;
}

function orderTransects(lines, entry){
	if(lines.length == 0)
		return [];

	function lineDist(line, p){
		let d = Infinity;
		for (const s of line)
			d = Math.min(d, Math.hypot(s.a.x - p.x, s.a.y - p.y), Math.hypot(s.b.x - p.x, s.b.y - p.y));
		return d;
	}

	if(lineDist(lines[lines.length-1], entry) < lineDist(lines[0], entry))
		lines = lines.slice().reverse();

	const segs = [];
	let cur = entry;
	for (const line of lines) {
		const remaining = line.slice();
		while(remaining.length > 0){
			let bestIdx = 0, bestFlip = false, bestDist = Infinity;
			for (let i = 0; i < remaining.length; i++) {
				const dA = Math.hypot(remaining[i].a.x - cur.x, remaining[i].a.y - cur.y);
				const dB = Math.hypot(remaining[i].b.x - cur.x, remaining[i].b.y - cur.y);
				if(dA < bestDist){ bestDist = dA; bestIdx = i; bestFlip = false; }
				if(dB < bestDist){ bestDist = dB; bestIdx = i; bestFlip = true; }
			}
			const seg = remaining.splice(bestIdx, 1)[0];
			const p1 = bestFlip ? seg.b : seg.a;
			const p2 = bestFlip ? seg.a : seg.b;
			segs.push({a: {x: p1.x, y: p1.y, z: p1.z}, b: {x: p2.x, y: p2.y, z: p2.z}});
			cur = p2;
		}
	}
	return segs;
}

function connectorCrossesUncharted(outer, p1, p2, tolerance){
	// sample the straight connector; it crosses uncharted area if any sample
	// lies outside the turnaround boundary by more than the tolerance
	for (let i = 1; i < 10; i++) {
		const t = i / 10;
		const s = {x: p1.x + (p2.x - p1.x) * t, y: p1.y + (p2.y - p1.y) * t};
		if(!pointInPolygon(outer, s) && closestOnPolygon(outer, s).dist > tolerance)
			return true;
	}
	return false;
}

function appendTransects(path, segs, outer, tolerance){
	for (const seg of segs) {
		const cur = path[path.length-1];
		if(cur && connectorCrossesUncharted(outer, cur, seg.a, tolerance)){
			if(directTransitCheckbox.checked){
				pushUnique(path, {x: seg.a.x, y: seg.a.y, z: seg.a.z, transit: true});
			}else{
				// route along the turnaround boundary instead of crossing uncharted area
				const from = closestOnPolygon(outer, cur);
				const to = closestOnPolygon(outer, seg.a);
				pushUnique(path, from);
				for (const v of tracePerimeter(outer, from, to))
					pushUnique(path, v);
				pushUnique(path, to);
				pushUnique(path, seg.a);
			}
		}else{
			pushUnique(path, seg.a);
		}
		pushUnique(path, seg.b);
		transect_labels.push({x: (seg.a.x + seg.b.x) * 0.5, y: (seg.a.y + seg.b.y) * 0.5});
	}
}

function pushUnique(list, p){
	const last = list[list.length-1];
	if(last && Math.hypot(last.x - p.x, last.y - p.y) < 1e-6)
		return;
	list.push({x: p.x, y: p.y, z: p.z, transit: p.transit === true});
}

function generateSurvey(){
	survey_points = [];
	transect_labels = [];

	if(polygon.length < 3 || !start_marker || !end_marker)
		return;

	const spacing = Math.max(parseFloat(spacingBox.value) || 1.0, 0.05);
	const angle = (parseInt(angleBox.value) || 0) * Math.PI / 180.0;
	const turnaround = Math.max(parseFloat(turnaroundBox.value) || 0, 0);

	const poly = getCCWPolygon(polygon);
	const outer = offsetPolygon(poly, turnaround);
	const tolerance = Math.max(0.01, turnaround * 0.05);

	const pass = orderTransects(generateTransects(poly, angle, spacing, turnaround), start_marker);
	if(pass.length == 0)
		return;

	const path = [];
	pushUnique(path, start_marker);

	// approach the outer boundary at the closest point, then trace edges to the survey entry
	const approach = closestOnPolygon(outer, start_marker);
	const entry = closestOnPolygon(outer, pass[0].a);
	pushUnique(path, approach);
	for (const v of tracePerimeter(outer, approach, entry))
		pushUnique(path, v);
	pushUnique(path, entry);

	appendTransects(path, pass, outer, tolerance);

	if(crosshatchCheckbox.checked){
		const cross = orderTransects(generateTransects(poly, angle + Math.PI/2, spacing, turnaround), path[path.length-1]);
		appendTransects(path, cross, outer, tolerance);
	}

	// trace edges from the survey exit to the point closest to the end marker, then connect
	const exit = closestOnPolygon(outer, path[path.length-1]);
	const depart = closestOnPolygon(outer, end_marker);
	pushUnique(path, exit);
	for (const v of tracePerimeter(outer, exit, depart))
		pushUnique(path, v);
	pushUnique(path, depart);
	pushUnique(path, end_marker);

	survey_points = path;
}

function ensureMarkers(){
	if(polygon.length < 3 || (start_marker && end_marker))
		return;

	let link = {translation: {x: 0, y: 0, z: 0}};
	if(base_link_frame != ""){
		link = tf.transformPose(base_link_frame, fixed_frame, {x: 0, y: 0, z: 0}, new Quaternion());
	}

	if(!start_marker)
		start_marker = {x: link.translation.x - 2, y: link.translation.y, z: link.translation.z};
	if(!end_marker)
		end_marker = {x: link.translation.x + 2, y: link.translation.y, z: link.translation.z};
}

function update(){
	ensureMarkers();
	generateSurvey();
	drawSurvey();
	saveSettings();
}

// Message sending

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
			position: {x: x, y: y, z: z},
			orientation: quat
		}
	});
}

function getPose(x, y, z, quat){
	return new ROSLIB.Message({
		position: {x: x, y: y, z: z},
		orientation: quat
	});
}

function sendMessage(pointlist){
	let timeStamp = getStamp();
	let poseList = [];
	let stamped = typedict[topic] == "nav_msgs/msg/Path";

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

	const publisher = new ROSLIB.Topic({
		ros: rosbridge.ros,
		name: topic,
		messageType: stamped ? 'nav_msgs/msg/Path' : 'geometry_msgs/msg/PoseArray',
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

	// survey path
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
			if(len < 30)
				continue;
			const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
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

	status.setOK();
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
		generateSurvey();
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

		if(z > 9999.99) z = 9999;
		else if(z < -9999.99) z = -9999;

		if (Math.abs(z) >= 100)
			z = parseInt(z);
		else
			z = parseInt(z*10)/10;

		drag_target.point.z = z;
		generateSurvey();
		drawSurvey();
	}
}

function distancePointToLineSegment(px, py, x1, y1, x2, y2){
	const dx = x2 - x1;
	const dy = y2 - y1;
	const lengthSquared = dx * dx + dy * dy;

	let t = lengthSquared > 0 ? ((px - x1) * dx + (py - y1) * dy) / lengthSquared : 0;
	t = Math.max(0, Math.min(1, t));

	const closestX = x1 + t * dx;
	const closestY = y1 + t * dy;

	return Math.hypot(px - closestX, py - closestY);
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
			// insert on a polygon edge if clicked near one, otherwise append
			let before = -1;
			const edges = polygon.length >= 3 ? polygon.length : polygon.length - 1;
			for (let i = 0; i < edges; i++) {
				const p0 = pointToScreen(polygon[i]);
				const p1 = pointToScreen(polygon[(i+1) % polygon.length]);
				if (distancePointToLineSegment(newpoint.x, newpoint.y, p0.x, p0.y, p1.x, p1.y) <= 10) {
					before = i + 1;
					break;
				}
			}

			const p = screenToPoint(newpoint);
			if(before > 0){
				const p0 = polygon[before-1];
				const p1 = polygon[before % polygon.length];
				const distP0P1 = Math.hypot(p1.x - p0.x, p1.y - p0.y);
				const distP0P = Math.hypot(p.x - p0.x, p.y - p0.y);
				p.z = distP0P1 > 0 ? p0.z + distP0P / distP0P1 * (p1.z - p0.z) : p0.z;
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
	generateSurvey();
	if(survey_points.length == 0){
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

generateSurvey();
resizeScreen();

console.log("Survey Widget Loaded {uniqueID}")
