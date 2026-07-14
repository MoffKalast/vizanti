let viewModule = await import(`${base_url}/js/modules/view.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);

let view = viewModule.view;
let settings = persistentModule.settings;
let Status = StatusModule.Status;
let tf = tfModule.tf;
let applyRotation = tfModule.applyRotation;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const click_icon = document.getElementById("{uniqueID}_icon");
const icon = click_icon.getElementsByTagName('object')[0];
const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const MAX_LINES = 300;
// |ex × ey| = cos(tilt) for a pure tilt; below this the plane is essentially edge-on
const MIN_PLANARITY = 0.02;
const FIXED_FRAME_LABEL = "&lt;fixed_frame&gt;";

let grid_size = 1.0;
let grid_thickness = 1;
let grid_colour = "#3e556a";
let grid_colour_sub = "#294056";
let grid_autoscale = 'Coarse';
let grid_subdivisions = 2;
let grid_frame = "";

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
const colourpicker_sub = document.getElementById("{uniqueID}_colorpicker_sub");
const autoscale = document.getElementById("{uniqueID}_autoscale");
const linethickness = document.getElementById("{uniqueID}_thickness");
const subdivisions = document.getElementById("{uniqueID}_subdivisions");
const gridstep = document.getElementById("{uniqueID}_step");
const frameselector = document.getElementById("{uniqueID}_frame");

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data = settings["{uniqueID}"];
	grid_size = loaded_data.size;
	grid_thickness = loaded_data.thickness;
	grid_colour = loaded_data.colour;
	grid_colour_sub = loaded_data.colour_sub ?? "#294056";
	grid_autoscale = loaded_data.autoscale ?? 'Off';
	grid_subdivisions = loaded_data.subdivisions ?? 1;
	grid_frame = loaded_data.frame ?? "";
}else{
	saveSettings();
}

linethickness.value = grid_thickness;
colourpicker.value = grid_colour;
autoscale.value = grid_autoscale;
colourpicker_sub.value = grid_colour_sub;
gridstep.value = grid_size;
subdivisions.value = grid_subdivisions;

icon.onload = () => {
	utilModule.setIconColor(icon, colourpicker.value);
};
if (icon.contentDocument) {
	utilModule.setIconColor(icon, colourpicker.value);
}

function saveSettings(){
	settings["{uniqueID}"] = {
		size: grid_size,
		thickness: grid_thickness,
		colour: grid_colour,
		colour_sub: grid_colour_sub,
		autoscale: grid_autoscale,
		subdivisions: grid_subdivisions,
		frame: grid_frame
	}
	settings.save();
}

// Snaps a length to the nearest 1/2/5 * 10^n step
function calculateScale(value) {
	const magnitude = Math.floor(Math.log10(value));
	value /= Math.pow(10, magnitude);

	if (value < 1.5) {
		value = 1.0;
	} else if (value < 3.5) {
		value = 2.0;
	} else if (value < 7.5) {
		value = 5.0;
	} else {
		value = 10.0;
	}

	value *= Math.pow(10, magnitude);
	return value;
}

function drawScreenLine(start_x, start_y, end_x, end_y, color, line_width) {
	ctx.beginPath();
	ctx.strokeStyle = color;
	ctx.lineWidth = line_width;
	ctx.moveTo(parseInt(start_x), parseInt(start_y));
	ctx.lineTo(parseInt(end_x), parseInt(end_y));
	ctx.stroke();
}

// Returns the grid plane's origin and X/Y basis vectors projected onto the fixed frame XY plane,
// i.e. the orthographic image of the frame's z=0 plane. Identity for the fixed frame itself.
function getGridBasis() {
	if (grid_frame === "" || grid_frame === tf.fixed_frame)
		return { origin: {x: 0, y: 0}, ex: {x: 1, y: 0}, ey: {x: 0, y: 1} };

	const abs = tf.absoluteTransforms[grid_frame];
	if (!abs)
		return null;

	const ex = applyRotation({x: 1, y: 0, z: 0}, abs.rotation, false);
	const ey = applyRotation({x: 0, y: 1, z: 0}, abs.rotation, false);
	return {
		origin: {x: abs.translation.x, y: abs.translation.y},
		ex: {x: ex.x, y: ex.y},
		ey: {x: ey.x, y: ey.y}
	};
}

// Composes the grid basis with the view into a single canvas affine matrix,
// mapping grid-local (u,v) meters directly to screen pixels.
// Y axes are negated because screen Y points down while fixed Y points up.
function getScreenMatrix(basis) {
	const s = view.scale;
	const o = view.fixedToScreen(basis.origin);
	return { a: basis.ex.x * s, b: -basis.ex.y * s, c: basis.ey.x * s, d: -basis.ey.y * s, e: o.x, f: o.y };
}

// Maps the screen rect corners through the inverse matrix and returns the
// axis-aligned bounding box of the visible region in grid-local coordinates.
// Only used for draw ranges; density decisions use rotation-invariant pixel spacing instead.
function getLocalBounds(m, wid, hei) {
	const det = m.a * m.d - m.b * m.c;
	let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;

	for (const [sx, sy] of [[0, 0], [wid, 0], [0, hei], [wid, hei]]) {
		const dx = sx - m.e;
		const dy = sy - m.f;
		const u = (m.d * dx - m.c * dy) / det;
		const v = (m.a * dy - m.b * dx) / det;
		umin = Math.min(umin, u);
		umax = Math.max(umax, u);
		vmin = Math.min(vmin, v);
		vmax = Math.max(vmax, v);
	}

	return { umin, umax, vmin, vmax };
}

function drawGridLines(m, bounds, step, subdivisions) {
	const u0 = Math.floor(bounds.umin / step) * step;
	const v0 = Math.floor(bounds.vmin / step) * step;

	// Paths are built under the affine transform (points get mapped as they're added),
	// then stroked under identity so lineWidth stays in screen pixels.
	if (subdivisions > 1) {
		const sub = step / subdivisions;
		ctx.setTransform(m.a, m.b, m.c, m.d, m.e, m.f);
		ctx.beginPath();
		for (let u = u0; u <= bounds.umax; u += step) {
			for (let i = 1; i < subdivisions; i++) {
				const x = u + i * sub;
				ctx.moveTo(x, bounds.vmin);
				ctx.lineTo(x, bounds.vmax);
			}
		}
		for (let v = v0; v <= bounds.vmax; v += step) {
			for (let i = 1; i < subdivisions; i++) {
				const y = v + i * sub;
				ctx.moveTo(bounds.umin, y);
				ctx.lineTo(bounds.umax, y);
			}
		}
		ctx.setTransform(1, 0, 0, 1, 0, 0);
		ctx.globalAlpha = 0.65;
		ctx.strokeStyle = grid_colour_sub;
		ctx.lineWidth = 1;
		ctx.stroke();
	}

	ctx.setTransform(m.a, m.b, m.c, m.d, m.e, m.f);
	ctx.beginPath();
	for (let u = u0; u <= bounds.umax; u += step) {
		ctx.moveTo(u, bounds.vmin);
		ctx.lineTo(u, bounds.vmax);
	}
	for (let v = v0; v <= bounds.vmax; v += step) {
		ctx.moveTo(bounds.umin, v);
		ctx.lineTo(bounds.umax, v);
	}
	ctx.setTransform(1, 0, 0, 1, 0, 0);
	ctx.globalAlpha = 1.0;
	ctx.strokeStyle = grid_colour;
	ctx.lineWidth = grid_thickness;
	ctx.stroke();
}

function drawGridScale(m, wid, hei) {
	const xoffset = wid < hei ? 40 : 100;
	const yoffset = 40;

	// pixel length of one grid step along the grid's local x axis
	const line_length = grid_size * Math.hypot(m.a, m.b);
	const xscale_start = parseInt(wid - xoffset - line_length);

	drawScreenLine(xscale_start, hei - yoffset, wid - xoffset, hei - yoffset, grid_colour, 2);
	drawScreenLine(xscale_start, hei - yoffset - 5, xscale_start, hei - yoffset + 5, grid_colour, 2);
	drawScreenLine(wid - xoffset, hei - yoffset - 5, wid - xoffset, hei - yoffset + 5, grid_colour, 2);

	let scale_text = String(grid_size) + ' m';
	if(grid_size >= 1000)
		scale_text = String(grid_size / 1000) + ' km';
	else if(grid_size < 1)
		scale_text = String(grid_size * 100) + ' cm';

	ctx.font = "16px Monospace";
	ctx.textAlign = "center";
	ctx.fillStyle = grid_colour;
	ctx.fillText(scale_text, parseInt(xscale_start + line_length / 2), parseInt(hei - 23));
}

async function drawGrid() {
	const wid = canvas.width;
	const hei = canvas.height;

	ctx.setTransform(1, 0, 0, 1, 0, 0);
	ctx.clearRect(0, 0, wid, hei);

	const basis = getGridBasis();
	if (!basis) {
		status.setWarn("Transform to frame '" + grid_frame + "' not yet available.");
		return;
	}

	// determinant of the projected basis: shrinks with tilt, hits 0 when the plane is edge-on
	const planarity = basis.ex.x * basis.ey.y - basis.ex.y * basis.ey.x;
	if (Math.abs(planarity) < MIN_PLANARITY) {
		status.setWarn("Grid plane is edge-on to the view, not rendering.");
		return;
	}

	const m = getScreenMatrix(basis);

	// On-screen pixel spacing of 1m along each grid axis. Invariant under both view and frame
	// yaw rotation (unlike the local-space AABB extents), so autoscale and the line cap can't
	// flicker between steps while rotating. Tilt foreshortening still shrinks it as intended.
	const px_per_meter = Math.min(Math.hypot(m.a, m.b), Math.hypot(m.c, m.d));
	const visible_meters = Math.min(wid, hei) / px_per_meter;

	if(grid_autoscale === 'Very Fine')
		grid_size = calculateScale(visible_meters / 21);
	else if(grid_autoscale === 'Fine')
		grid_size = calculateScale(visible_meters / 14);
	else if(grid_autoscale === 'Coarse')
		grid_size = calculateScale(visible_meters / 7);
	else if(grid_autoscale === 'Rough')
		grid_size = calculateScale(visible_meters / 3);

	// diagonal / spacing is a rotation-invariant upper bound on how many lines the AABB draw
	// ranges can produce, so staying under it here guarantees the draw loops stay bounded too
	const diag = Math.hypot(wid, hei);
	let temp_subdivisions = grid_subdivisions;
	const lineCount = () => diag / ((grid_size / Math.max(1, temp_subdivisions)) * px_per_meter);

	while (lineCount() > MAX_LINES && temp_subdivisions > 1)
		temp_subdivisions--;

	if (lineCount() > MAX_LINES) {
		if (grid_autoscale === 'Off') {
			status.setWarn("Too many lines to render, increase step size");
			return;
		}
		while (lineCount() > MAX_LINES)
			grid_size = calculateScale(grid_size * 2.1);
	}

	const bounds = getLocalBounds(m, wid, hei);
	drawGridLines(m, bounds, grid_size, temp_subdivisions);

	if(grid_autoscale != 'Off'){
		drawGridScale(m, wid, hei);
	}

	status.setOK();
}

function updateFrameList() {
	const current = grid_frame;
	let html = "<option value=''>" + FIXED_FRAME_LABEL + "</option>";
	for (const frame of tf.frame_list) {
		html += "<option value='" + frame + "'>" + frame + "</option>";
	}
	frameselector.innerHTML = html;
	frameselector.value = current;
	if (frameselector.value !== current && current !== "") {
		// keep a stale selection visible until its TF shows up again
		frameselector.innerHTML += "<option value='" + current + "'>" + current + "</option>";
		frameselector.value = current;
	}
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawGrid();
}

window.addEventListener("view_changed", drawGrid);
window.addEventListener("tf_fixed_frame_changed", drawGrid);
window.addEventListener("tf_changed", () => {
	if (grid_frame !== "" && grid_frame !== tf.fixed_frame)
		drawGrid();
});
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

click_icon.addEventListener("click", (event) => {
	updateFrameList();
});

frameselector.addEventListener("input", (event) => {
	grid_frame = frameselector.value;
	drawGrid();
	saveSettings();
});

linethickness.addEventListener("input", (event) =>{
	if(linethickness.value > 20)
		grid_thickness = 20;
	else if(linethickness.value < 1)
		grid_thickness = 1;
	else if(isNaN(linethickness.value))
		grid_thickness = 1;
	else
		grid_thickness = parseFloat(linethickness.value);

	drawGrid();
	saveSettings();
});

gridstep.addEventListener("input", (event) =>{
	if(gridstep.value > 1000000)
		grid_size = 1000000;
	else if(gridstep.value < 0.01)
		grid_size = 0.01;
	else if(isNaN(gridstep.value))
		grid_size = 1.0;
	else
		grid_size = parseFloat(gridstep.value);

	drawGrid();
	saveSettings();
});

colourpicker.addEventListener("input", (event) =>{
	grid_colour = colourpicker.value;
	utilModule.setIconColor(icon, grid_colour);
	drawGrid();
	saveSettings();
});

colourpicker_sub.addEventListener("input", (event) =>{
	grid_colour_sub = colourpicker_sub.value;
	drawGrid();
	saveSettings();
});

autoscale.addEventListener("input", (event) =>{
	grid_autoscale = autoscale.value;
	if(gridstep.value > 1000000)
		grid_size = 1000000;
	else if(gridstep.value < 0.01)
		grid_size = 0.01;
	else if(isNaN(gridstep.value))
		grid_size = 1.0;
	else
		grid_size = parseFloat(gridstep.value);

	drawGrid();
	saveSettings();
});

subdivisions.addEventListener("input", (event) =>{
	grid_subdivisions = parseInt(subdivisions.value);
	drawGrid();
	saveSettings();
});

updateFrameList();
resizeScreen();

console.log("Grid Widget Loaded {uniqueID}")
