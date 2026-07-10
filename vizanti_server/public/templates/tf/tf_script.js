let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);

let view = viewModule.view;
let tf = tfModule.tf;
let applyRotation = tfModule.applyRotation;
let settings = persistentModule.settings;
let Status = StatusModule.Status;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);
status.setWarn("No TF data received yet.");

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const namesCheckbox = document.getElementById('{uniqueID}_shownames');
const axesCheckbox = document.getElementById('{uniqueID}_showaxes');
const linesCheckbox = document.getElementById('{uniqueID}_showlines');
const scaleSlider = document.getElementById('{uniqueID}_scale');
const scaleSliderValue = document.getElementById('{uniqueID}_scale_value');

scaleSlider.addEventListener('input', function () {
	scaleSliderValue.textContent = this.value;
});

namesCheckbox.addEventListener('change', saveSettings);
axesCheckbox.addEventListener('change', saveSettings);
linesCheckbox.addEventListener('change', saveSettings);
scaleSlider.addEventListener('change', saveSettings);

let prev_transforms = new Set();
let grouped_frames = [];
let frame_visibility = {};

// Settings

if (settings.hasOwnProperty('{uniqueID}')) {
	const loadedData = settings['{uniqueID}'];

	namesCheckbox.checked = loadedData.show_names;
	axesCheckbox.checked = loadedData.show_axes;
	linesCheckbox.checked = loadedData.show_lines;

	scaleSlider.value = loadedData.scale;
	scaleSliderValue.textContent = scaleSlider.value;

	frame_visibility = loadedData.frame_visibility;
}else{
	saveSettings();
}

function saveSettings() {
	settings['{uniqueID}'] = {
		show_names: namesCheckbox.checked,
		show_axes: axesCheckbox.checked,
		show_lines: linesCheckbox.checked,
		scale: parseFloat(scaleSlider.value),
		frame_visibility: frame_visibility
	};
	settings.save();
}

// Rendering

async function drawLines(relative, absolute){

	ctx.strokeStyle = "#eba834";
	ctx.lineWidth = 1*parseFloat(scaleSlider.value);
	ctx.beginPath();

	Object.keys(absolute).forEach(key => {

		let transform = absolute[key];

		let point = view.fixedToScreen({
			x: transform.translation.x,
			y: transform.translation.y,
		});

		if(relative[key])
		{
			let parent = absolute[relative[key].parent];
			if(parent !== undefined){
	
				let parentpoint = view.fixedToScreen({
					x: parent.translation.x,
					y: parent.translation.y,
				});
	
				ctx.moveTo(parseInt(point.x), parseInt(point.y));
				ctx.lineTo(parseInt(parentpoint.x), parseInt(parentpoint.y));
			}
		}	
	});

	ctx.stroke();

}

async function drawText(absolute){

	ctx.lineJoin = 'round';
	ctx.miterLimit = 2;
	ctx.font = (12*parseFloat(scaleSlider.value))+"px Monospace";
	ctx.textAlign = "center";
	ctx.fillStyle = "white";

	ctx.strokeStyle = "#161B21";
	ctx.lineWidth = 5*parseFloat(scaleSlider.value);

	Object.keys(absolute).forEach(key => {

		let transform = absolute[key];

		let point = view.fixedToScreen({
			x: transform.translation.x,
			y: transform.translation.y,
		});

		ctx.strokeText(key, point.x, point.y+15);
		ctx.fillText(key, point.x, point.y+15);	
	});
}

function getBasis(rotation, translation, unit) {
	const w = rotation.w;
	const x = rotation.x;
	const y = rotation.y;
	const z = rotation.z;

	const xx = x * x;
	const yy = y * y;
	const zz = z * z;
	const xy = x * y;
	const xz = x * z;
	const yz = y * z;
	const wx = w * x;
	const wy = w * y;
	const wz = w * z;

	// Compute rotation matrix columns inline
	const v1x = 1 - 2 * (yy + zz);
	const v1y = 2 * (xy + wz);
	const v1z = 2 * (xz - wy);
	const v2x = 2 * (xy - wz);
	const v2y = 1 - 2 * (xx + zz);
	const v2z = 2 * (yz + wx);

	// Cross product
	const v3x = v1y * v2z - v1z * v2y;
	const v3y = v1z * v2x - v1x * v2z;
	const v3z = v1x * v2y - v1y * v2x;

	const tx = translation.x;
	const ty = translation.y;

	// Return flat array instead of nested objects
	const origin = view.fixedToScreen({x: tx, y: ty});
	const vx = view.fixedToScreen({x: tx + v1x * unit, y: ty + v1y * unit});
	const vy = view.fixedToScreen({x: tx + v2x * unit, y: ty + v2y * unit});
	const vz = view.fixedToScreen({x: tx + v3x * unit, y: ty + v3y * unit});

	// Store as flat structure for better cache locality
	return [origin, vx, vy, vz, v3z < 0];
}

function drawAxes(absolute) {
	const unit = view.getPixelsInMapUnits(30 * parseFloat(scaleSlider.value));
	const lineWidth = 2 * parseFloat(scaleSlider.value);
	
	const frame_keys = Object.keys(absolute);
	const vectors = new Array(frame_keys.length);
	
	for (let i = 0; i < frame_keys.length; i++) {
		const transform = absolute[frame_keys[i]];
		vectors[i] = getBasis(transform.rotation, transform.translation, unit);
	}

	ctx.lineWidth = lineWidth;

	// Red X
	ctx.strokeStyle = "#E0000B";
	ctx.beginPath();
	for (let i = 0; i < vectors.length; i++) {
		const p = vectors[i];
		ctx.moveTo(p[0].x | 0, p[0].y | 0); // Bitwise OR for fast int conversion
		ctx.lineTo(p[1].x | 0, p[1].y | 0);
	}
	ctx.stroke();

	// Green Y
	ctx.strokeStyle = "#29FF26";
	ctx.beginPath();
	for (let i = 0; i < vectors.length; i++) {
		const p = vectors[i];
		ctx.moveTo(p[0].x | 0, p[0].y | 0);
		ctx.lineTo(p[2].x | 0, p[2].y | 0);
	}
	ctx.stroke();

	// Blue +Z
	ctx.strokeStyle = "#0084ff";
	ctx.beginPath();
	for (let i = 0; i < vectors.length; i++) {
		const p = vectors[i];
		if (!p[4]) {
			ctx.moveTo(p[0].x | 0, p[0].y | 0);
			ctx.lineTo(p[3].x | 0, p[3].y | 0);
		}
	}
	ctx.stroke();

	// Dark blue -Z
	ctx.strokeStyle = "#0003c0";
	ctx.beginPath();
	for (let i = 0; i < vectors.length; i++) {
		const p = vectors[i];
		if (p[4]) {
			ctx.moveTo(p[0].x | 0, p[0].y | 0);
			ctx.lineTo(p[3].x | 0, p[3].y | 0);
		}
	}
	ctx.stroke();
}

const framesDiv = document.getElementById('{uniqueID}_frames');

let visibleRelativeKeys = new Set();
let visibleAbsoluteKeys = new Set();

async function drawFrames() {
	ctx.clearRect(0, 0, canvas.width, canvas.height);

	const relative = Object.fromEntries(
		[...visibleRelativeKeys].map(k => [k, tf.transforms[k]])
	);
	const absolute = Object.fromEntries(
		[...visibleAbsoluteKeys].map(k => [k, tf.absoluteTransforms[k]])
	);

	if(linesCheckbox.checked)
		drawLines(relative, absolute);

	if(axesCheckbox.checked)
		drawAxes(absolute);

	if(namesCheckbox.checked)
		drawText(absolute);
}

function updateVisibility(){

	//check for new frames and precompute filters once a second

	visibleRelativeKeys.clear();
	visibleAbsoluteKeys.clear();

	Object.keys(tf.transforms).forEach(key => {
		const child = key;
		const parent = tf.transforms[key].parent;

		if(!frame_visibility.hasOwnProperty(child)){
			frame_visibility[child] = true;
		}

		if(!frame_visibility.hasOwnProperty(parent)){
			frame_visibility[parent] = true;
		}

		if (frame_visibility[child]) {
			visibleRelativeKeys.add(child);
			if(tf.absoluteTransforms.hasOwnProperty(child)){
				visibleAbsoluteKeys.add(child);
			}
		}
		if (frame_visibility[parent]) {
			visibleRelativeKeys.add(parent);
			if(tf.absoluteTransforms.hasOwnProperty(parent)){
				visibleAbsoluteKeys.add(parent);
			}
		}
	});	
}

function updateGUI(){

	updateVisibility();

	const eqSet = (xs, ys) => xs.size === ys.size && [...xs].every((x) => ys.has(x));

	let current_transforms = new Set();
	Object.keys(tf.transforms).forEach(child => {
		current_transforms.add(child);
		current_transforms.add(tf.transforms[child].parent);
	});

	if (!eqSet(prev_transforms, current_transforms)){
		grouped_frames = utilModule.groupStringsByPrefix(Array.from(current_transforms), 2);
		prev_transforms = current_transforms;
	}


	function getEntry(key) {
		const checkbox = document.createElement('input');
		checkbox.type = 'checkbox';
		checkbox.id = `{uniqueID}_${key}`;
		checkbox.checked = frame_visibility[key];
		checkbox.addEventListener('change', (event) => {
			frame_visibility[key] = event.target.checked;
			saveSettings();
		});

		const frame = tf.transforms[key];
		const label = document.createElement('label');

		function addSpan(text, color) {
			const span = document.createElement('span');
			if (color) span.style.color = color;
			span.textContent = text;
			label.appendChild(span);
		}
		
		addSpan(` ${key} `, 'white');

		if(frame != undefined){
			addSpan(`← ${frame.parent} `, 'darkgray');
			addSpan('(', 'darkgray');
			addSpan(frame.translation.x.toFixed(2), '#ff6666');  // soft red
			addSpan(' ', null);
			addSpan(frame.translation.y.toFixed(2), '#66cc66');  // soft green
			addSpan(' ', null);
			addSpan(frame.translation.z.toFixed(2), '#9aacc3ff');  // light blue
			addSpan(')', 'darkgray');
		}else{
			addSpan('(origin)', 'darkgray');
		}


		const div = document.createElement('div');
		div.classList.add('tf_label');
		div.appendChild(checkbox);
		div.appendChild(label);
		return div;
	}

	framesDiv.innerHTML = '';
	for(let i = 0; i < grouped_frames.length; i++){
		const elementlist = grouped_frames[i];

		if(elementlist.length == 1){
			framesDiv.appendChild(getEntry(elementlist[0]));
		}else{
			const detailsElement = document.createElement("details");
			detailsElement.setAttribute("open", "open");
			detailsElement.classList.add("tf_details");

			const summaryElement = document.createElement("summary");
			summaryElement.classList.add("tf_summary");
			summaryElement.textContent = elementlist[0].replace(/_+$/, '');

			detailsElement.appendChild(summaryElement);
			detailsElement.appendChild(document.createElement('br'));
			for(let j = 1; j < elementlist.length; j++){
				detailsElement.appendChild(getEntry(elementlist[j]));
				detailsElement.appendChild(document.createElement('br'));
			}		  
			framesDiv.appendChild(detailsElement);
		}

		framesDiv.appendChild(document.createElement('br'));
	}
}

document.getElementById('{uniqueID}_enable_all').addEventListener('click',  async () => {
	for (const [key, value] of Object.entries(frame_visibility)) {
		frame_visibility[key] = true;
	}
	saveSettings();
	updateGUI();
});

document.getElementById('{uniqueID}_standard_only').addEventListener('click',  async () => {
	const standard_frames = ["world", "earth", "map", "odom", "base_link", "base_footprint", "laser", "base_stabilized"];
	const hasStandardFrame = (str) => standard_frames.some(frame => str.includes(frame));

	for (const [key, value] of Object.entries(frame_visibility)) {
		frame_visibility[key] = hasStandardFrame(key);
	}
	saveSettings();
	updateGUI();

});

document.getElementById('{uniqueID}_disable_all').addEventListener('click',  async () => {
	for (const [key, value] of Object.entries(frame_visibility)) {
		frame_visibility[key] = false;
	}
	saveSettings();
	updateGUI();
});

icon.addEventListener("click", updateGUI);

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawFrames();
}

window.addEventListener("tf_changed", ()=>{
	status.setOK();
	drawFrames();
});

window.addEventListener("tf_fixed_frame_changed", ()=>{
	drawFrames();
});

setInterval(updateVisibility, 1000);

window.addEventListener("view_changed", drawFrames);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();

console.log("TF Widget Loaded {uniqueID}")
