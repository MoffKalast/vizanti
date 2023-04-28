import { view } from '/js/modules/view.js';
import { tf, applyRotation } from '/js/modules/tf.js';
import { settings } from '/js/modules/persistent.js';

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

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
}
else{
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

function drawLines(origin, relative, absolute){

	ctx.strokeStyle = "#eba834";
	ctx.lineWidth = 1*parseFloat(scaleSlider.value);
	ctx.beginPath();

	Object.keys(absolute).forEach(key => {

		let transform = absolute[key];

		let point = view.mapToScreen({
			x: transform.translation.x,
			y: transform.translation.y,
		});

		let parent = absolute[relative[key].parent];
		if(parent !== undefined){

			let parentpoint = view.mapToScreen({
				x: parent.translation.x,
				y: parent.translation.y,
			});

			ctx.moveTo(parseInt(point.x), parseInt(point.y));
			ctx.lineTo(parseInt(parentpoint.x), parseInt(parentpoint.y));
		}
		else{
			ctx.moveTo(parseInt(point.x), parseInt(point.y));
			ctx.lineTo(parseInt(origin.x), parseInt(origin.y));
		}		
	});

	ctx.stroke();

}

function drawText(origin, relative, absolute){

	ctx.font = (12*parseFloat(scaleSlider.value))+"px Monospace";
	ctx.textAlign = "center";
	ctx.fillStyle = "white";

	ctx.strokeStyle = "#161B21";
	ctx.lineWidth = 5*parseFloat(scaleSlider.value);

	ctx.strokeText(tf.frame, origin.x, origin.y+15);
	ctx.fillText(tf.frame, origin.x, origin.y+15);

	Object.keys(absolute).forEach(key => {

		let transform = absolute[key];

		let point = view.mapToScreen({
			x: transform.translation.x,
			y: transform.translation.y,
		});

		ctx.strokeText(key, point.x, point.y+15);
		ctx.fillText(key, point.x, point.y+15);	
	});
}

function getPixelsInMapUnits(length){
	let p1 = view.screenToMap({
		x: 0,
		y: 0,
	});

	let p2 = view.screenToMap({
		x: length,
		y: 0,
	});

	return Math.abs(p1.x-p2.x);
}

function getBasisPoints(basis, translation, rotation){
	let basis_x = applyRotation(basis, rotation);
	return [
		view.mapToScreen({
			x: translation.x,
			y: translation.y,
		}), 
		view.mapToScreen({
			x: translation.x + basis_x.x,
			y: translation.y + basis_x.y,
		})
	];
}

function drawAxes(origin, relative, absolute) {

	const unit = getPixelsInMapUnits(30*parseFloat(scaleSlider.value));
	const width_unit = getPixelsInMapUnits(2*parseFloat(scaleSlider.value));

	ctx.lineWidth = 2*parseFloat(scaleSlider.value);
	ctx.strokeStyle = "#E0000B"; //red
	ctx.beginPath();

	let xarrow = view.mapToScreen({
		x: unit,
		y: 0,
	});

	ctx.moveTo(parseInt(origin.x), parseInt(origin.y));
	ctx.lineTo(parseInt(xarrow.x), parseInt(xarrow.y));

	Object.keys(absolute).forEach(key => {
		let transform = absolute[key];
		let p = getBasisPoints(
			{x: unit, y: 0, z: 0},
			transform.translation,
			transform.rotation
		);
		ctx.moveTo(parseInt(p[0].x), parseInt(p[0].y));
		ctx.lineTo(parseInt(p[1].x), parseInt(p[1].y));
	});
	
	ctx.stroke();

	ctx.strokeStyle = "#29FF26"; //green
	ctx.beginPath();

	let yarrow = view.mapToScreen({
		x: 0,
		y: unit,
	});

	ctx.moveTo(parseInt(origin.x), parseInt(origin.y));
	ctx.lineTo(parseInt(yarrow.x), parseInt(yarrow.y));

	Object.keys(absolute).forEach(key => {
		let transform = absolute[key];
		let p = getBasisPoints(
			{x: 0, y: unit, z: 0},
			transform.translation,
			transform.rotation
		);
		ctx.moveTo(parseInt(p[0].x), parseInt(p[0].y));
		ctx.lineTo(parseInt(p[1].x), parseInt(p[1].y));
	});

	ctx.stroke();


	ctx.strokeStyle = "#005DFF"; //blue
	ctx.fillStyle = "#005DFF";
	ctx.fillRect(parseInt(origin.x)-1, parseInt(origin.y)-1, 2, 2);

	ctx.beginPath();

	Object.keys(absolute).forEach(key => {
		let transform = absolute[key];
		let p = getBasisPoints(
			{x: 0, y: 0, z: unit},
			transform.translation,
			transform.rotation
		);
		ctx.moveTo(parseInt(p[0].x), parseInt(p[0].y));
		ctx.lineTo(parseInt(p[1].x), parseInt(p[1].y));
	});

	ctx.stroke();

}

function filterFrames(framelist){
	let filteredlist = {};
	Object.keys(framelist).forEach(key => {
		if(frame_visibility.hasOwnProperty(key) && frame_visibility[key])
			filteredlist[key] = framelist[key];
	});
	return filteredlist;
}

function drawFrames() {
	ctx.clearRect(0, 0, canvas.width, canvas.height);

	const relative = filterFrames(tf.transforms);
	const absolute = filterFrames(tf.absoluteTransforms);

	let origin = view.mapToScreen({
		x: 0,
		y: 0,
	});

	if(linesCheckbox.checked)
		drawLines(origin, relative, absolute);

	if(axesCheckbox.checked)
		drawAxes(origin, relative, absolute);

	if(namesCheckbox.checked)
		drawText(origin, relative, absolute);

	updateFrameBox();
}

const framesDiv = document.getElementById('{uniqueID}_frames');

function updateVisibility(){
	framesDiv.innerHTML = '';
	Object.keys(frame_visibility).forEach(key => {
		const checkbox = document.createElement('input');
		checkbox.type = 'checkbox';
		checkbox.id = `{uniqueID}_${key}`;
		checkbox.checked = frame_visibility[key];
		checkbox.addEventListener('change', (event) => {
			frame_visibility[key] = event.target.checked;
			saveSettings();
		});

		const label = document.createElement('label');
		label.textContent = ` ${key}`;

		const br = document.createElement('br');

		framesDiv.appendChild(checkbox);
		framesDiv.appendChild(label);
		framesDiv.appendChild(br);
	});
}

function updateFrameBox(){
	let updatevisibility = false;
	Object.keys(tf.transforms).forEach(key => {
		if(!frame_visibility.hasOwnProperty(key)){
			updatevisibility = true;
			frame_visibility[key] = true;
		}
	});

	if (updatevisibility) {
		updateVisibility();
	}
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawFrames();
}

window.addEventListener("tf_changed", drawFrames);
window.addEventListener("view_changed", drawFrames);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

resizeScreen();
updateVisibility();

