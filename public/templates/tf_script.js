import { view } from '/js/modules/view.js';
import { tf, applyRotation } from '/js/modules/tf.js';
import { settings } from '/js/modules/persistent.js';

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

function drawLines(origin){
	const relative = tf.transforms;
	const absolute = tf.absoluteTransforms;

	ctx.strokeStyle = "#eba834";
	ctx.lineWidth = 1;

	Object.keys(absolute).forEach(key => {

		let transform = absolute[key];

		let point = view.mapToScreen({
			x: transform.translation.x,
			y: transform.translation.y,
		});

		ctx.fillText(key, point.x, point.y+15);

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

function drawText(origin){
	const absolute = tf.absoluteTransforms;

	ctx.font = "12px Monospace";
	ctx.textAlign = "center";
	ctx.fillStyle = "white";

	ctx.strokeStyle = "#161B21";
	ctx.lineWidth = 5;

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

function drawAxes(origin) {

	const absolute = tf.absoluteTransforms;
	const unit = getPixelsInMapUnits(30);
	const width_unit = getPixelsInMapUnits(2);

	ctx.lineWidth = 2;
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

function drawFrames() {

	ctx.clearRect(0, 0, canvas.width, canvas.height);

	let origin = view.mapToScreen({
		x: 0,
		y: 0,
	});

	drawLines(origin);
	drawAxes(origin);
	drawText(origin);
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

