import { rosbridge } from './rosbridge.js';

export function applyRotation(vector, rotation){
	const r = new Quaternion(
		rotation.w,
		rotation.x,
		rotation.y,
		rotation.z
	);
		
	const v = r.rotateVector([
		vector.x,
		vector.y,
		vector.z
	]);

	return {
		x: v[0],
		y: v[1],
		z: v[2]
	}
}

export class TF {
	constructor() {
		this.frame = 'odom';
		this.transforms = {};
		this.absoluteTransforms = {};

		this.tf_topic = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: 'tf',
			messageType: 'tf/tfMessage',
		});

		this.tf_listener = this.tf_topic.subscribe((msg) => {
			this.updateTransforms(msg.transforms);
		});

		this.tf_static_topic = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: 'tf_static',
			messageType: 'tf/tfMessage',
		});

		this.tf_static_listener = this.tf_static_topic.subscribe((msg) => {
			this.updateTransforms(msg.transforms);
		});
	}

	updateTransforms(transforms) {
		transforms.forEach((transform) => {
			const childFrameId = transform.child_frame_id;
			const parentFrameId = transform.header.frame_id;
	
			this.transforms[childFrameId] = {
				translation: transform.transform.translation,
				rotation: transform.transform.rotation,
				parent: parentFrameId,
			};
		});
	
		this.recalculateAbsoluteTransforms();
		window.dispatchEvent(new Event('tf_changed'));
	}

	set_frame(newframe) {
		this.frame = newframe;
		this.recalculateAbsoluteTransforms();
	}

	recalculateAbsoluteTransforms() {
		for (const [key, value] of Object.entries(this.transforms)) {
			if(!key.includes("sonar") && !key.includes("sonar"))
				this.absoluteTransforms[key] = this.transformVector(key, this.frame, {x: 0, y:0, z:0}, {x: 0, y:0, z:0, w:1});
		};
	}

	transformVector(sourceFrame, targetFrame, inputVector, inputQuat) {
		let source = this.transforms[sourceFrame];	
		if (!source) {
			return {inputVector, inputQuat};
		}

		let outputVector = { x: inputVector.x, y: inputVector.y, z: inputVector.z };
		let outputQuat = new Quaternion(
			inputQuat.w,
			inputQuat.x,
			inputQuat.y,
			inputQuat.z
		);
	
		while (source && source.parent !== targetFrame) {

			outputQuat = (new Quaternion(
				source.rotation.w,
				source.rotation.x,
				source.rotation.y,
				source.rotation.z
			)).mul(outputQuat);

			outputVector = applyRotation(outputVector, source.rotation);
			outputVector.x += source.translation.x;
			outputVector.y += source.translation.y;
			outputVector.z += source.translation.z;
	
			source = this.transforms[source.parent];
		}
	
		if (source && source.parent === targetFrame) {

			outputQuat = (new Quaternion(
				source.rotation.w,
				source.rotation.x,
				source.rotation.y,
				source.rotation.z
			)).mul(outputQuat);

			outputVector = applyRotation(outputVector, source.rotation);
			outputVector.x += source.translation.x;
			outputVector.y += source.translation.y;
			outputVector.z += source.translation.z;
		}
	
		return {
			translation: outputVector,
			rotation: outputQuat
		};
	}
}

export let tf = new TF();
