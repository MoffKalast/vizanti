import { rosbridge } from './rosbridge.js';

export function applyRotation(vector, rotation, inverse){
	let r = new Quaternion(
		rotation.w,
		rotation.x,
		rotation.y,
		rotation.z
	);

	if(inverse)
		r = r.inverse();
		
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
		this.fixed_frame = 'odom';

		this.tf_tree = {};
		this.transforms = {};
		this.absoluteTransforms = {};

		this.frame_list = new Set();

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


		/* setTimeout(() =>{
			const testCases = [
				{
					input: ['map', 'map'],
					expected: ['map'],
				},
				{
					input: ['map', 'world'],
					expected: ['map', 'world'],
				},
				{
					input: ['map', 'prop_right'],
					expected: ['map', 'odom', 'base_link', 'prop_right'],
				},
				{
					input: ['prop_left', 'world'],
					expected: ['prop_left', 'base_link', 'odom', 'map', 'world'],
				},
				{
					input: ['prop_right', 'prop_left'],
					expected: ['prop_right', 'base_link', 'prop_left'],
				},
				{
					input: ['prop_right', 'gps'],
					expected: ['prop_right', 'base_link', 'gps'],
				},
				{
					input: ['gps', 'imu_link'],
					expected: ['gps', 'base_link', 'imu_link'],
				},
				{
					input: ['imu_link', 'map'],
					expected: ['imu_link', 'base_link', 'odom', 'map'],
				},
			];
			
			testCases.forEach((testCase, index) => {
				const result = this.findPath(...testCase.input);
				const isSuccess = JSON.stringify(result) === JSON.stringify(testCase.expected);
				console.log(
					`Test ${index + 1}: ${isSuccess ? 'PASSED' : 'FAILED ------'}`,
					'\nInput:',	testCase.input,
					'\nResult:', result,
					'\nExpected:', testCase.expected,
					'\n'
				);
			});
		}, 2000); */
	}

	addToTree(parentFrameId, childFrameId) {

		if (this.tf_tree.hasOwnProperty(childFrameId)) {
			delete this.tf_tree[childFrameId];
		}
	
		if (this.tf_tree.hasOwnProperty(parentFrameId)) {
			this.tf_tree[parentFrameId][childFrameId] = {};
		} else {
			const foundParent = (node) => {
				for (const key in node) {
					if (key === parentFrameId) {
						node[key][childFrameId] = {};
						return true;
					} else {
						if (foundParent(node[key])) {
							return true;
						}
					}
				}
				return false;
			};
			if (!foundParent(this.tf_tree)) {
				this.tf_tree[parentFrameId] = { [childFrameId]: {} };
			}
		}
	}

	getPathToRoot(frame) {
		const currentFrame = this.transforms[frame];
		if (!currentFrame) {
			return [frame];
		}
		if (!currentFrame.parent) {
			return [frame];
		}
		return [frame].concat(this.getPathToRoot(currentFrame.parent));
	}
	
	findPath(startFrame, endFrame) {
		const p = this.getPathToRoot(startFrame);
		const q = this.getPathToRoot(endFrame);
	
		let common = null;
		while (p.length > 0 && q.length > 0 && p[p.length - 1] === q[q.length - 1]) {
			common = p.pop();
			q.pop();
		}
	
		return p.concat(common, q.reverse());
	}

	updateTransforms(transforms) {
		transforms.forEach((transform) => {

			const childFrameId = transform.child_frame_id;
			const parentFrameId = transform.header.frame_id;

			this.frame_list.add(childFrameId);
			this.frame_list.add(parentFrameId);
	
			this.transforms[childFrameId] = {
				translation: transform.transform.translation,
				rotation: transform.transform.rotation,
				parent: parentFrameId,
				children: new Set()
			};

			Object.keys(this.transforms).forEach(key => {
				if(this.transforms[key].parent == childFrameId)
					this.transforms[childFrameId].children.add(key);
			});

			this.addToTree(parentFrameId, childFrameId);
		});

		this.recalculateAbsoluteTransforms();
		window.dispatchEvent(new Event('tf_changed'));
	}

	setFixedFrame(newframe) {
		this.fixed_frame = newframe;
		this.recalculateAbsoluteTransforms();
	}

	recalculateAbsoluteTransforms() {
		for (const key of this.frame_list.values()) {
			this.absoluteTransforms[key] = this.transformVector(key, this.fixed_frame, {x: 0, y:0, z:0}, {x: 0, y:0, z:0, w:1});
		}
	}

	getZeroFrame(){
		return {
			translation:{x: 0, y:0, z:0},
			rotation: {x: 0, y:0, z:0, w:1}
		}
	}

	transformVector(sourceFrame, targetFrame, inputVector, inputQuat) {

		if(sourceFrame == targetFrame){
			return {
				translation: inputVector,
				rotation: inputQuat
			};
		}

		let outputVector = { x: inputVector.x, y: inputVector.y, z: inputVector.z };
		let outputQuat = new Quaternion(
			inputQuat.w,
			inputQuat.x,
			inputQuat.y,
			inputQuat.z
		);

		const path = this.findPath(sourceFrame, targetFrame);

		for (let i = 0; i < path.length - 1; i++) {
			let source = this.transforms[path[i]];

			if(!source)
				source = this.getZeroFrame();

			if(source.parent == path[i+1]){
				outputQuat = (new Quaternion(
					source.rotation.w,
					source.rotation.x,
					source.rotation.y,
					source.rotation.z
				)).mul(outputQuat);
	
				outputVector = applyRotation(outputVector, source.rotation, false);
				outputVector.x += source.translation.x;
				outputVector.y += source.translation.y;
				outputVector.z += source.translation.z;
			}else{
				source = this.transforms[path[i+1]];

				if(!source)
					source = this.getZeroFrame();

				outputQuat = (new Quaternion(
					source.rotation.w,
					source.rotation.x,
					source.rotation.y,
					source.rotation.z
				)).inverse().mul(outputQuat);
	
				outputVector.x -= source.translation.x;
				outputVector.y -= source.translation.y;
				outputVector.z -= source.translation.z;
				outputVector = applyRotation(outputVector, source.rotation, true);
			}
		}
	
		return {
			translation: outputVector,
			rotation: outputQuat
		};
	}
}

export let tf = new TF();
