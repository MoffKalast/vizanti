import { rosbridge } from './rosbridge.js';

export function applyRotation(vector, r, inverse){
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

		//this.previousTime = null;
		//this.previousTransforms = null;
		//this.lastReceivedTransforms = null;
		//this.lastReceivedTime = null;

		this.tf_topic = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: 'tf/consolidated',
			messageType: 'tf/tfMessage',
			throttle_rate: 30
		});
		
		const setListener = () => {
			this.tf_listener = this.tf_topic.subscribe((msg) => {
				//this.previousTransforms = this.lastReceivedTransforms;
				//this.previousTime = this.lastReceivedTime;
	
				//this.lastReceivedTransforms = msg.transforms;
				//this.lastReceivedTime = performance.now();
				this.updateTransforms(msg.transforms);
			});
		};

		setListener();

		this.tf_static_topic = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: 'tf_static',
			messageType: 'tf/tfMessage',
		});

		this.tf_static_listener = this.tf_static_topic.subscribe((msg) => {
			this.updateTransforms(msg.transforms);
		});

		setInterval(()=>{
			if(performance.now() - this.lastReceivedTime > 1000){
				console.log('%c TF Connection Reset!', 'background: #222; color: #bada55');

				this.tf_topic.unsubscribe(this.tf_listener);
				setListener();
			}
		},1000);
	}

	/* interpolateTransforms() {
		if (this.lastReceivedTransforms !== null && this.previousTransforms !== null && performance.now() - this.lastReceivedTime > 15) {
			const prevdelta = (this.lastReceivedTime - this.previousTime) / 1000;
			let delta = ((performance.now() - this.lastReceivedTime) / 1000) / prevdelta;

			console.log("interpolation")

			delta *= 0.005;

			const transformsMap = new Map();
			this.previousTransforms.forEach(transform => {
				transformsMap.set(transform.child_frame_id, { previous: transform });
			});
	
			this.lastReceivedTransforms.forEach(transform => {
				if (transformsMap.has(transform.child_frame_id)) {
					transformsMap.get(transform.child_frame_id).current = transform;
				}
			});
	
			const interpolatedTransforms = [];
	
			transformsMap.forEach(({ previous, current }, child_frame_id) => {
				if (current) {
					const positionDelta = {
						x: current.transform.translation.x - previous.transform.translation.x,
						y: current.transform.translation.y - previous.transform.translation.y
					};
					const velocity = {
						x: positionDelta.x * delta,
						y: positionDelta.y * delta,
					};

					const interpolatedTransform = {
						child_frame_id: child_frame_id,
						header: current.header,
						transform: {
							translation: {
								x: current.transform.translation.x + velocity.x,
								y: current.transform.translation.y + velocity.y,
								z: current.transform.translation.z
							},
							rotation: current.transform.rotation
						}
					};
					interpolatedTransforms.push(interpolatedTransform);
				}
			});
	
			this.updateTransforms(interpolatedTransforms);
		}
	} */

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

	updateTransforms(newtransforms) {
		newtransforms.forEach((pose) => {

			const childFrameId = pose.child_frame_id;
			const parentFrameId = pose.header.frame_id;

			this.frame_list.add(childFrameId);
			this.frame_list.add(parentFrameId);
	
			this.transforms[childFrameId] = {
				translation: pose.transform.translation,
				rotation: new Quaternion(
					pose.transform.rotation.w,
					pose.transform.rotation.x,
					pose.transform.rotation.y,
					pose.transform.rotation.z
				),
				parent: parentFrameId
			};
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
			this.absoluteTransforms[key] = this.transformPose(key, this.fixed_frame, {x: 0, y:0, z:0}, new Quaternion());
		}
	}

	getZeroFrame(){
		return {
			translation:{x: 0, y:0, z:0},
			rotation: new Quaternion()
		}
	}

	transformPose(sourceFrame, targetFrame, inputVector, inputQuat) {

		let outputVector =  Object.assign({}, inputVector);
		let outputQuat =  new Quaternion(inputQuat);

		if(sourceFrame == targetFrame){
			return {
				translation: outputVector,
				rotation: outputQuat
			};
		}

		const path = this.findPath(sourceFrame, targetFrame);

		for (let i = 0; i < path.length - 1; i++) {
			let source = this.transforms[path[i]];

			if(!source)
				source = this.getZeroFrame();

			if(source.parent == path[i+1]){
				outputQuat = source.rotation.mul(outputQuat);
	
				outputVector = applyRotation(outputVector, source.rotation, false);
				outputVector.x += source.translation.x;
				outputVector.y += source.translation.y;
				outputVector.z += source.translation.z;
			}else{
				source = this.transforms[path[i+1]];

				if(!source)
					source = this.getZeroFrame();

				outputQuat = source.rotation.inverse().mul(outputQuat);
	
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
