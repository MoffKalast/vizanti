import { rosbridge } from './rosbridge.js';

/* class TimeStampedData {
	constructor(maxLength) {
		this.maxLength = maxLength;
		this.data = [];
		this.index = new Map();
	}

	getKey({ secs, nsecs }) {
		// Use the provided data structure to form a unique key
		// The precision should be enough for up to 1ns resolution over approximately 285 years.
		return secs * 1e9 + nsecs;
	}

	add(key, value) {
		const timestampKey = this.getKey(key);

		// If we're at maximum capacity, remove the oldest item
		if (this.data.length >= this.maxLength) {
			const oldestKey = this.getKey(this.data[0].key);
			this.index.delete(oldestKey);
			this.data.shift();
		}

		// Add the new item
		this.data.push({ key, value });
		this.index.set(timestampKey, value);
	}

	get(key) {
		const timestampKey = this.getKey(key);
		return this.index.get(timestampKey);
	}

	find(key) {
		const timestampKey = this.getKey(key);
		const keys = Array.from(this.index.keys());

		console.log("Find: ",timestampKey," in ",keys);

		// Find the key closest to the input timestamp
		const closestKey = keys.reduce((prev, curr) => {
			return (Math.abs(curr - timestampKey) < Math.abs(prev - timestampKey) ? curr : prev);
		});
		return this.index.get(closestKey);
	}
} */

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
		this.fixed_frame = '';

		this.tf_tree = {};
		this.transforms = {};
		this.absoluteTransforms = {};
		//this.absoluteTransformsHistory = new TimeStampedData(20);
		this.frame_list = new Set();
		this.frame_timestamps = {};

		this.tf_topic = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: 'vizanti/tf_consolidated',
			messageType: 'tf/tfMessage',
			throttle_rate: 33,
			compression: "cbor"
		});

		this.tf_listener = this.tf_topic.subscribe((msg) => {

			//local timestamping for removing inactive frames
			const time_stamp = new Date();
			msg.transforms.forEach((pose) => {
				this.frame_timestamps[pose.child_frame_id] = time_stamp;
				this.frame_timestamps[pose.header.frame_id] = time_stamp;
			})

			this.updateTransforms(msg.transforms, false);
			//this.absoluteTransformsHistory.add(msg.transforms[0].header.stamp, this.absoluteTransforms);
		});

		this.tf_static_topic = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: 'tf_static',
			messageType: 'tf/tfMessage',
			compression: "cbor"
		});

		this.tf_static_listener = this.tf_static_topic.subscribe((msg) => {
			this.updateTransforms(msg.transforms, true);
		});

		this.event_timestamp = performance.now();

		window.addEventListener("view_changed", ()=> {
			this.event_timestamp = performance.now();
		});

		//removing inactive TF frames
		setInterval(()=>{
			const now = new Date()
			let deleted_anything = false;
			for (const [frame_id, time_stamp] of Object.entries(this.frame_timestamps)) {
				if(now - time_stamp > 1000 * 10){
					delete this.tf_tree[frame_id];
					delete this.frame_list[frame_id];
					delete this.transforms[frame_id];
					delete this.absoluteTransforms[frame_id];
					deleted_anything = true;
				}
			}

			if(deleted_anything){
				window.dispatchEvent(new Event("tf_changed"));
			}
		},5000)
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

	async sendUpdateEvent(){
		if(performance.now() - this.event_timestamp > 12){
			window.dispatchEvent(new Event("tf_changed"));
			this.event_timestamp = performance.now();
		}
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
		this.sendUpdateEvent();
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
