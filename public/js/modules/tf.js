import { rosbridge } from './rosbridge.js';

export function applyRotation(vector, r, inverse) {
	if (inverse)
		r = r.inverse();

	const v = r.rotateVector([vector.x, vector.y, vector.z]);
	return { x: v[0], y: v[1], z: v[2] };
}

// Fixed-size ring buffer storing { secs, nsecs, transform } entries per frame.
// Oldest entry is overwritten once capacity is reached.
class TransformRingBuffer {
	constructor(capacity) {
		this.capacity = capacity;
		this.buf = new Array(capacity);
		this.head = 0; // next write position
		this.size = 0;
	}

	push(secs, nsecs, transform) {
		this.buf[this.head] = { t: secs + nsecs * 1e-9, transform };
		this.head = (this.head + 1) % this.capacity;
		if (this.size < this.capacity) this.size++;
	}

	// Returns the transform whose timestamp is closest to the given stamp.
	// Falls back to the most recently pushed entry if called with no arguments.
	// Full scan (no early exit) handles non-monotonic clocks (e.g. rosbag restart).
	nearest(secs, nsecs) {
		if (this.size === 0)
			return null;

		const latest = (this.head - 1 + this.capacity) % this.capacity;

		if (secs == null || nsecs == null)
			return this.buf[latest].transform;

		const target = secs + nsecs * 1e-9;
		let bestIdx = latest;
		let bestDist = Infinity;

		for (let i = 0; i < this.size; i++) {
			const idx = (this.head - 1 - i + this.capacity) % this.capacity;
			const dist = Math.abs(this.buf[idx].t - target);
			if (dist < bestDist) {
				bestDist = dist;
				bestIdx = idx;
			}
		}

		return this.buf[bestIdx].transform;
	}
}

export class TF {
	constructor() {
		this.fixed_frame = '';

		this.transforms = {};
		this.absoluteTransforms = {};
		this.absoluteTransformBuffers = {};
		this.frame_list = new Set();
		this.frame_timestamps = {};
		this.frame_headerstamps = {};

		this.tf_topic = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: 'vizanti/tf_consolidated',
			messageType: 'tf/tfMessage',
			throttle_rate: 33,
			compression: "cbor",
			queue_length: 1
		});

		this.tf_listener = this.tf_topic.subscribe((msg) => {
			const time_stamp = new Date();
			msg.transforms.forEach((pose) => {
				this.frame_timestamps[pose.child_frame_id] = time_stamp;
				this.frame_timestamps[pose.header.frame_id] = time_stamp;
				this.updateFrameTimestamp(pose.child_frame_id, pose.header.stamp);
			});

			this.updateTransforms(msg.transforms);
		});

		this.tf_static_topic = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: 'tf_static',
			messageType: 'tf/tfMessage',
			compression: "cbor"
		});

		this.tf_static_listener = this.tf_static_topic.subscribe((msg) => {
			this.updateTransforms(msg.transforms);
		});

		this.event_timestamp = performance.now();

		window.addEventListener("view_changed", () => {
			this.event_timestamp = performance.now();
		});

		setInterval(() => {
			const now = new Date();
			let deleted_anything = false;
			for (const [frame_id, time_stamp] of Object.entries(this.frame_timestamps)) {
				if (now - time_stamp > 1000 * 100) {
					this.frame_list.delete(frame_id);
					delete this.transforms[frame_id];
					delete this.absoluteTransforms[frame_id];
					delete this.absoluteTransformBuffers[frame_id];
					delete this.frame_headerstamps[frame_id];
					delete this.frame_timestamps[frame_id];
					deleted_anything = true;
				}
			}

			if (deleted_anything)
				window.dispatchEvent(new Event("tf_changed"));
		}, 5000);
	}

	updateFrameTimestamp(frame_id, stamp) {
		this.frame_headerstamps[frame_id] = stamp;

		for (const frame in this.transforms) {
			if (this.transforms[frame].parent === frame_id) {
				this.frame_headerstamps[frame] = stamp;
				this.updateFrameTimestamp(frame, stamp);
			}
		}
	}

	sendUpdateEvent() {
		if (performance.now() - this.event_timestamp > 12) {
			window.dispatchEvent(new Event("tf_changed"));
			this.event_timestamp = performance.now();
		}
	}

	getPathToRoot(frame) {
		const currentFrame = this.transforms[frame];
		if (!currentFrame) return [frame];
		if (!currentFrame.parent) return [frame];
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
			this.frame_list = new Set([...this.frame_list].sort());

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
		});

		this.recalculateAbsoluteTransforms();
		this.sendUpdateEvent();
	}

	setFixedFrame(newframe) {
		this.fixed_frame = newframe;
		this.absoluteTransformBuffers = {};
		this.recalculateAbsoluteTransforms();
		window.dispatchEvent(new Event("tf_fixed_frame_changed"));
	}

	recalculateAbsoluteTransforms() {
		for (const key of this.frame_list.values()) {
			const transform = this.transformPose(key, this.fixed_frame, { x: 0, y: 0, z: 0 }, new Quaternion());
			this.absoluteTransforms[key] = transform;

			if (!this.absoluteTransformBuffers[key])
				this.absoluteTransformBuffers[key] = new TransformRingBuffer(16);

			const stamp = this.frame_headerstamps[key];
			if (stamp)
				this.absoluteTransformBuffers[key].push(stamp.secs, stamp.nsecs, transform);
		}
	}

	getZeroFrame() {
		return {
			translation: { x: 0, y: 0, z: 0 },
			rotation: new Quaternion()
		};
	}

	getAbsoluteTransform(header) {
		const buf = this.absoluteTransformBuffers[header.frame_id];
		if (!buf)
			return this.absoluteTransforms[header.frame_id];
		return buf.nearest(header.stamp?.secs, header.stamp?.nsecs) ?? this.absoluteTransforms[header.frame_id];
	}

	// Timestamp-aware drop-in for transformPose(frame, fixed_frame, position, orientation).
	// Applies the offset pose on top of the buffered absolute transform for header.frame_id.
	transformPoseStamped(header, position, orientation) {
		const abs = this.getAbsoluteTransform(header);
		
		if (!abs)
			return this.getZeroFrame();

		const inputQuat = new Quaternion(orientation);

		// Rotate the local offset into fixed-frame space, then add the frame's fixed-frame origin.
		const rotated = applyRotation(position, abs.rotation, false);
		return {
			translation: {
				x: abs.translation.x + rotated.x,
				y: abs.translation.y + rotated.y,
				z: abs.translation.z + rotated.z
			},
			rotation: abs.rotation.mul(inputQuat)
		};
	}

	transformPose(sourceFrame, targetFrame, inputVector, inputQuat) {
		let outputVector = Object.assign({}, inputVector);
		let outputQuat = new Quaternion(inputQuat);

		if (sourceFrame == targetFrame)
			return { translation: outputVector, rotation: outputQuat };

		const path = this.findPath(sourceFrame, targetFrame);

		for (let i = 0; i < path.length - 1; i++) {
			let source = this.transforms[path[i]];

			if (!source)
				source = this.getZeroFrame();

			if (source.parent == path[i + 1]) {
				outputQuat = source.rotation.mul(outputQuat);
				outputVector = applyRotation(outputVector, source.rotation, false);
				outputVector.x += source.translation.x;
				outputVector.y += source.translation.y;
				outputVector.z += source.translation.z;
			} else {
				source = this.transforms[path[i + 1]];

				if (!source)
					source = this.getZeroFrame();

				outputQuat = source.rotation.inverse().mul(outputQuat);
				outputVector.x -= source.translation.x;
				outputVector.y -= source.translation.y;
				outputVector.z -= source.translation.z;
				outputVector = applyRotation(outputVector, source.rotation, true);
			}
		}

		return { translation: outputVector, rotation: outputQuat };
	}

	getTimeStampDelta(timestamp1, timestamp2) {
		return timestamp2.secs - timestamp1.secs + ((timestamp2.nsecs - timestamp1.nsecs) / 1e9);
	}
}

export let tf = new TF();