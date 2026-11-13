import '../lib/roslib.min.js';

const paramsModule = await import(`${base_url}/ros_launch_params`);
const params = paramsModule.default;

class Rosbridge {

	constructor(url) { 
		this.url = url;
		this.port = params.port_rosbridge;
		this.compression = params.compression;
		this.connected = false;
		this.reconnect_pending = false;
		this.suspended = false;
		this.suspend_timer = undefined;
		this.status = "Connecting...";

		this.ros = new ROSLIB.Ros({
			url: 'ws://' + this.url + ':' + this.port
		});

		this.ros.on('connection', () => {
			console.log('Connected to robot.');
			this.connected = true;
			this.status = "Connected.";
			window.dispatchEvent(new Event('rosbridge_change'));
		});

		this.ros.on('error', (error) => {
			this.connected = false;
			this.status = "Failed to connect.";
			window.dispatchEvent(new Event('rosbridge_change'));
		});

		this.ros.on('close', () => {
			this.connected = false;
			this.status = "Connection lost.";
			window.dispatchEvent(new Event('rosbridge_change'));

			if (this.suspended || this.reconnect_pending)
				return;

			this.reconnect_pending = true;
			setTimeout(() => {
				this.reconnect_pending = false;
				if (this.suspended)
					return;
				this.status = "Reconnecting...";
				window.dispatchEvent(new Event('rosbridge_change'));
				this.ros.connect('ws://' + this.url + ':' + this.port);
			}, 1000);
		});

		document.addEventListener('visibilitychange', () => {
			if (document.hidden) {
				this.suspended = true;
				this.status = "Suspended (tab inactive).";
				setTimeout(() => this.ros.close(), 5);
			} else {
				if (this.suspended) {
					this.suspended = false;
					this.status = "Reconnecting...";
					window.dispatchEvent(new Event('rosbridge_change'));
					this.ros.connect('ws://' + this.url + ':' + this.port);
				}
			}
		});

		this.topics_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/topics',
			serviceType : 'rosapi_msgs/srv/Topics',
		});

		this.nodes_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/nodes',
			serviceType : 'rosapi_msgs/srv/Nodes',
		});

		this.publishers_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/publishers',
			serviceType : 'rosapi_msgs/srv/Publishers',
		});

		this.subscribers_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/subscribers',
			serviceType : 'rosapi_msgs/srv/Subscribers',
		});

		this.services_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/services',
			serviceType : 'rosapi_msgs/srv/Services',
		});
		
		this.services_for_type_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/services_for_type',
			serviceType : 'rosapi_msgs/srv/ServicesForType',
		});

		window.dispatchEvent(new Event('rosbridge_change'));
	}

	call_rosapi(client, request = {}) {
		return new Promise((resolve) => {
			const attempt = () => client.callService(new ROSLIB.ServiceRequest(request), resolve, (error) => {
				console.warn(`${client.name} call failed, retrying:`, error);
				setTimeout(attempt, 1000);
			});

			attempt();
		});
	}

	async get_all_nodes() {
		return await this.call_rosapi(this.nodes_client);
	}

	async get_all_topics() {
		const result = await this.call_rosapi(this.topics_client);
		const combined = result.topics.map((t, i) => [t, result.types[i]]);
		combined.sort((a, b) => a[0].localeCompare(b[0]));

		result.topics = combined.map(([t]) => t);
		result.types = combined.map(([, ty]) => ty);
		return result;
	}

	async get_topics(requested_type) {
		const result = await this.call_rosapi(this.topics_client);
		return result.topics.filter((topic, i) => result.types[i] == requested_type).sort();
	}

	async get_services(requested_type) {
		const result = await this.call_rosapi(this.services_for_type_client, { type: requested_type });
		return result.services;
	}

	async get_topic_publishers_and_subscribers(topic) {
		const [publishers, subscribers] = await Promise.all([this.call_rosapi(this.publishers_client, { topic: topic }), this.call_rosapi(this.subscribers_client, { topic: topic })]);
		return { publishers: publishers.publishers, subscribers: subscribers.subscribers };
	}
}

export var rosbridge = new Rosbridge(window.location.hostname);