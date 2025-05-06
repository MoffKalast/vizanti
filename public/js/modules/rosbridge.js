import '../lib/roslib.min.js';

const paramsModule = await import(`${base_url}/ros_launch_params`);
const params = paramsModule.default;

class Rosbridge {

	constructor(url) { 
		this.url = url;
		this.port = params.port_rosbridge;
		this.connected = false;

		this.connect();
		this.status = "Connecting...";

		this.reset_reconnect = false;
	}

	connect(){
		this.connected = false;

		this.ros = new ROSLIB.Ros({
			url: 'ws://' + this.url + ':' + this.port
		});

		this.ros.on('connection', () => {
			console.log('Connected to robot.');

			this.connected = true;
			this.status = "Connected.";

			if(this.reset_reconnect){
				location.reload(false); //otherwise topics won't re-subscribe automatically :/
			}

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
			this.reset_reconnect = true;
			window.dispatchEvent(new Event('rosbridge_change'));

			setTimeout(() => {
				this.status = "Reconnecting...";
				this.connect();
			}, 1000);
		});

		this.topics_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/topics',
			serviceType : 'rosapi/Topics',
		});

		this.nodes_clinet = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/nodes',
			serviceType : 'rosapi/Nodes',
		});

		this.publishers_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/publishers',
			serviceType : 'rosapi/Publishers',
		});

		this.subscribers_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/subscribers',
			serviceType : 'rosapi/Subscribers',
		});

		this.services_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/services',
			serviceType : 'rosapi/Services',
		});
		
		this.services_for_type_client = new ROSLIB.Service({
			ros : this.ros,
			name : '/rosapi/services_for_type',
			serviceType : 'rosapi/ServicesForType',
		});

		window.dispatchEvent(new Event('rosbridge_change'));
	}

	async get_all_nodes() {
		return new Promise(async (resolve) => {
			this.nodes_clinet.callService(new ROSLIB.ServiceRequest({}), function (result) {
				resolve(result);
			});
		});
	}

	async get_all_topics() {
		return new Promise((resolve) => {
			this.topics_client.callService(new ROSLIB.ServiceRequest({}), (result) => {
				const combined = result.topics.map((t, i) => [t, result.types[i]]);
				combined.sort((a, b) => a[0].localeCompare(b[0]));
				result.topics = combined.map(([t]) => t);
				result.types = combined.map(([, ty]) => ty);
				resolve(result);
			});
		});
	}
	
	async get_topics(requested_type) {
		return new Promise(async (resolve) => {
			this.topics_client.callService(new ROSLIB.ServiceRequest({}), function (result) {
	
				let topics = result.topics;
				let types = result.types;
	
				let matching = [];
				for (let i = 0; i < topics.length; i++) {
					if (types[i] == requested_type) {
						matching.push(topics[i]);
					}
				}
					
				matching.sort();
				resolve(matching);
			});
		});
	}
	

	async get_services(requested_type) {
		return new Promise((resolve, reject) => {
			const request = new ROSLIB.ServiceRequest({ type: requested_type });
	
			this.services_for_type_client.callService(request, (result) => {
				resolve(result.services);
			}, (err) => {
				console.error(`Failed to fetch services for type ${requested_type}:`, err);
				resolve([]);
			});
		});
	}

	async get_topic_publishers_and_subscribers(topic) {
		const publishersRequest = new ROSLIB.ServiceRequest({ topic: topic });
		const subscribersRequest = new ROSLIB.ServiceRequest({ topic: topic });

		const publishersPromise = new Promise((resolve) => {
			this.publishers_client.callService(publishersRequest, (result) => {
				resolve(result.publishers);
			});
		});

		const subscribersPromise = new Promise((resolve) => {
			this.subscribers_client.callService(subscribersRequest, (result) => {
				resolve(result.subscribers);
			});
		});

		const [publishers, subscribers] = await Promise.all([publishersPromise, subscribersPromise]);

		return { publishers, subscribers };
	}
}

export var rosbridge = new Rosbridge(window.location.hostname);