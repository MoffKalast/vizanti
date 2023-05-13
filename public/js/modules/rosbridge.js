import '/js/lib/roslib.min.js';

class Rosbridge {

	constructor(url, port) { 
		this.url = url;
		this.port = port;
		this.connected = false;

		this.connect();
		this.status = "Connecting...";

		console.log("bridge created!")
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
		return new Promise(async (resolve) => {
			this.topics_client.callService(new ROSLIB.ServiceRequest({}), function (result) {
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
				for(let i = 0; i < topics.length; i++){
					if(types[i] == requested_type){
						matching.push(topics[i]);
					}
				}

				resolve(matching);
			});
		});
	}
}

export var rosbridge = new Rosbridge(window.location.hostname, 5001);