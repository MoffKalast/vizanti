import { rosbridge } from '/js/modules/rosbridge.js';

async function getNodes() {
	const nodesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/rosapi/nodes",
		serviceType: "rosapi/Nodes",
	});

	return new Promise((resolve, reject) => {
		nodesService.callService(new ROSLIB.ServiceRequest(), (result) => {
			resolve(result.nodes);
		}, (error) => {
			reject(error);
		});
	});
}
  
async function getParamList() {
	const service = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: '/rosapi/get_param_names',
		serviceType: 'osapi/GetParamNames',
	});

	return new Promise((resolve, reject) => {
		service.callService(new ROSLIB.ServiceRequest(), (response) => {
			resolve(response.names);
		}, (error) => {
			reject(error);
		});
	});
}

async function getNodeDetails(name) {	
	const detailsService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/rosapi/node_details",
		serviceType: "rosapi/NodeDetails",
	});

	return new Promise((resolve, reject) => {
		detailsService.callService(new ROSLIB.ServiceRequest({node: name}), (details) => {
			resolve(details);
		}, (error) => {
			reject(error);
		});
	});
}

async function getParamValue(node, paramName) {
	const service = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: '/rosapi/get_param',
		serviceType: 'rosapi/GetParam',
	});

	return new Promise((resolve, reject) => {

		const request = new ROSLIB.ServiceRequest({
			name: node + '/' + paramName,
		});

		service.callService(request, (response) => {
			resolve({
				name: paramName,
				value: response.value,
			});
		}, (error) => {
			reject(error);
		});
	});
}

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const nodeSelector = document.getElementById("{uniqueID}_node");

let nodeName = "";

async function listParameters(){
	if(nodeName == "")
		return;

	const params = await getParamList();
	const nodeParams = params.filter((param) => param.startsWith(nodeName + "/"));

	



	console.log(nodeParams);
}

async function setList(){

	let result = await getNodes();
	let nodelist = "";
	for (const node of result) {
		const details = await getNodeDetails(node);

		console.log(details)
  
		if (details.services.includes(node + '/set_parameters')) {
			nodelist += "<option value='"+node+"'>"+node+"</option>"
		}
	}

	nodeSelector.innerHTML = nodelist

	if(nodeName == "")
		nodeName = nodeSelector.value;
	else if(result.includes(nodeName)){
		nodeSelector.value = nodeName;
	}

	listParameters();
}

nodeSelector.addEventListener("change", listParameters);
nodeSelector.addEventListener("click", listParameters);

icon.addEventListener("click", setList);

setList();

console.log("Reconfigure Widget Loaded {uniqueID}")