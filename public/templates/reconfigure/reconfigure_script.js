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

async function getParamValue(paramName) {
	const service = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: '/rosapi/get_param',
		serviceType: 'rosapi/GetParam',
	});

	return new Promise((resolve, reject) => {

		const request = new ROSLIB.ServiceRequest({
			name: paramName,
		});

		service.callService(request, (response) => {
			resolve(response.value);
		}, (error) => {
			reject(error);
		});
	});
}

async function setNodeParamValue(fullname, type, newValue) {
	const setParamClient = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: nodeName + '/set_parameters',
		serviceType: 'dynamic_reconfigure/Reconfigure',
	});

	let paramName = fullname.replace(nodeName+"/","");

	let valueConfig = {
		bools: [],
		ints: [],
		strs: [],
		doubles: [],
		groups: [],
	};

	switch (type) {
		case "string": valueConfig.strs.push({ name: paramName, value: newValue }); break;
		case "int": valueConfig.ints.push({ name: paramName, value: newValue }); break;
		case "float": valueConfig.doubles.push({ name: paramName, value: newValue }); break;
		case "bool": valueConfig.bools.push({ name: paramName, value: newValue }); break;
		default: return Promise.reject(`Invalid parameter value type: ${valueType}`);
	}

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ config: valueConfig });
		setParamClient.callService(request, (response) => {
			console.log(`Parameter ${paramName} set to:`, newValue);
			resolve(response);
		}, (error) => {
			console.error(`Failed to call set_parameters service for ${nodeName}:`, error);
			reject(error);
		});
	});
}

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const nodeSelector = document.getElementById("{uniqueID}_node");
const infoLabel = document.getElementById("{uniqueID}_info");
const paramBox = document.getElementById("{uniqueID}_params");

function createParameterInput(fullname, defaultValue, type) {
	const name = fullname.replace(nodeName+"/","");
	const id = "${uniqueID}_"+fullname;
	let inputElement;

	switch (type) {
		case "string":
			inputElement = `
				<label for="${id}"><i>string </i> ${name}:</label>
				<input id="${id}" type="text" value="${defaultValue}">
				<br>`;
			break;
		case "int":
			inputElement = `
				<label for="${id}"><i>int </i> ${name}:</label>
				<input type="number" value="${defaultValue}" step="1" id="${id}">
				<br>`;
			break;
		case "float":
			inputElement = `
				<label for="${id}"><i>float </i>${name}:</label>
				<input type="number" value="${defaultValue}" step="0.001" id="${id}">
				<br>`;
			break;
		case "bool":
			inputElement = `
				<label for="${id}"><i>bool </i>${name}:</label>
				<input type="checkbox" id="${id}" ${defaultValue ? "checked" : ""}>
				<br>`;
			break;
		default:
			console.error("Invalid parameter type:", type);
			return;
	}

	paramBox.insertAdjacentHTML("beforeend", inputElement);

	document.getElementById(id).addEventListener("change", (event) => {
		let val;

		switch(type){
			case "string": val = event.target.value; break;
			case "int": val = parseInt(event.target.value); break;
			case "float": val = parseFloat(event.target.value); break;
			case "bool": val = event.target.checked; break;
		}
		setNodeParamValue(fullname, type, val);
	});
}

function detectValueType(inputString) {
	if (inputString.toLowerCase() === 'true' || inputString.toLowerCase() === 'false') {
		return 'bool';
	}

	const integerRegex = /^-?[0-9]+$/;
	if (integerRegex.test(inputString)) {
		return 'int';
	}

	const floatRegex = /^-?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/;
	if (floatRegex.test(inputString)) {
		return 'float';
	}

	return 'string';
}

let nodeName = "";

async function listParameters(){
	if(nodeName == ""){
		infoLabel.innerText = "Select a valid node with reconfigurable parameters.";
		return;
	}

	const params = await getParamList();
	const nodeParams = params.filter((param) => param.startsWith(nodeName + "/"));

	infoLabel.innerText = "";
	paramBox.innerHTML = "";

	for (const element of nodeParams) {
		const value = await getParamValue(element);
		createParameterInput(element,value,detectValueType(value));
	}
}

async function setList(){

	let result = await getNodes();
	let nodelist = "";
	for (const node of result) {
		const details = await getNodeDetails(node);  
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

nodeSelector.addEventListener("change", (event)=>{
	nodeName = nodeSelector.value;
	listParameters();
});
icon.addEventListener("click", setList);

setList();

console.log("Reconfigure Widget Loaded {uniqueID}")