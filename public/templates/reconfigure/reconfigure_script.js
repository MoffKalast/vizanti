import { rosbridge } from '/js/modules/rosbridge.js';

async function getDynamicReconfigureNodes() {
	const getNodesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/outdooros/get_dynamic_reconfigure_nodes",
		serviceType: "std_srvs/Trigger",
	});

	return new Promise((resolve, reject) => {
		getNodesService.callService(new ROSLIB.ServiceRequest(), (result) => {
			resolve(result.message.split("\n"));
		}, (error) => {
			reject(error);
		});
	});
}

async function getNodeParameters(node) {
	const getNodeParametersService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/outdooros/get_node_parameters",
		serviceType: "outdooros/GetNodeParameters",
	});

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ node });
		getNodeParametersService.callService(request, (result) => {
			resolve(result.parameters);
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
const loaderSpinner = document.getElementById("{uniqueID}_loader");
const paramBox = document.getElementById("{uniqueID}_params");

function createParameterInput(fullname, defaultValue, type) {
	const name = fullname.replace(nodeName+"/","").replaceAll("_","_<wbr>");
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

function detectValueType(value) {
	if (typeof value == "boolean") {
		return "bool";
	}

	//this is incorrectly detecting floats as integers, shelfed for now
	//if (Number.isInteger(value)) {
		//return "int";
	//}

	if (isNaN(value)) {
		return "string";
	}

	return "float";
}

function convertAlmostJsonToValidJson(almostJson) {
	const validJson = almostJson.replace(/'/g, '"');
	return validJson.replace(/(True|False)/g, (match) => {
		return match.toLowerCase();
	});
}

let nodeName = "";

async function listParameters(){
	if(nodeName == ""){
		infoLabel.innerText = "Select a valid node with reconfigurable parameters.";
		return;
	}
	infoLabel.innerText = "";
	paramBox.innerHTML = "";
	loaderSpinner.style.display = "block";

	const params = await getNodeParameters(nodeName);	
	let parsedParams = JSON.parse(convertAlmostJsonToValidJson(params));
	delete parsedParams.groups;

	infoLabel.innerText = "";
	paramBox.innerHTML = "";

	for (const [key,value] of Object.entries(parsedParams)) {
		createParameterInput(key,value,detectValueType(value));
	}
	loaderSpinner.style.display = "none";
}

async function setList(){

	paramBox.innerHTML = "";
	loaderSpinner.style.display = "block";
	let result = await getDynamicReconfigureNodes();

	let nodelist = "";
	for (const node of result) {
		nodelist += "<option value='"+node+"'>"+node+"</option>"
	}

	loaderSpinner.style.display = "none";
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