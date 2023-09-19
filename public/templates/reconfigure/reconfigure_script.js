let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);

let rosbridge = rosbridgeModule.rosbridge;

async function getDynamicReconfigureNodes() {
	const getNodesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/get_dynamic_reconfigure_nodes",
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
		name: "/vizanti/get_node_parameters",
		serviceType: "vizanti/GetNodeParameters",
	});

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ node });
		getNodeParametersService.callService(request, (result) => {
			let parsedParams = JSON.parse(convertAlmostJsonToValidJson(result.parameters));
			delete parsedParams.groups;
			resolve(parsedParams);
		}, (error) => {
			reject(error);
		});
	});
}

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const nodeSelector = document.getElementById("{uniqueID}_node");
const loaderSpinner = document.getElementById("{uniqueID}_loader");
const paramBox = document.getElementById("{uniqueID}_params");
const refreshButton = document.getElementById("{uniqueID}_refresh");

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

function createParameterInput(fullname, defaultValue, type) {
	const name = fullname.replace(nodeName+"/","").replaceAll("_","_<wbr>");
	const id = "${uniqueID}_"+fullname;
	const arrowId = `${id}_arrow`;
	let inputElement;

	switch (type) {
		case "string":
			inputElement = `
				<label for="${id}"><i>string </i> ${name}:</label>
				<input id="${id}" type="text" value="${defaultValue}"><span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
				<br>`;
			break;
		case "int":
			inputElement = `
				<label for="${id}"><i>int </i> ${name}:</label>
				<input type="number" value="${defaultValue}" step="1" id="${id}"><span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
				<br>`;
			break;
		case "float":
			inputElement = `
				<label for="${id}"><i>float </i>${name}:</label>
				<input type="number" value="${defaultValue}" step="0.001" id="${id}"><span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
				<br>`;
			break;
		case "bool":
			inputElement = `
				<label for="${id}"><i>bool </i>${name}:</label>
				<input type="checkbox" id="${id}" ${defaultValue ? "checked" : ""}><span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
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

		const arrowElement = document.getElementById(arrowId);
		arrowElement.style.visibility = "visible";
		arrowElement.style.animation = "none";
		// Force reflow to make the new animation take effect
		arrowElement.offsetHeight;
		arrowElement.style.animation = null;

		arrowElement.addEventListener('animationend', () => {
			arrowElement.style.visibility = "hidden";
		}, {once: true});
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
let cached_params = undefined;

async function listParameters(){
	if(nodeName == "" || !cached_params[nodeName]){
		return;
	}
	loaderSpinner.style.display = "block";

	paramBox.innerHTML = "";
	for (const [key,value] of Object.entries(cached_params[nodeName])) {
		createParameterInput(key,value,detectValueType(value));
	}
	loaderSpinner.style.display = "none";
}

async function getAll(results){
	loaderSpinner.style.display = "block";
	cached_params = {};
	for (const node of results) {
		cached_params[node] = await getNodeParameters(node);
		if(node == nodeName){
			listParameters();
		}
	}
	loaderSpinner.style.display = "none";
}

async function setNodeList(){
	let results = await getDynamicReconfigureNodes();
	let nodelist = "";
	for (const node of results) {
		nodelist += "<option value='"+node+"'>"+node+"</option>"
	}
	nodeSelector.innerHTML = nodelist;

	if(nodeName == "")
		nodeName = nodeSelector.value;
	else if(results.includes(nodeName)){
		nodeSelector.value = nodeName;
	}

	if(!cached_params){
		await getAll(results);
	}
}

nodeSelector.addEventListener("change", (event)=>{
	nodeName = nodeSelector.value;
	listParameters();
});

icon.addEventListener("click", setNodeList);

refreshButton.addEventListener("click", async (event)=>{
	loaderSpinner.style.display = "block";
	cached_params[nodeName] = await getNodeParameters(nodeName);
	listParameters();
	loaderSpinner.style.display = "none";
});

await setNodeList();

console.log("Reconfigure Widget Loaded {uniqueID}")