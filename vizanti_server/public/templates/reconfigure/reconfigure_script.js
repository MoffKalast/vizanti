let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);

let rosbridge = rosbridgeModule.rosbridge;

async function getNodeParameters(node) {
	const getNodeParametersService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/get_node_parameters",
		serviceType: "vizanti_msgs/srv/GetNodeParameters",
	});

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ node });
		getNodeParametersService.callService(request, (result) => {
			resolve(JSON.parse(result.parameters));
		}, (error) => {
			reject(error);
		});
	});
}

async function setNodeParameter(node, param, newValue) {
	const setParamClient = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: '/vizanti/set_node_parameter',
		serviceType: 'vizanti_msgs/srv/SetNodeParameter',
	});

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({
			 node: nodeName+"",
			 param: param.toLocaleString('en-US'),
			 value: newValue.toLocaleString('en-US')
		});
		setParamClient.callService(request, (response) => {
			console.log(`Parameter ${paramName} set to:`, newValue);
			resolve(response);
		}, (error) => {
			console.error(`Failed to call set_parameters service for ${nodeName}:`, error);
			reject(error);
		});
	});
}


function createParameterInput(fullname, defaultValue, type, element) {
	const name = fullname.replace(nodeName+"/","").replaceAll("_","_<wbr>");
	const id = "${uniqueID}_"+fullname;
	const arrowId = `${id}_arrow`;
	let inputElement;

	switch (type) {
		case "string":
			inputElement = `
				<label for="${id}"><i>string </i> ${name}:</label>
				<span><input id="${id}" type="text" value="${defaultValue}"><span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span></span>
				<br>`;
			break;
		case "int":
			inputElement = `
				<label for="${id}"><i>int </i> ${name}:</label>
				<span><input type="number" value="${defaultValue}" step="1" id="${id}"><span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span></span>
				<br>`;
			break;
		case "float":
			inputElement = `
				<label for="${id}"><i>float </i>${name}:</label>
				<span><input type="number" value="${defaultValue}" step="0.001" id="${id}"><span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span></span>
				<br>`;
			break;
		case "bool":
			inputElement = `
				<label for="${id}"><i>bool </i>${name}:</label>
				<span><input type="checkbox" id="${id}" ${defaultValue ? "checked" : ""}><span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span></span>
				<br>`;
			break;
		default:
			console.error("Invalid parameter type:", type);
			return;
	}

	element.insertAdjacentHTML("beforeend", inputElement);

	document.getElementById(id).addEventListener("change", (event) => {
		let val;

		switch(type){
			case "string": val = event.target.value; break;
			case "int": val = parseInt(event.target.value); break;
			case "float": val = parseFloat(event.target.value); break;
			case "bool": val = event.target.checked; break;
		}
		setNodeParameter(nodeName, fullname, val);

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

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const nodeSelector = document.getElementById("{uniqueID}_node");
const loaderSpinner = document.getElementById("{uniqueID}_loader");
const paramBox = document.getElementById("{uniqueID}_params");
const refreshButton = document.getElementById("{uniqueID}_refresh");

let nodeName = "";
let cached_params = {};

function addParams(map, element){
	for (const [key,value] of Object.entries(map)) {
		if (typeof value == "object") {
			const detailsElement = document.createElement('details');
			detailsElement.innerHTML = "<summary>"+key+"</summary>";
			element.appendChild(detailsElement);
			addParams(value, detailsElement);
		}else{
			createParameterInput(key,value,detectValueType(value), element);
		}
	}
}

async function listParameters(){
	if(nodeName == ""){
		return;
	}
	loaderSpinner.style.display = "block";
	paramBox.innerHTML = "";

	if(cached_params[nodeName]){
		addParams(cached_params[nodeName], paramBox);
	}
	
	const data = await getNodeParameters(nodeName);
	paramBox.innerHTML = "";
	addParams(data.ros__parameters, paramBox);
	cached_params[nodeName] = data.ros__parameters;

	loaderSpinner.style.display = "none";
}

async function setNodeList(){
	let results = await rosbridge.get_all_nodes();
	let nodelist = "";
	let nodes = [];
	for (const node of results.nodes) {
		if(!node.includes("vizanti")){
			nodelist += "<option value='"+node+"'>"+node+"</option>"
			nodes.push(node);
		}
	}
	nodeSelector.innerHTML = nodelist;
	loaderSpinner.style.display = "none";

	nodeName = nodeSelector.value;
	listParameters();
}

nodeSelector.addEventListener("change", (event)=>{
	nodeName = nodeSelector.value;
	listParameters();
});

refreshButton.addEventListener("click", listParameters);
icon.addEventListener("click", setNodeList);

console.log("Reconfigure Widget Loaded {uniqueID}");