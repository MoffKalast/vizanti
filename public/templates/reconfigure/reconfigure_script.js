let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);

let rosbridge = rosbridgeModule.rosbridge;

async function getDynamicReconfigureNodes() {
	const getNodesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/get_dynamic_reconfigure_nodes",
		serviceType: "std_srvs/Trigger",
	});

	return new Promise((resolve, reject) => {
		try {
			getNodesService.callService(new ROSLIB.ServiceRequest(), (result) => {
				resolve(JSON.parse(result.message));
			});
		} catch (error) {
			reject(error);
		}
	});
}

async function getNodeParameters(node) {
	const getNodeParametersService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/get_node_parameters",
		serviceType: "vizanti/GetNodeParameters",
	});

	return new Promise((resolve, reject) => {
		try {
			const request = new ROSLIB.ServiceRequest({ node });
			getNodeParametersService.callService(request, (result) => {
				resolve(JSON.parse(result.parameters));
			});
		} catch (error) {
			reject(error);
		}
	});
}

async function getNodeParameterInfo(node) {
	const getNodeParametersService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/get_node_parameter_info",
		serviceType: "vizanti/GetNodeParameters",
	});

	return new Promise((resolve, reject) => {
		try {
			const request = new ROSLIB.ServiceRequest({ node });
			getNodeParametersService.callService(request, (result) => {
				resolve(JSON.parse(result.parameters));
			});
		} catch (error) {
			reject(error);
		}
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
		case "str": valueConfig.strs.push({ name: paramName, value: newValue }); break;
		case "int": valueConfig.ints.push({ name: paramName, value: newValue }); break;
		case "double": valueConfig.doubles.push({ name: paramName, value: newValue }); break;
		case "bool": valueConfig.bools.push({ name: paramName, value: newValue }); break;
		default: return Promise.reject(`Invalid parameter value type: ${type}`);
	}

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ config: valueConfig });
		setParamClient.callService(request, (response) => {
			//console.log(`Parameter ${paramName} set to:`, newValue);
			resolve(response);
		}, (error) => {
			console.error(`Failed to call set_parameters service for ${nodeName}:`, error);
			reject(error);
		});
	});
}

function createParameterInput(fullname, defaultValue, info) {
	const min = info.min;
	const max = info.max;
	const type = info.type;
	const desc = info.description;
	const is_enum = info.enum !== undefined;
	
	const name = fullname.replace(nodeName+"/","").replaceAll("_","_<wbr>");
	const id = "${uniqueID}_"+fullname;
	const arrowId = `${id}_arrow`;
	let inputElement;

	if (is_enum) {
		const optionsHtml = info.enum
			.map(enumItem => {
				const isSelected = enumItem.value === defaultValue ? 'selected' : '';
				return `<option value="${enumItem.value}" ${isSelected}>(${enumItem.value}) ${enumItem.description}</option>`;
			})
			.join('');
		inputElement = `
			<label for="${id}"><i>enum </i> ${name}:</label>
			<div class="input-group">
				<select id="${id}">
					${optionsHtml}
				</select>
				<span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
			</div>
			<p class="minicomment">${desc}</p>
			<div class="spacer"></div>`;
	} else {
		switch (type) {
			case "str":
				inputElement = `
					<label for="${id}"><i>string </i> ${name}:</label>
					<div class="input-group">
						<input style="width: 30%;" id="${id}" type="text" value="${defaultValue}">
						<span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
					</div>
					<p class="minicomment">${desc}</p>
					<div class="spacer"></div>`;
				break;
			case "int":
				inputElement = `
					<label for="${id}"><i>int </i> ${name}:</label>
					<div class="input-group">
						<input type="number" value="${defaultValue}" step="1" min="${min}" max="${max}" id="${id}">
						<span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
					</div>
					<p class="minicomment">${desc}</p>
					<div class="spacer"></div>`;
				break;
			case "double":
			case "float":
				inputElement = `
					<label for="${id}"><i>double </i>${name}:</label>
					<div class="input-group">
						<input type="number" value="${defaultValue}" step="0.001" min="${min}" max="${max}" id="${id}">
						<span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
					</div>
					<p class="minicomment">${desc}</p>
					<div class="spacer"></div>`;
				break;
			case "bool":
				inputElement = `
					<label for="${id}"><i>bool </i>${name}:</label>
					<div class="input-group">
						<input type="checkbox" id="${id}" ${defaultValue ? "checked" : ""}>
						<span id="${arrowId}" class="arrow" style="visibility: hidden;">➡</span>
					</div>
					<p class="minicomment">${desc}</p>
					<div class="spacer"></div>`;
				break;
			default:
				console.error("Invalid parameter type:", type);
				return;
		}
	}

	paramBox.insertAdjacentHTML("beforeend", inputElement);

	setTimeout(() => {
		document.getElementById(id).addEventListener("change", (event) => {
			let processedValue;
	
			if (is_enum) {
				// For enums, determine the actual type from the enum value and parse accordingly
				const selectedEnumValue = event.target.value;
				const enumItem = info.enum.find(item => item.value == selectedEnumValue);
				
				if (enumItem) {
					processedValue = enumItem.value;
				} else {
					console.error("Selected enum value not found:", selectedEnumValue);
					return;
				}
			} else {
				switch(type) {
					case "str": 
						processedValue = event.target.value; 
						break;
					case "int": 
						processedValue = parseInt(event.target.value); 
						break;
					case "double":
					case "float": 
						processedValue = parseFloat(event.target.value); 
						break;
					case "bool": 
						processedValue = event.target.checked; 
						break;
					default:
						console.error("Unsupported type for value processing:", type);
						return;
				}
			}
			
			setNodeParamValue(fullname, type, processedValue);
	
			const arrowElement = document.getElementById(arrowId);
			arrowElement.style.visibility = "visible";
			arrowElement.style.animation = "none";
			arrowElement.offsetHeight; // Force reflow
			arrowElement.style.animation = null;
	
			arrowElement.addEventListener('animationend', () => {
				arrowElement.style.visibility = "hidden";
			}, {once: true});
		});
	}, 1);
}

let nodeName = "";
let cached_info = {};
let cached_params = {};
let is_loaded = false;

async function listParameters(){
	if (nodeName === "") {
		return;
	}

	loaderSpinner.style.display = "block";
	paramBox.innerHTML = "";

	if(!cached_params[nodeName]){
		setTimeout(listParameters,1000);
		return;
	}

	for (const [index, entry] of Object.entries(cached_params[nodeName])) {
		let [key,value] = entry;
		createParameterInput(key,value,cached_info[nodeName][key]);
	}
	loaderSpinner.style.display = "none";
}

async function getAll(results){
	loaderSpinner.style.display = "block";
	cached_params = {};

	const [, params] = await Promise.all([
		(async () => {
			if(cached_info[nodeName] == undefined){
				cached_info[nodeName] = await getNodeParameterInfo(nodeName);
			}
		})(),
		getNodeParameters(nodeName)
	]);
	
	cached_params[nodeName] = params;
	listParameters();

	for (const node of results) {
		if(node != nodeName){
			if(cached_info[node] == undefined){
				cached_info[node] = await getNodeParameterInfo(node);
			}
			cached_params[node] = await getNodeParameters(node);
		}
	}
	loaderSpinner.style.display = "none";
}

let loading_nodes = false;

async function setNodeList(){
	if(loading_nodes){
		return;
	}

	loading_nodes = true;
	
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

	await getAll(results);
	loading_nodes = false;
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