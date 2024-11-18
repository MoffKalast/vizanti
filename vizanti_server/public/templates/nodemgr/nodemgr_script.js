let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);

let rosbridge = rosbridgeModule.rosbridge;
let packages = [];

async function runRosWTF() {
	const wtfService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/roswtf",
		serviceType: "std_srvs/srv/Trigger",
	});

	return new Promise((resolve, reject) => {
		wtfService.callService(new ROSLIB.ServiceRequest(), (result) => {
			// The ASCII code for escape is 27, represented in hexadecimal as \x1B
			resolve(result.message.replace(/\x1B\[1m/g, '').replace(/\x1B\[0m/g, '').replace(/ /g, '\u00a0'));
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

//https://docs.ros2.org/foxy/api/lifecycle_msgs/msg/State.html
const LIFECYCLES = {
	0: "Unknown",
	1: "Unconfigured",
	2: "Inactive",
	3: "Active",
	4: "Finalized",
	10: "Configuring",
	11: "Cleaning Up",
	12: "Shutting Down",
	13: "Activating",
	14: "Deactivating",
	15: "Error Processing"
};

const LIFECYCLE_COLORS = {
	0: "lightgray", //Unknown
	1: "#c296d6", //Unconfigured
	2: "#9995c9", //Inactive
	3: "#5fd980", //Active
	4: "#5997ba", //Finalized
	10: "#f8ff30", //Configuring
	11: "#ffcf30", //Cleaningup
	12: "#ffa530", //Shutting Down
	13: "#7be1e3", //Activating
	14: "#e67f5a", //Deactivating
	15: "#cc4747" //Error Processing
};

let lifecycles = {};

async function getLifecycles() {
	const getExecutablesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/list_lifecycle_nodes",
		serviceType: "vizanti_msgs/srv/ListLifecycles",
	});

	return new Promise((resolve, reject) => {
		getExecutablesService.callService(new ROSLIB.ServiceRequest(), (result) => {
			const map = {};
			for(let i = 0; i < result.nodes.length; i++){
				const name = result.nodes[i];
				const state = result.states[i];
				map[name] = state;
			}
			resolve(map);
		}, (error) => {
			reject(error);
		});
	});
}

async function getExecutables(pkg_name) {
	const getExecutablesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/list_executables",
		serviceType: "vizanti_msgs/srv/ListExecutables",
	});

	return new Promise((resolve, reject) => {
		getExecutablesService.callService(new ROSLIB.ServiceRequest({package : pkg_name}), (result) => {
			resolve(result.executables);
		}, (error) => {
			reject(error);
		});
	});
}

async function getPackages() {
	const getPackagesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/list_packages",
		serviceType: "vizanti_msgs/srv/ListPackages",
	});

	return new Promise((resolve, reject) => {
		getPackagesService.callService(new ROSLIB.ServiceRequest(), (result) => {
			resolve(result.packages);
		}, (error) => {
			reject(error);
		});
	});
}

async function startNode(command) {
	const startService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/node/start",
		serviceType: "vizanti_msgs/srv/ManageNode",
	});

	return new Promise((resolve, reject) => {
		startService.callService(new ROSLIB.ServiceRequest({ node: command }), (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
		});
	});
}

async function killNode(name) {
	const killService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/node/kill",
		serviceType: "vizanti_msgs/srv/ManageNode",
	});

	return new Promise((resolve, reject) => {
		killService.callService(new ROSLIB.ServiceRequest({ node: name }), (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

async function nodeInfo(name) {
	const infoService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/node/info",
		serviceType: "vizanti_msgs/srv/ManageNode",
	});

	return new Promise((resolve, reject) => {
		infoService.callService(new ROSLIB.ServiceRequest({ node: name }), (result) => {
			resolve(result.message.replace(/ /g, '\u00a0'));
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const nodeDiv = document.getElementById('{uniqueID}_nodelist');

const contextTitle = document.getElementById('{uniqueID}_context_title');
const killButton = document.getElementById('{uniqueID}_nodekill');
const infoText = document.getElementById('{uniqueID}_rosnode_info');
const lifecycleText = document.getElementById('{uniqueID}_rosnode_lifecycle');

let currentNode = "";

killButton.addEventListener('click', async () => {
	if(await confirm("Do you really want to kill "+currentNode+"?")){
		console.log("Killing "+currentNode);
		await killNode(currentNode);
		closeModal("{uniqueID}_contextmodal");
		updateNodeList();
	}
});

async function updateNodeList(){

	function renderLifeCycle(element, node){
		const state = lifecycles[node];
		element.textContent = "("+LIFECYCLES[state]+")";
		element.style.color = LIFECYCLE_COLORS[state];
		element.parentElement.style.borderLeft = "6px solid "+LIFECYCLE_COLORS[state];
	}

	let result = await rosbridge.get_all_nodes();

	nodeDiv.innerHTML = '';
	result.nodes.forEach(node => {

		if(node.includes("vizanti"))
			return;

		const nodeBox = document.createElement('div');
		nodeBox.className = 'node-box';
	
		const nodeName = document.createElement('span');
		nodeName.textContent = node;

		const nodeStatus = document.createElement('span');
		nodeStatus.style.marginLeft = "15px";
		nodeStatus.style.marginRight = "15px";
		nodeStatus.id = "{uniqueID}_nodestatus_"+node;
	
		nodeBox.addEventListener('click', async () => {

			if(node in lifecycles){
				const state = lifecycles[node];
				lifecycleText.innerText = "Lifecycle status: "+LIFECYCLES[state];
				lifecycleText.style.color = LIFECYCLE_COLORS[state];
			}else{
				lifecycleText.innerText = "Lifecycle status: Not a lifecycle node.";
				lifecycleText.style.color = "lightgray";
			}

			infoText.innerText = "Waiting for rosnode info...";

			currentNode = node;
			contextTitle.innerText = node;
			openModal("{uniqueID}_contextmodal");

			infoText.innerText = await nodeInfo(node);
		});
	
		nodeBox.appendChild(nodeName);
		nodeBox.appendChild(nodeStatus);
		nodeDiv.appendChild(nodeBox);

		if(node in lifecycles)
			renderLifeCycle(nodeStatus, node);

	});

	lifecycles = await getLifecycles();
	for (const [name, state] of Object.entries(lifecycles)) {
		const element = document.getElementById("{uniqueID}_nodestatus_"+name);
		renderLifeCycle(element, name);
	}
}

// roswtf

const wtfText = document.getElementById('{uniqueID}_roswtf_data');
const wtfButton = document.getElementById('{uniqueID}_roswtf');

wtfButton.addEventListener('click', async () => {
	wtfText.innerText = "Waiting for roswtf report (might take several seconds)...";
	openModal("{uniqueID}_roswtfmodal");
	wtfText.innerText = await runRosWTF();
});


// package picking
const executeButton = document.getElementById('{uniqueID}_execute');

const typeBox = document.getElementById('{uniqueID}_type');
const packageBox = document.getElementById('{uniqueID}_package');
const packageDataList = document.getElementById('{uniqueID}_package_datalist');
const nameBox = document.getElementById('{uniqueID}_name');

packageBox.addEventListener('change', async function(e) {
	const val = packageBox.value;

	if(packages.includes(val)) {
		let executables = await getExecutables(val);
		let nodelist = "";
		for (const exe of executables) {
			nodelist += "<option value='"+exe+"'>"+exe+"</option>"
		}
		nameBox.innerHTML = nodelist;

		if(nameBox.value.includes("launch")){
			typeBox.value = "ros2 launch";
		}else{
			typeBox.value = "ros2 run";
		}
	}
});

nameBox.addEventListener('change', async function(e) {
	if(nameBox.value.includes("launch")){
		typeBox.value = "ros2 launch";
	}else{
		typeBox.value = "ros2 run";
	}
});

async function updatePackageList(){
	packages = await getPackages();
	packages.forEach(pkg => {
		let option = document.createElement('option');
		option.value = pkg;
		packageDataList.appendChild(option);
	});
}

executeButton.addEventListener('click', async () => {
	if(await confirm("Do you really want to run '"+typeBox.value+" "+packageBox.value+" "+nameBox.value+"'?")){
		console.log("Executing "+typeBox.value+" "+packageBox.value+" "+nameBox.value);
		let response = await startNode(typeBox.value+" "+packageBox.value+" "+nameBox.value);
		alert(response.message);
		setTimeout(updateNodeList,1000);
	}
});

icon.addEventListener("click", updateNodeList);

updateNodeList();
updatePackageList();

console.log("Node Manager Loaded {uniqueID}")