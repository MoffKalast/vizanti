let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);

let rosbridge = rosbridgeModule.rosbridge;

let packages = [];

async function runRosWTF() {
	const wtfService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/roswtf",
		serviceType: "std_srvs/Trigger",
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
async function getExecutables(pkg_name) {
	const getExecutablesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/list_executables",
		serviceType: "vizanti/ListExecutables",
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
		serviceType: "vizanti/ListPackages",
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
		serviceType: "vizanti/ManageNode",
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
		serviceType: "vizanti/ManageNode",
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
		serviceType: "vizanti/ManageNode",
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
	let result = await rosbridge.get_all_nodes();

	nodeDiv.innerHTML = '';
	result.nodes.forEach(node => {

		if(node.includes("vizanti"))
			return;

		const nodeBox = document.createElement('div');
		nodeBox.className = 'node-box';
	
		const nodeName = document.createElement('span');
		nodeName.textContent = node;
	
		nodeBox.addEventListener('click', async () => {
			infoText.innerText = "Waiting for rosnode info...";

			currentNode = node;
			contextTitle.innerText = node;
			openModal("{uniqueID}_contextmodal");

			infoText.innerText = await nodeInfo(node);
		});
	
		nodeBox.appendChild(nodeName);
		nodeDiv.appendChild(nodeBox);
	});
}

// roswtf

const wtfText = document.getElementById('{uniqueID}_roswtf_data');
const wtfButton = document.getElementById('{uniqueID}_roswtf');

wtfButton.addEventListener('click', async () => {
	wtfText.innerText = "Waiting for roswtf report (might take several seconds)...";
	openModal("{uniqueID}_roswtfmodal");
	wtfText.innerText = await runRosWTF();
	console.log(wtfText.innerText)
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


		if(nameBox.value.endsWith(".launch")){
			typeBox.value = "roslaunch";
		}else{
			typeBox.value = "rosrun";
		}
	}
});

nameBox.addEventListener('change', async function(e) {
	if(nameBox.value.endsWith(".launch")){
		typeBox.value = "roslaunch";
	}else{
		typeBox.value = "rosrun";
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