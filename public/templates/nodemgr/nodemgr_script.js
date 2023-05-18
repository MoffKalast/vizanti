import { rosbridge } from '/js/modules/rosbridge.js';
  
async function killNode(name) {
    const recordRosbagService = new ROSLIB.Service({
        ros: rosbridge.ros,
        name: "/outdooros/node/kill",
        serviceType: "outdooros/ManageNode",
    });

    return new Promise((resolve, reject) => {
        const request = new ROSLIB.ServiceRequest({ node: name });
        recordRosbagService.callService(request, (result) => {
            resolve(result);
        }, (error) => {
            reject(error);
			alert(error);
        });
    });
}

async function nodeInfo(name) {
    const recordRosbagService = new ROSLIB.Service({
        ros: rosbridge.ros,
        name: "/outdooros/node/info",
        serviceType: "outdooros/ManageNode",
    });

    return new Promise((resolve, reject) => {
        const request = new ROSLIB.ServiceRequest({ node: name });
        recordRosbagService.callService(request, (result) => {
            resolve(result.message);
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

const typeBox = document.getElementById('{uniqueID}_type');
const packageBox = document.getElementById('{uniqueID}_package');
const nameBox = document.getElementById('{uniqueID}_name');
const executeButton = document.getElementById('{uniqueID}_execute');

let currentNode = "";

killButton.addEventListener('click', async () => {
	if(await confirm("Do you really want to kill "+currentNode+"?")){
		console.log("Killing "+currentNode);
		await killNode(currentNode);
		closeModal("{uniqueID}_contextmodal");
		updateNodeList();
	}
});

executeButton.addEventListener('click', async () => {
	if(await confirm("Do you really want to run '"+typeBox.value+" "+packageBox.value+" "+nameBox.value+"'?")){
		console.log("Executing "+typeBox.value+" "+packageBox.value+" "+nameBox.value);
	}
});

async function updateNodeList(){
	let result = await rosbridge.get_all_nodes();

	nodeDiv.innerHTML = '';
	result.nodes.forEach(node => {

		if(node.includes("outdooros"))
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

icon.addEventListener("click", updateNodeList);

updateNodeList();

console.log("Node Manager Loaded {uniqueID}")