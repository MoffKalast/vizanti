import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let path = "~/recording.bag";
let topic_list = new Set();

let active = await getRecordingStatus();

// Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	path = loaded_data.path;
	topic_list = new Set(loaded_data.topic_list);
}

function saveSettings(){
	settings["{uniqueID}"] = {
		path: savePathBox.value,
		topic_list: Array.from(topic_list)
	}
	settings.save();
}

async function getRecordingStatus(topics, start, path) {
	const recordRosbagService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/outdooros/bag/status",
		serviceType: "std_srvs/Trigger",
	});

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ topics, start, path });
		recordRosbagService.callService(request, (result) => {
			console.log(result.message);
			resolve(result.success);
		}, (error) => {
			console.log(error);
			resolve(false);
		});
	});
}

async function recordRosbag(topics, start, path) {
	const recordRosbagService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/outdooros/bag/setup",
		serviceType: "outdooros/RecordRosbag",
	});

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ topics, start, path });
		recordRosbagService.callService(request, (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

const savePathBox = document.getElementById("{uniqueID}_savepath");
const selectAllButton = document.getElementById('{uniqueID}_selectall');
const selectNoneButton = document.getElementById('{uniqueID}_selectnone');
const startButton = document.getElementById('{uniqueID}_toggle');

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

function setState(state){

	if(state){
		startButton.style.backgroundColor = "#e14044ff";
		startButton.style.color = "black";
		startButton.innerText = "Stop recording";
		icon.src = "assets/rosbag_active.svg";
	}else{
		startButton.style.backgroundColor = "rgb(38, 104, 31)";
		startButton.style.color = "white";
		startButton.innerText = "Start recording";
		icon.src = "assets/rosbag.svg";
	}

	active = state;
}

async function startRecording() {
	const result = await recordRosbag(Array.from(topic_list), true, path);
	console.log(result);
	setState(result.success);

	if(!result.success)
		alert(result.message)
}

async function stopRecording() {
	const result = await recordRosbag([], false, '');
	console.log(result);
	setState(!result.success);

	if(!result.success)
		alert(result.message)
}

startButton.addEventListener('click', async () => {
	if(active){
		stopRecording();
	}else{
		startRecording();
	}
});

selectAllButton.addEventListener('click', async () => {
	let result = await rosbridge.get_all_topics();

	result.topics.forEach((topic) => {
		topic_list.add(topic);
	});
	
	updateTopics();
});

selectNoneButton.addEventListener('click', async () => {
	topic_list = new Set();	
	updateTopics();
});

savePathBox.addEventListener('click', async () => {
	saveSettings();
});

const topicsDiv = document.getElementById('{uniqueID}_topics');

async function updateTopics(){
	let result = await rosbridge.get_all_topics();

	topicsDiv.innerHTML = '';

	// Group topics by type
	let topicsByType = new Map();
	result.topics.forEach((topic, index) => {
		const type = result.types[index];
		if (topicsByType.has(type)) {
			topicsByType.get(type).push(topic);
		} else {
			topicsByType.set(type, [topic]);
		}
	});

	// Create checkboxes for each group of topics
	topicsByType.forEach((topics, type) => {
		const brBefore = document.createElement('br');
		topicsDiv.appendChild(brBefore);

		const button = document.createElement('button');
		button.textContent = type;
		button.className = 'collapsible';

		const div = document.createElement('div');
		div.className = 'content';
	   	div.style.display = 'none';  // Initially hide the content

		topics.forEach(topic => {
			const checkbox = document.createElement('input');
			checkbox.type = 'checkbox';
			checkbox.id = `${uniqueID}_${topic}`;
			checkbox.checked = topic_list.has(topic);
			checkbox.addEventListener('change', (event) => {
				if(checkbox.checked){
					topic_list.add(topic);
				}else{
					topic_list.delete(topic);
				}
				saveSettings();
			});

			const label = document.createElement('label');
			label.textContent = ` ${topic}`;

			const br = document.createElement('br');

			div.appendChild(checkbox);
			div.appendChild(label);
			div.appendChild(br);

			if(checkbox.checked)
				div.style.display = 'block';
		});

		topicsDiv.appendChild(button);
		topicsDiv.appendChild(div);

		const brAfter = document.createElement('br');
		topicsDiv.appendChild(brAfter);
	});

	// Add event listener to all collapsible buttons
	let coll = document.getElementsByClassName('collapsible');
	for (let i = 0; i < coll.length; i++) {
		coll[i].addEventListener('click', function() {
			this.classList.toggle('active');
			let content = this.nextElementSibling;
			if (content.style.display === 'block') {
				content.style.display = 'none';
			} else {
				content.style.display = 'block';
			}
		});
	}
}

icon.addEventListener("click", updateTopics);
updateTopics();
setState(active);

console.log("Rosbag Widget Loaded {uniqueID}")