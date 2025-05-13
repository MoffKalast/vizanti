let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);

let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;

const savePathBox = document.getElementById("{uniqueID}_savepath");
const selectAllButton = document.getElementById('{uniqueID}_selectall');
const selectNoneButton = document.getElementById('{uniqueID}_selectnone');
const startButton = document.getElementById('{uniqueID}_toggle');

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

let path = "~/recording.bag";
let topic_list = new Set();
let active = false;

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	path = loaded_data.path;
	topic_list = new Set(loaded_data.topic_list);
}else{
	saveSettings();
}

savePathBox.value = path;

function saveSettings(){
	settings["{uniqueID}"] = {
		path: path,
		topic_list: Array.from(topic_list)
	}
	settings.save();
}

async function getRecordingStatus(topics, start, path) {
	const recordRosbagService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/bag/status",
		serviceType: "std_srvs/Trigger",
	});

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ topics, start, path });
		recordRosbagService.callService(request, (result) => {
			//console.log(result.message);
			setState(result.success);
			resolve(result.success);			
		}, (error) => {
			console.log(error);
			resolve(false);
		});
	});
}

function getCurrentDateTimeString() {
	const date = new Date();
	const year = date.getFullYear();
	const month = (date.getMonth() + 1).toString().padStart(2, '0');
	const day = date.getDate().toString().padStart(2, '0');
	const hours = date.getHours().toString().padStart(2, '0');
	const minutes = date.getMinutes().toString().padStart(2, '0');

	return `${year}-${month}-${day}-${hours}-${minutes}`;
}

async function recordRosbag(topics, start, path) {
	const recordRosbagService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/bag/setup",
		serviceType: "vizanti/RecordRosbag",
	});

	const timestamp = getCurrentDateTimeString();
    const pathArray = path.split("/");
    const fileName = pathArray.pop();
    const timepath = `${pathArray.join("/")}/${timestamp}-${fileName}`;

	return new Promise((resolve, reject) => {
		const request = new ROSLIB.ServiceRequest({ topics, start, path: timepath });
		recordRosbagService.callService(request, (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

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
	if(await confirm("Are you sure you want to start recording a bag?")){
		const result = await recordRosbag(Array.from(topic_list), true, path);
		console.log(result);
		setState(result.success);
		alert(result.message)
	}
}

async function stopRecording() {
	if(await confirm("Are you sure you want to stop recording?")){
		const result = await recordRosbag([], false, '');
		console.log(result);
		setState(!result.success);
		alert(result.message)
	}
}

startButton.addEventListener('click', async () => {
	if(active){
		stopRecording();
	}else{
		startRecording();
	}
	saveSettings();
});

selectAllButton.addEventListener('click', async () => {
	let result = await rosbridge.get_all_topics();

	result.topics.forEach((topic) => {
		if(!topic.includes("/vizanti/tf_consolidated")){
			topic_list.add(topic);
		}
	});
	
	updateTopics();
	saveSettings();
});

selectNoneButton.addEventListener('click', async () => {
	topic_list = new Set();	
	updateTopics();
	saveSettings();
});

savePathBox.addEventListener('input', async () => {
	path = savePathBox.value;
	saveSettings();
});

const topicsDiv = document.getElementById('{uniqueID}_topics');

async function updateTopics(){
	//recheck in case another client started a recording
	await getRecordingStatus();

	let result = await rosbridge.get_all_topics();

	topicsDiv.innerHTML = '';

	// Group topics by type
	let topicsByType = new Map();
	result.topics.forEach((topic, index) => {
		if(!topic.includes("/vizanti/tf_consolidated")){
			const type = result.types[index];
			if (topicsByType.has(type)) {
				topicsByType.get(type).push(topic);
			} else {
				topicsByType.set(type, [topic]);
			}
		}
	});

	// Create checkboxes for each group of topics
	topicsByType.forEach((topics, type) => {
		const brBefore = document.createElement('div');
		brBefore.className = 'spacer';
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

			const span = document.createElement('span');
			span.style.whiteSpace = "nowrap";
			span.appendChild(checkbox);
			span.appendChild(label);
			div.appendChild(span);

			const br = document.createElement('div');
			br.className = 'spacer';
			div.appendChild(br);

			if(checkbox.checked)
				div.style.display = 'block';
		});

		topicsDiv.appendChild(button);
		topicsDiv.appendChild(div);
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

console.log("Rosbag Widget Loaded {uniqueID}")