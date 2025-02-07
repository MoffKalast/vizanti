let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let Status = StatusModule.Status;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const throttle = document.getElementById('{uniqueID}_throttle');
throttle.addEventListener("input", (event) =>{
	saveSettings();
	connect();
});

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const info_div = document.getElementById("{uniqueID}_info_display");
const live_data_div = document.getElementById("{uniqueID}_live_data_display");

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	throttle.value = loaded_data.throttle;
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		throttle: throttle.value
	}
	settings.save();
}

let listener = undefined;
let topicobj = undefined;
let topic_type = await getTopicType();

let prevtopic = undefined;

function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(topicobj !== undefined){
		topicobj.unsubscribe(listener);
	}

	topicobj = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : topic_type,
		throttle_rate: parseInt(throttle.value)
	});

	status.setWarn("No data received.");

	if(prevtopic != topic){
		//clear data when the topic changes
		live_data_div.innerHTML = '<p>Waiting for data...</p>';
		info_div.innerHTML = '<p>Waiting for data...</p>';
		prevtopic = topic;
	}

	listener = topicobj.subscribe(async (msg) => {

		function createNestedDisplay(obj, indent = 0) {
			const container = document.createElement('div');
			const indentSize = 20; // pixels per level of indentation
			const maxStringLength = 200; // maximum length for string values
			const maxArrayLength = 50; // maximum number of array items to display
			
			// Helper function to truncate strings
			const truncateString = (str) => {
				if (typeof str !== 'string') return str;
				if (str.length <= maxStringLength) return str;
				return str.substring(0, maxStringLength) + '... [truncated]';
			};
			
			Object.entries(obj).forEach(([key, value]) => {
				const p = document.createElement('p');
				p.style.marginLeft = `${indent * indentSize}px`;
				p.style.marginTop = '2px';
				p.style.marginBottom = '2px';
				
				if (Array.isArray(value)) {
					// Handle arrays
					p.textContent = `${key}: Array(${value.length})`;
					container.appendChild(p);
					
					// Truncate array if too long
					const displayArray = value.length > maxArrayLength 
						? value.slice(0, maxArrayLength) 
						: value;
					
					displayArray.forEach((item, index) => {
						const arrayItem = document.createElement('p');
						arrayItem.style.marginLeft = `${(indent + 1) * indentSize}px`;
						arrayItem.style.marginTop = '2px';
						arrayItem.style.marginBottom = '2px';
						
						if (typeof item === 'object' && item !== null) {
							// Recursive call for objects within arrays
							const nestedContent = createNestedDisplay(item, indent + 2);
							container.appendChild(nestedContent);
						} else {
							arrayItem.textContent = `[${index}]: ${truncateString(item)}`;
							container.appendChild(arrayItem);
						}
					});
					
					if (value.length > maxArrayLength) {
						const omitted = document.createElement('p');
						omitted.style.marginLeft = `${(indent + 1) * indentSize}px`;
						omitted.style.marginTop = '2px';
						omitted.style.marginBottom = '2px';
						omitted.textContent = `... ${value.length - maxArrayLength} more items`;
						container.appendChild(omitted);
					}
					
				} else if (typeof value === 'object' && value !== null) {
					// Handle nested objects
					p.textContent = `${key}:`;
					container.appendChild(p);
					const nestedContent = createNestedDisplay(value, indent + 1);
					container.appendChild(nestedContent);
					
				} else {
					// Handle primitive values
					p.textContent = `${key}: ${truncateString(value)}`;
					container.appendChild(p);
				}
			});
			
			return container;
		}
		
		live_data_div.innerHTML = '';
		live_data_div.appendChild(createNestedDisplay(msg));

		let results = await rosbridge.get_topic_publishers_and_subscribers(topic);
		info_div.innerHTML = '';
		info_div.appendChild(createNestedDisplay(results));
		
		status.setOK();
	});

	saveSettings();
}

async function getTopicType(){
	let results = await rosbridge.get_all_topics();
	for (let i = 0; i < results.topics.length; i++) {
		if(results.topics[i] == topic)
			return results.types[i]
	}
	return undefined;
}

async function loadTopics(){
	let results = await rosbridge.get_all_topics();

	let topiclist = "";
	for (let i = 0; i < results.topics.length; i++) {
		topiclist += `<option value='${results.topics[i]}'>${results.topics[i]} (${results.types[i]})</option>`;
	}
	selectionbox.innerHTML = topiclist

	if(topic == ""){
		topic = selectionbox.value;
		topic_type = getTopicType();
	}else{
		if(results.topics.includes(topic)){
			selectionbox.value = topic;
		}else{
			topiclist += "<option value='"+topic+"'>"+topic+"</option>"
			selectionbox.innerHTML = topiclist
			selectionbox.value = topic;
		}
	}
	connect();
}

selectionbox.addEventListener("change", async (event) => {
	topic = selectionbox.value;
	topic_type = await getTopicType();
	connect();
	saveSettings();
});

selectionbox.addEventListener("click", connect);
icon.addEventListener("click", loadTopics);

loadTopics();

console.log("Inspector Widget Loaded {uniqueID}")
