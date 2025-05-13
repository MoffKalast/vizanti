let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;

let topic = getTopic("{uniqueID}");
let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let listener = undefined;
let temperature_topic = undefined;

//persistent loading, so we don't re-fetch on every update
let icons = {};
icons["hot"] = await imageToDataURL("assets/temp_hot.svg");
icons["cold"] = await imageToDataURL("assets/temp_cold.svg");
icons["warm"] = await imageToDataURL("assets/temp_warm.svg");
icons["unknown"] = await imageToDataURL("assets/temp_warm.svg");

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const highBox = document.getElementById('{uniqueID}_hightemp');
const lowBox = document.getElementById('{uniqueID}_lowtemp');

const text_temperature = document.getElementById("{uniqueID}_temperature");
const text_variance = document.getElementById("{uniqueID}_variance");
const text_link = document.getElementById("{uniqueID}_tflink");

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	highBox.value = loaded_data.high;
	lowBox.value = loaded_data.low;
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		low: lowBox.value,
		high: highBox.value
	}
	settings.save();
}

function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(temperature_topic !== undefined){
		temperature_topic.unsubscribe(listener);
	}

	temperature_topic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/Temperature',
		throttle_rate: 500 // throttle to twice a second max
	});

	status.setWarn("No data received.");
	
	listener = temperature_topic.subscribe((msg) => {

		if(msg.temperature > highBox.value){
			icon.src = icons["hot"];
		}
		else if(msg.temperature < lowBox.value){
			icon.src = icons["cold"];
		}
		else{
			icon.src = icons["warm"];
		}

		text_temperature.innerText = "Temperature (°C): "+(Math.round(msg.temperature * 100) / 100).toFixed(2);
		text_variance.innerText = "Variance: "+(Math.round(msg.variance * 100) / 100).toFixed(2);
		text_link.innerText = "TF Frame: "+msg.header.frame_id;

		status.setOK();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/Temperature");

	let topiclist = "";
	result.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+"</option>"
	});
	selectionbox.innerHTML = topiclist

	if(topic == "")
		topic = selectionbox.value;
	else{
		if(result.includes(topic)){
			selectionbox.value = topic;
		}else{
			topiclist += "<option value='"+topic+"'>"+topic+"</option>"
			selectionbox.innerHTML = topiclist
			selectionbox.value = topic;
		}
	}
	connect();
}

selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	icon.src = icons["unknown"];
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();

console.log("Temperature Widget Loaded {uniqueID}")