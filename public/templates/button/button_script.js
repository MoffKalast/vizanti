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

let typedict = {};

//persistent loading, so we don't re-fetch on every update
let icons = {};
icons["true"] = await imageToDataURL("assets/button_true.svg");
icons["false"] = await imageToDataURL("assets/button_false.svg");
icons["default"] = await imageToDataURL("assets/button.svg");

const selectionbox = document.getElementById("{uniqueID}_topic");
const icondiv = document.getElementById("{uniqueID}_icon");
const icon = icondiv.getElementsByTagName('img')[0];
const icontext = icondiv.getElementsByTagName('p')[0];
const namebox = document.getElementById("{uniqueID}_name");

//dataset text for in-folder text display
function setLabel(string){
	icontext.textContent = string;
	icon.alt = string;
	icon.dataset.text = string;
}

namebox.addEventListener('input', function() {
	setLabel(namebox.value);
	saveSettings();
});

//Settings
if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
	typedict = loaded_data.typedict ?? {};

	namebox.value = loaded_data.text;
	setLabel(namebox.value);
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
		text: namebox.value,
		typedict: typedict
	}
	settings.save();
}

//Messaging
function sendMessage(){

	icondiv.classList.add("button-press-effect");

	setTimeout(() => {
		icondiv.classList.remove("button-press-effect");
	}, 200);

	if(typedict[topic] == "std_msgs/Bool" || typedict[topic] == "std_msgs/Empty"){
		const publisher = new ROSLIB.Topic({
			ros: rosbridge.ros,
			name: topic,
			messageType: typedict[topic],
			throttle_rate: 33
		});

		if(typedict[topic] == "std_msgs/Bool"){
			publisher.publish(new ROSLIB.Message({
				data: !value,
			}));
		}else{
			publisher.publish(new ROSLIB.Message({}));
		}
	}
	else if(typedict[topic] == "std_srvs/Empty"){
		const service = new ROSLIB.Service({
			ros: rosbridge.ros,
			name: topic,
			serviceType: "std_srvs/Empty"
		});
		const request = new ROSLIB.ServiceRequest({});
		service.callService(request, (result) => {});
	}
	else if(typedict[topic] == "std_srvs/Trigger"){
		const service = new ROSLIB.Service({
			ros: rosbridge.ros,
			name: topic,
			serviceType: "std_srvs/Trigger"
		});
		const request = new ROSLIB.ServiceRequest({});
		service.callService(request, (result) => {
			if(result.success){
				status.setOK(result.message);
			}else{
				status.setError(result.message);
			}
			
			//flash result state
			icon.src = icons[result.success];
			setTimeout(()=>{
				icon.src = icons["default"];
			}, 500);
		});
	}
	else if(typedict[topic] == "std_srvs/SetBool"){
		const service = new ROSLIB.Service({
			ros: rosbridge.ros,
			name: topic,
			serviceType: "std_srvs/SetBool"
		});
		const request = new ROSLIB.ServiceRequest({
			data: !value  // toggle the value
		});
		service.callService(request, (result) => {
			if(result.success){
				value = !value;
				icon.src = icons[value];
				status.setOK(result.message);
			}else{
				status.setError(result.message);
			}
		});
	}
}

let value = false;
let listener = undefined;
let booltopic = undefined;

function connect(){

	if(topic == ""){
		status.setError("Empty topic/service.");
		return;
	}

	if(booltopic !== undefined){
		booltopic.unsubscribe(listener);
	}

	if(typedict[topic] == "std_msgs/Bool"){

		status.setWarn("No data received.");

		booltopic = new ROSLIB.Topic({
			ros : rosbridge.ros,
			name : topic,
			messageType : "std_msgs/Bool",
			throttle_rate: 33
		});	
		
		listener = booltopic.subscribe((msg) => {
			value = msg.data;
			icon.src = icons[value];
			status.setOK();
		});

		icon.src = icons["false"];
	}
	else if(typedict[topic] == "std_srvs/SetBool"){
		icon.src = icons["false"];
	}	
	else{
		icon.src = icons["default"];
	}	

	saveSettings();
}

async function loadTopics(){
	let booltopics = await rosbridge.get_topics("std_msgs/Bool");
	let emptypubs = await rosbridge.get_topics("std_msgs/Empty");
	let emptysrvs = await rosbridge.get_services("std_srvs/Empty");
	let triggersrvs = await rosbridge.get_services("std_srvs/Trigger");
	let setboolsrvs = await rosbridge.get_services("std_srvs/SetBool");

	let topiclist = "";

	booltopics.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+" (msgs/Bool)</option>";
		typedict[element] = "std_msgs/Bool";
	});

	emptypubs.forEach(element => {
		topiclist += "<option value='"+element+"'>"+element+" (msgs/Empty)</option>";
		typedict[element] = "std_msgs/Empty";
	});

	emptysrvs.forEach(element => {
		if(!element.includes("/vizanti/")){
			topiclist += "<option value='"+element+"'>"+element+" (srvs/Empty)</option>";
			typedict[element] = "std_srvs/Empty";
		}
	});

	triggersrvs.forEach(element => {
		if(!element.includes("/vizanti/")){
			topiclist += "<option value='"+element+"'>"+element+" (srvs/Trigger)</option>";
			typedict[element] = "std_srvs/Trigger";
		}
	});

	setboolsrvs.forEach(element => {
		if(!element.includes("/vizanti/")){
			topiclist += "<option value='"+element+"'>"+element+" (srvs/SetBool)</option>";
			typedict[element] = "std_srvs/SetBool";
		}
	});

	selectionbox.innerHTML = topiclist;

	if(topic == "")
		topic = selectionbox.value;
	else{
		if(typedict.hasOwnProperty(topic)){
			selectionbox.value = topic;
		}else{
			topiclist += "<option value='"+topic+"'>"+topic+"</option>";
			selectionbox.innerHTML = topiclist;
			selectionbox.value = topic;
		}
	}
}


selectionbox.addEventListener("change", (event) => {
	topic = selectionbox.value;
	icon.src = icons["default"];
	connect();
});

selectionbox.addEventListener("click", connect);
icon.addEventListener("click", loadTopics);

loadTopics();
connect();

// Long press modal open stuff
let longPressTimer;
let isLongPress = false;

icondiv.addEventListener("click", (event) =>{
	if(!isLongPress){
		sendMessage();
	}else{
		isLongPress = false;
	}
});

icondiv.addEventListener("mousedown", startLongPress);
icondiv.addEventListener("touchstart", startLongPress);

icondiv.addEventListener("mouseup", cancelLongPress);
icondiv.addEventListener("mouseleave", cancelLongPress);
icondiv.addEventListener("touchend", cancelLongPress);
icondiv.addEventListener("touchcancel", cancelLongPress);

icondiv.addEventListener("contextmenu", (event) => {
	event.preventDefault();
});

function startLongPress(event) {
	isLongPress = false;
	longPressTimer = setTimeout(() => {
		isLongPress = true;
		loadTopics();
		connect();
		openModal("{uniqueID}_modal");
	}, 500);
}

function cancelLongPress(event) {
	clearTimeout(longPressTimer);
}

console.log("Button Widget Loaded {uniqueID}")