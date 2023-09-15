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

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
}else{
	saveSettings();
}


function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
	}
	settings.save();
}

let icons = {};
icons["20%"] = await imageToDataURL("assets/battery_20.svg");
icons["40%"] = await imageToDataURL("assets/battery_40.svg");
icons["60%"] = await imageToDataURL("assets/battery_60.svg");
icons["80%"] = await imageToDataURL("assets/battery_80.svg");
icons["100%"] = await imageToDataURL("assets/battery_100.svg");
icons["unknown"] = await imageToDataURL("assets/battery_unknown.svg");

const STATUS = [
	"UNKNOWN",
	"CHARGING",
	"DISCHARGING",
	"NOT_CHARGING",
	"FULL",
]

const HEALTH = [
	"UNKNOWN",
	"GOOD",
	"OVERHEAT",
	"DEAD",
	"OVERVOLTAGE",
	"UNSPEC_FAILURE",
	"COLD",
	"WATCHDOG_TIMER_EXPIRE",
	"SAFETY_TIMER_EXPIRE"
]

const CHEMISTRY = [
	"UNKNOWN",
	"NIMH",
	"LION",
	"LIPO",
	"LIFE",
	"NICD",
	"LIMN"
]

const selectionbox = document.getElementById("{uniqueID}_topic");
const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

const text_percent = document.getElementById("{uniqueID}_pecentage");
const text_voltage = document.getElementById("{uniqueID}_voltage");
const text_cell_voltage = document.getElementById("{uniqueID}_cell_voltage");
const text_current = document.getElementById("{uniqueID}_current");
const text_charge = document.getElementById("{uniqueID}_charge");

const text_status = document.getElementById("{uniqueID}_status");
const text_health = document.getElementById("{uniqueID}_health");
const text_chemistry = document.getElementById("{uniqueID}_chemistry");

let listener = undefined;
let batterytopic = undefined;

function connect(){

	if(topic == ""){
		status.setError("Empty topic.");
		return;
	}

	if(batterytopic !== undefined){
		batterytopic.unsubscribe(listener);
	}

	batterytopic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/BatteryState',
		throttle_rate: 500 // throttle to twice a second max
	});

	status.setWarn("No data received.");
	
	listener = batterytopic.subscribe((msg) => {

		if(msg.percentage <= 0.2){
			icon.src = icons["20%"];
		}
		else if(msg.percentage <= 0.4){
			icon.src = icons["40%"];
		}
		else if(msg.percentage <= 0.6){
			icon.src = icons["60%"];
		}
		else if(msg.percentage <= 0.8){
			icon.src = icons["80%"];
		}
		else{
			icon.src = icons["100%"];
		}

		text_percent.innerText = "Percentage: "+parseInt(msg.percentage*100)+" %";

		if(msg.voltage)
			text_voltage.innerText = "Voltage: "+msg.voltage.toFixed(2)+" V";

		if(msg.current)
			text_current.innerText = "Current draw: "+msg.current.toFixed(2)+" A";

		if(msg.charge)
			text_charge.innerText = "Charge: "+msg.charge.toFixed(2)+"/"+msg.capacity.toFixed(2)+" Ah";

		if(msg.cell_voltage.length > 0){
			let cellstr = "Cell Voltages: ";

			for(let i = 0; i < msg.cell_voltage.length; i++){
				cellstr += msg.cell_voltage[i].toFixed(2)+" V, ";
			}

			text_cell_voltage.innerText = cellstr.substring(0, cellstr.length - 2);
		}
			

		text_status.innerText = "Status: "+STATUS[msg.power_supply_status];
		text_health.innerText = "Health: "+HEALTH[msg.power_supply_health];
		text_chemistry.innerText = "Type: "+CHEMISTRY[msg.power_supply_technology];
		
		status.setOK();
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/BatteryState");
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
	saveSettings();
});

selectionbox.addEventListener("click", connect);
icon.addEventListener("click", loadTopics);

loadTopics();

console.log("Battery Widget Loaded {uniqueID}")
