import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';
import { toDataURL } from '/js/modules/util.js';

let topic = getTopic("{uniqueID}");

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
}

function saveSettings(){
	settings["{uniqueID}"] = {
		topic: topic,
	}
	settings.save();
}

let icons = {};
icons["20%"] = await toDataURL("assets/battery_20.svg");
icons["40%"] = await toDataURL("assets/battery_40.svg");
icons["60%"] = await toDataURL("assets/battery_60.svg");
icons["80%"] = await toDataURL("assets/battery_80.svg");
icons["100%"] = await toDataURL("assets/battery_100.svg");
icons["unknown"] = await toDataURL("assets/battery_unknown.svg");

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
const text_current = document.getElementById("{uniqueID}_current");
const text_charge = document.getElementById("{uniqueID}_charge");

const text_status = document.getElementById("{uniqueID}_status");
const text_health = document.getElementById("{uniqueID}_health");
const text_chemistry = document.getElementById("{uniqueID}_chemistry");

let listener = undefined;
let batterytopic = undefined;

function connect(){

	if(topic == "")
		return;

	if(batterytopic !== undefined){
		batterytopic.unsubscribe(listener);
	}

	batterytopic = new ROSLIB.Topic({
		ros : rosbridge.ros,
		name : topic,
		messageType : 'sensor_msgs/msg/BatteryState',
		throttle_rate: 500 // throttle to twice a second max
	});
	
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

		text_status.innerText = "Status: "+STATUS[msg.power_supply_status];
		text_health.innerText = "Health: "+HEALTH[msg.power_supply_health];
		text_chemistry.innerText = "Type: "+CHEMISTRY[msg.power_supply_technology];
	});

	saveSettings();
}

async function loadTopics(){
	let result = await rosbridge.get_topics("sensor_msgs/msg/BatteryState");
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
