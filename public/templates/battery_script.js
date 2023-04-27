import { rosbridge } from '/js/modules/rosbridge.js';
import { settings } from '/js/modules/persistent.js';

let topic = "";

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	topic = loaded_data.topic;
}

function save_settings(){
	settings["{uniqueID}"] = {
		topic: topic,
	}
	settings.save();
}

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
		messageType : 'sensor_msgs/BatteryState'
	});
	
	listener = batterytopic.subscribe((msg) => {

		if(msg.percentage <= 0.2){
			icon.src = "assets/battery_20.svg";
		}
		else if(msg.percentage <= 0.4){
			icon.src = "assets/battery_40.svg";
		}
		else if(msg.percentage <= 0.6){
			icon.src = "assets/battery_60.svg";
		}
		else if(msg.percentage <= 0.8){
			icon.src = "assets/battery_80.svg";
		}
		else{
			icon.src = "assets/battery_100.svg";
		}

		text_percent.innerText = "Percentage: "+parseInt(msg.percentage*100)+" %";
		text_voltage.innerText = "Voltage: "+msg.voltage+" V";

		text_current.innerText = "Current draw: "+msg.current+" A";
		text_charge.innerText = "Charge: "+msg.charge+"/"+msg.capacity+" Ah";

		text_status.innerText = "Status: "+STATUS[msg.power_supply_status];
		text_health.innerText = "Health: "+HEALTH[msg.power_supply_health];
		text_chemistry.innerText = "Type: "+CHEMISTRY[msg.power_supply_technology];
	});

	save_settings();
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
	connect();
});

selectionbox.addEventListener("click", (event) => {
	connect();
});

icon.addEventListener("click", (event) => {
	loadTopics();
});

loadTopics();
