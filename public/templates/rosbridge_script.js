import { rosbridge } from '/js/modules/rosbridge.js';

let url = document.getElementById("{uniqueID}_url");
let statustext = document.getElementById("{uniqueID}_status");
let icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

function update_gui(){
	url.innerText = "Bridge URL: ws://"+rosbridge.url + ":"+rosbridge.port;

	statustext.innerText = rosbridge.status;

	switch (rosbridge.status) {
		case "Reconnecting...":
			icon.src = "assets/rosbridge_reconnect.svg";
			statustext.style.color = "orange";
			break;
		case "Connecting...":
			icon.src = "assets/rosbridge_reconnect.svg";
			statustext.style.color = "yellow";
			break;
		case 'Connection lost.':
		case 'Failed to connect.':
			icon.src = "assets/rosbridge_disconnected.svg";
			statustext.style.color = "red";
			break;
		default:
			icon.src = "assets/rosbridge_connected.svg";
			statustext.style.color = "lime";
	}
}

window.addEventListener("rosbridge_change", function(event){
	update_gui();    
});

update_gui();