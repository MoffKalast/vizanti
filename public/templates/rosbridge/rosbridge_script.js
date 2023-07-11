import { rosbridge } from '/js/modules/rosbridge.js';
import { imageToDataURL } from '/js/modules/util.js';

let url = document.getElementById("{uniqueID}_url");
let statustext = document.getElementById("{uniqueID}_status");
let icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

// can't show images about reconnecting without preloading them before we lose connection
let img_reconnect = await imageToDataURL('assets/rosbridge_reconnect.svg');
let img_connect = await imageToDataURL('assets/rosbridge_connected.svg');
let img_disconnect = await imageToDataURL('assets/rosbridge_disconnected.svg');

function update_gui(){
	url.innerText = "Bridge URL: ws://"+rosbridge.url + ":"+rosbridge.port;

	statustext.innerText = rosbridge.status;

	switch (rosbridge.status) {
		case "Reconnecting...":
			icon.src = img_reconnect;
			statustext.style.color = "orange";
			break;
		case "Connecting...":
			icon.src = img_connect;
			statustext.style.color = "yellow";
			break;
		case 'Connection lost.':
		case 'Failed to connect.':
			icon.src = img_disconnect;
			statustext.style.color = "red";
			break;
		default:
			icon.src = img_connect;
			statustext.style.color = "lime";
	}
}

window.addEventListener("rosbridge_change", function(event){
	update_gui();    
});

update_gui();