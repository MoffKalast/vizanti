import { rosbridge } from '/js/modules/rosbridge.js';

let url = document.getElementById("{uniqueID}_url");
let statustext = document.getElementById("{uniqueID}_status");
let icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

async function urlToBase64(url) {
	const response = await fetch(url);
	const blob = await response.blob();
	return new Promise((resolve, reject) => {
		const reader = new FileReader();
		reader.onloadend = () => resolve(reader.result);
		reader.onerror = reject;
		reader.readAsDataURL(blob);
	});
}

let img_reconnect = await urlToBase64('assets/rosbridge_reconnect.svg');
let img_connect = await urlToBase64('assets/rosbridge_connected.svg');
let img_disconnect = await urlToBase64('assets/rosbridge_disconnected.svg');

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