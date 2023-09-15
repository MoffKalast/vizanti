let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

let url = document.getElementById("{uniqueID}_url");
let icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];

// can't show images about reconnecting without preloading them before we lose connection
let img_reconnect = await imageToDataURL('assets/rosbridge_reconnect.svg');
let img_connect = await imageToDataURL('assets/rosbridge_connected.svg');
let img_disconnect = await imageToDataURL('assets/rosbridge_disconnected.svg');

function update_gui(){
	url.innerText = "Bridge URL: ws://"+rosbridge.url + ":"+rosbridge.port;

	switch (rosbridge.status) {
		case "Reconnecting...":
			icon.src = img_reconnect;
			status.setError(rosbridge.status);
			break;
		case "Connecting...":
			icon.src = img_connect;
			status.setWarn(rosbridge.status);
			break;
		case 'Connection lost.':
		case 'Failed to connect.':
			icon.src = img_disconnect;
			status.setError(rosbridge.status);
			break;
		default:
			icon.src = img_connect;
			status.setOK(rosbridge.status);
	}
}

window.addEventListener("rosbridge_change", function(event){
	update_gui();    
});

update_gui();