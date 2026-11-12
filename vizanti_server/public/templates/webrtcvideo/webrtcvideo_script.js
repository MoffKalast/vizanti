let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let settings = persistentModule.settings;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;

let img_offset_x = "-999";
let img_offset_y = "-999";

const clamp = (num, min, max) => Math.min(Math.max(num, min), max);
const vwToVh = vw => (vw * window.innerWidth) / window.innerHeight;

const DEFAULT_WEBRTC_PORT = "8080";
const FALLBACK_WEBRTC_HOST = "192.168.33.66";

function getDefaultWebrtcServer() {
	const host = window.location.hostname;
	if (host === "localhost") {
		return `localhost:${DEFAULT_WEBRTC_PORT}`;
	}
	if (host && host !== "localhost" && host !== "127.0.0.1") {
		return `${host}:${DEFAULT_WEBRTC_PORT}`;
	}
	return `${FALLBACK_WEBRTC_HOST}:${DEFAULT_WEBRTC_PORT}`;
}


let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

//persistent loading, so we don't re-fetch on every update
let stock_images = {};
stock_images["loading"] = await imageToDataURL("assets/tile_loading.png");
stock_images["error"] = await imageToDataURL("assets/tile_error.png");

let image_topic = undefined;
let listener = undefined;

let serverIPBox = null;
let cameraNameBox = null;
let resolutionBox = null;
let minBitrateBox = null;
let maxBitrateBox = null;
let codecBox = null;
let rotationbox = null;
let connectButton = null;

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const video = document.getElementById('{uniqueID}_video');
const imgpreview = document.getElementById('{uniqueID}_imgpreview');

let opacitySlider = null;
let opacityValue = null;
let widthSlider = null;
let widthValue = null;

function bindModalElements() {
	serverIPBox = document.getElementById("{uniqueID}_server_ip");
	cameraNameBox = document.getElementById('{uniqueID}_camera_name');
	resolutionBox = document.getElementById('{uniqueID}_resolution');
	minBitrateBox = document.getElementById('{uniqueID}_min_bitrate');
	maxBitrateBox = document.getElementById('{uniqueID}_max_bitrate');
	codecBox = document.getElementById('{uniqueID}_codec');
	rotationbox = document.getElementById("{uniqueID}_rotation");
	connectButton = document.getElementById('{uniqueID}_connect_button');
	opacitySlider = document.getElementById('{uniqueID}_opacity');
	opacityValue = document.getElementById('{uniqueID}_opacity_value');
	widthSlider = document.getElementById('{uniqueID}_width');
	widthValue = document.getElementById('{uniqueID}_width_value');

	if (serverIPBox && !serverIPBox.dataset.boundWidget) {
		serverIPBox.addEventListener('input', () => saveSettings());
		serverIPBox.dataset.boundWidget = '1';
	}

	if (connectButton && !connectButton.dataset.boundWidget) {
		connectButton.addEventListener('click', () => {
			toggleConnection();
		});
		connectButton.dataset.boundWidget = '1';
	}

	[cameraNameBox, resolutionBox, minBitrateBox, maxBitrateBox, codecBox, rotationbox].forEach((el) => {
		if (el && !el.dataset.boundWidget) {
			el.addEventListener('change', () => saveSettings());
			el.dataset.boundWidget = '1';
		}
	});

	if (opacitySlider && !opacitySlider.dataset.boundWidget) {
		opacitySlider.addEventListener('input', () => {
			if (opacityValue) opacityValue.textContent = opacitySlider.value;
			saveSettings();
		});
		opacitySlider.dataset.boundWidget = '1';
	}

	if (widthSlider && !widthSlider.dataset.boundWidget) {
		widthSlider.addEventListener('input', () => {
			if (widthValue) widthValue.textContent = widthSlider.value;
			saveSettings();
		});
		widthSlider.dataset.boundWidget = '1';
	}

	updateConnectButton();
}

bindModalElements();

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	bindModalElements();
	const loaded_data  = settings["{uniqueID}"];
	if (serverIPBox) serverIPBox.value = loaded_data.serverIP || getDefaultWebrtcServer();
	if (cameraNameBox) cameraNameBox.value = loaded_data.cameraName || "Front";
	if (resolutionBox) resolutionBox.value = loaded_data.resolution || "1280,720";
	if (minBitrateBox) minBitrateBox.value = loaded_data.minBitrate || 500000;
	if (maxBitrateBox) maxBitrateBox.value = loaded_data.maxBitrate || 1500000;
	if (codecBox) codecBox.value = loaded_data.codec || "h264";

	img_offset_x = loaded_data.img_offset_x;
	img_offset_y = loaded_data.img_offset_y;

	if (opacitySlider) opacitySlider.value = loaded_data.opacity;
	if (opacityValue) opacityValue.innerText = loaded_data.opacity;
	video.style.opacity = loaded_data.opacity;

	if (widthSlider) widthSlider.value = loaded_data.width;
	if (widthValue) widthValue.innerText = loaded_data.width;
	if (rotationbox) rotationbox.value = loaded_data.rotation;

	video.style.transform = `translate(-50%, -50%) rotate(${loaded_data.rotation}deg)`;
	displayImageOffset(img_offset_x, img_offset_y);
}else{
	if (serverIPBox) serverIPBox.value = getDefaultWebrtcServer();
	displayImageOffset(0, 100);
	saveSettings();
}

function saveSettings(){
	bindModalElements();
	const opacity = opacitySlider ? opacitySlider.value : "0.85";
	const width = widthSlider ? widthSlider.value : "25";
	const rotation = rotationbox ? rotationbox.value : "0";

	settings["{uniqueID}"] = {
		serverIP: serverIPBox ? serverIPBox.value : getDefaultWebrtcServer(),
		cameraName: cameraNameBox ? cameraNameBox.value : "Front",
		resolution: resolutionBox ? resolutionBox.value : "1280,720",
		minBitrate: minBitrateBox ? minBitrateBox.value : 500000,
		maxBitrate: maxBitrateBox ? maxBitrateBox.value : 1500000,
		codec: codecBox ? codecBox.value : "h264",
		opacity: opacity,
		width: width,
		img_offset_x: img_offset_x,
		img_offset_y: img_offset_y,
		rotation: rotation
	}
	settings.save();

	video.style.opacity = opacity;
	video.style.transform = `translate(-50%, -50%) rotate(${rotation}deg)`;
	displayImageOffset(img_offset_x, img_offset_y);
}


var pc = null;

function updateConnectButton() {
	if (!connectButton) {
		return;
	}

	const connected = !!pc;
	connectButton.textContent = connected ? 'Disconnect' : 'Connect';
	connectButton.classList.toggle('delete_button', connected);
}

function negotiate() {
	pc.addTransceiver('video', { direction: 'recvonly' });
	return pc.createOffer().then((offer) => {
		return pc.setLocalDescription(offer);
	}).then(() => {
		var offer = pc.localDescription;
		const resolutionValue = resolutionBox ? resolutionBox.value : "1280,720";
		const resolutionParts = resolutionValue.split(",").map((v) => parseInt(v, 10));
		const resolution = [
			Number.isFinite(resolutionParts[0]) ? resolutionParts[0] : 1280,
			Number.isFinite(resolutionParts[1]) ? resolutionParts[1] : 720,
		];
		const minBitrate = parseInt(minBitrateBox ? minBitrateBox.value : 500000, 10);
		const maxBitrate = parseInt(maxBitrateBox ? maxBitrateBox.value : 1500000, 10);

        return fetch('http://'+serverIPBox.value+'/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
				name: (cameraNameBox ? cameraNameBox.value : "Front") || "Front",
				resolution: resolution,
				min_bitrate: Number.isFinite(minBitrate) ? minBitrate : 500000,
				max_bitrate: Number.isFinite(maxBitrate) ? maxBitrate : 1500000,
				codec: (codecBox ? codecBox.value : "h264") || "h264",
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then((response) => {
        return response.json();
    }).then((answer) => {
		return pc.setRemoteDescription(new RTCSessionDescription(answer));
	}).then(() => {
		updateConnectButton();
    }).catch((e) => {
		updateConnectButton();
        alert(e);
    });
}

function start() {
	bindModalElements();
	const regex = /^((25[0-5]|2[0-4][0-9]|1?[0-9]{1,2})\.){3}(25[0-5]|2[0-4][0-9]|1?[0-9]{1,2}):([0-9]{1,5})$/;
	if (!serverIPBox) {
		console.log("start webrtc {uniqueID}: missing server IP input");
		return;
	}

	console.log("start webrtc {uniqueID}", serverIPBox.value);

	if(serverIPBox.value.match(regex)){
		console.log("patern corect")
		if (pc) {
			stop();
		}
		var config = {
			sdpSemantics: 'unified-plan'
		};

		pc = new RTCPeerConnection(config);
		updateConnectButton();

		pc.oniceconnectionstatechange = () => {
			console.log("ICE {uniqueID}:", pc.iceConnectionState);
			if (
				pc.iceConnectionState === 'disconnected' ||
				pc.iceConnectionState === 'failed' ||
				pc.iceConnectionState === 'closed'
			) {
				stop();
			}
		};

		pc.onconnectionstatechange = () => {
			console.log("Connection {uniqueID}:", pc.connectionState);
			if (
				pc.connectionState === 'disconnected' ||
				pc.connectionState === 'failed' ||
				pc.connectionState === 'closed'
			) {
				stop();
			}
			updateConnectButton();
		};

		// connect audio / video
		pc.addEventListener('track', (evt) => {
			if (evt.track.kind == 'video') {
				video.srcObject = evt.streams[0];
				video.play()
			}
		});
		negotiate();
	}
	else{
		console.log("patern wrong")
	}

}

function stop() {
	if (!pc) {
		updateConnectButton();
		return;
	}

	const currentPc = pc;
	pc = null;
	video.srcObject = null;
	updateConnectButton();

	setTimeout(() => {
		try {
			currentPc.close();
		} catch (_) {
			// no-op
		}
	}, 100);
}

function toggleConnection() {
	if (pc) {
		stop();
		return;
	}

	start();
}


//Topic
async function getImage(src) {
    return new Promise((resolve, reject) => {
        let img = new Image();
        img.onload = () => resolve(src);
        img.onerror = () => reject(src);
        img.src = src;
    });
}

function connect(){

	saveSettings();
}

async function loadTopics(){
	updateConnectButton();
}

loadTopics();

//preview for definining position
let preview_active = false;

function onStart(event) {
	preview_active = true;
	document.addEventListener('mousemove', onMove);
	document.addEventListener('touchmove', onMove);
	document.addEventListener('mouseup', onEnd);
	document.addEventListener('touchend', onEnd);
}

function displayImageOffset(x, y){
	bindModalElements();
	if (!widthSlider || !imgpreview) {
		return;
	}

	if(video.naturalWidth == 0)
		return;

	let img_width = widthSlider.value;
	let img_height = (vwToVh(img_width) * 600)/800;
	video.style.width = img_width+"vw";
	video.style.height = img_height+"vh";

	let offset_x = clamp(x, img_width/2, 100 - img_width/2);
	let offset_y = clamp(y, img_height/2, 100 - img_height/2);
	imgpreview.style.left = offset_x+"vw";
	imgpreview.style.top = offset_y+"vh";

	video.style.left = offset_x+"vw";
	video.style.top = offset_y+"vh";
	
}

window.addEventListener('resize', ()=>{
	displayImageOffset(img_offset_x, img_offset_y);
});

function onMove(event) {
	if (preview_active) {
		event.preventDefault();
		let currentX, currentY;

		if (event.type === "touchmove") {
			currentX = event.touches[0].clientX;
			currentY = event.touches[0].clientY;
		} else {
			currentX = event.clientX;
			currentY = event.clientY;
		}
	
		let img_width = widthSlider.value/2;
		let img_height = (vwToVh(img_width) * 600)/800;
	
		img_offset_x = clamp(currentX/window.innerWidth * 100, img_width, 100 - img_width);
		img_offset_y = clamp(currentY/window.innerHeight * 100, img_height, 100 - img_height);

		saveSettings();
	}
}

function onEnd() {
	preview_active = false;
	document.removeEventListener('mousemove', onMove);
	document.removeEventListener('touchmove', onMove);
	document.removeEventListener('mouseup', onEnd);
	document.removeEventListener('touchend', onEnd);
}
  
if (imgpreview) {
	imgpreview.addEventListener('mousedown', onStart);
	imgpreview.addEventListener('touchstart', onStart);
}

if (icon) {
	icon.addEventListener('click', () => {
		bindModalElements();
	});
}

displayImageOffset(img_offset_x, img_offset_y);

console.log("Image Widget Loaded {uniqueID}")

