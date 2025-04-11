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

const serverIPBox = document.getElementById("{uniqueID}_server_ip");
serverIPBox.addEventListener('input', () =>  {
	saveSettings();
});
const rotationbox = document.getElementById("{uniqueID}_rotation");

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const video = document.getElementById('{uniqueID}_video');
const imgpreview = document.getElementById('{uniqueID}_imgpreview');

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');
opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

const widthSlider = document.getElementById('{uniqueID}_width');
const widthValue = document.getElementById('{uniqueID}_width_value');
widthSlider.addEventListener('input', () =>  {
	widthValue.textContent = widthSlider.value;
	saveSettings();
});

rotationbox.addEventListener("change", (event) => {
	saveSettings();
});

//Settings

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	serverIPBox.value = loaded_data.serverIP;

	img_offset_x = loaded_data.img_offset_x;
	img_offset_y = loaded_data.img_offset_y;

	opacitySlider.value = loaded_data.opacity;
	opacityValue.innerText = loaded_data.opacity;
	video.style.opacity = loaded_data.opacity;

	widthSlider.value = loaded_data.width;
	widthValue.innerText = loaded_data.width;
	rotationbox.value = loaded_data.rotation;

	video.style.transform = `translate(-50%, -50%) rotate(${loaded_data.rotation}deg)`;
	displayImageOffset(img_offset_x, img_offset_y);
}else{
	displayImageOffset(0, 100);
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		serverIP: serverIPBox.value,
		opacity: opacitySlider.value,
		width: widthSlider.value,
		img_offset_x: img_offset_x,
		img_offset_y: img_offset_y,
		rotation: rotationbox.value
	}
	settings.save();

	video.style.opacity = opacitySlider.value;
	video.style.transform = `translate(-50%, -50%) rotate(${rotationbox.value}deg)`;
	displayImageOffset(img_offset_x, img_offset_y);
}


var pc = null;

function negotiate() {
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('audio', { direction: 'recvonly' });
    return pc.createOffer().then((offer) => {
        return pc.setLocalDescription(offer);
    }).then(() => {
        // wait for ICE gathering to complete
        return new Promise((resolve) => {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                const checkState = () => {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                };
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(() => {
        var offer = pc.localDescription;
        return fetch('http://'+serverIPBox.value+'/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
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
    }).catch((e) => {
        alert(e);
    });
}

function start() {
	const regex = /^((25[0-5]|2[0-4][0-9]|1?[0-9]{1,2})\.){3}(25[0-5]|2[0-4][0-9]|1?[0-9]{1,2}):([0-9]{1,5})$/;
	console.log("start webrtc {uniqueID}", serverIPBox.value);

	if(serverIPBox.value.match(regex)){
		console.log("patern corect")
		var config = {
			sdpSemantics: 'unified-plan'
		};

		pc = new RTCPeerConnection(config);

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

    // close peer connection
    setTimeout(() => {
        pc.close();
    }, 500);
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
	start();
}

serverIPBox.addEventListener("change", (event) => {
	start();
});


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
  
imgpreview.addEventListener('mousedown', onStart);
imgpreview.addEventListener('touchstart', onStart);

displayImageOffset(img_offset_x, img_offset_y);

console.log("Image Widget Loaded {uniqueID}")

