let viewModule = await import(`${base_url}/js/modules/view.js`);
let tfModule = await import(`${base_url}/js/modules/tf.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let pathsModule = await import(`${base_url}/assets/robot_model/paths`);

let view = viewModule.view;
let tf = tfModule.tf;
let settings = persistentModule.settings;
let Status = StatusModule.Status;
let paths = pathsModule.default;

let models = {};
let categorizedModels = {};
let thumbnailCache = {};

// Since paths is now categorized, we need to handle it differently
Object.keys(paths).forEach(category => {
	categorizedModels[category] = [];

	paths[category].forEach(file => {
		const name = file.split('.png')[0].split("_")[1];
		categorizedModels[category].push(name);
		
		if (!models[name]) {
			models[name] = new Image();
			models[name].category = category;

			if(category == "misc")
				models[name].src = `${base_url}/assets/robot_model/${file}`;
			else
				models[name].src = `${base_url}/assets/robot_model/${category}/${file}`;
		}
	});
});

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const frameSelector = document.getElementById("{uniqueID}_frame");
const lengthSelector = document.getElementById("{uniqueID}_length");
const galleryTabs = document.getElementById("{uniqueID}_gallery_tabs");
const gallery = document.getElementById('{uniqueID}_gallery');

let frame = "";
let sprite = "4wd";

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	frame = loaded_data.frame;
	lengthSelector.value = loaded_data.length;

	sprite = loaded_data.sprite ?? "4wd";
}else{

	if(frame == ""){
		frame = "base_link";
		status.setWarn("No frame found, defaulting to base_link");
	}

	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		frame: frame,
		sprite: sprite,
		length: lengthSelector.value
	}
	settings.save();
}

async function drawRobot() {

	function getRotationMatrix(pitchRad, yawRad, rollRad) {
		const cosPitch = Math.cos(pitchRad);
		const sinPitch = Math.sin(pitchRad);
		const cosYaw = Math.cos(yawRad);
		const sinYaw = Math.sin(yawRad);
		const cosRoll = Math.cos(rollRad);
		const sinRoll = Math.sin(rollRad);
		const sinPitchSinRoll = sinPitch * sinRoll;
		
		return [
			cosYaw * cosPitch,                           // m11
			cosYaw * sinPitchSinRoll - sinYaw * cosRoll, // m12
			sinYaw * cosPitch,                           // m21
			sinYaw * sinPitchSinRoll + cosYaw * cosRoll  // m22
		];
	}

	const unit = view.getMapUnitsInPixels(lengthSelector.value);

    const wid = canvas.width;
    const hei = canvas.height;

	ctx.setTransform(1,0,0,1,0,0);
    ctx.clearRect(0, 0, wid, hei);

	const robotframe = tf.absoluteTransforms[frame];
	const modelimg = models[sprite];

	if(robotframe){
		const pos = view.fixedToScreen({
			x: robotframe.translation.x,
			y: robotframe.translation.y,
		});
	
		const euler = robotframe.rotation.toEuler();

		const roll = euler.g;
		const pitch = euler.pitch;
		const yaw = euler.h;

		const matrix = getRotationMatrix(
			-pitch, 
			Math.PI - yaw, 
			-roll
		);

		let ratio = modelimg.naturalHeight/modelimg.naturalWidth;
		ctx.setTransform(matrix[0], matrix[2], matrix[1], matrix[3],pos.x, pos.y); //sx,0,0,sy,px,py
		ctx.drawImage(modelimg, -unit/2, -(unit*ratio)/2, unit, unit*ratio);
		
		status.setOK();
	}else{
		status.setError("Required transform frame \""+frame+"\" not found.");
	}
}

function buildThumbnailGallery() {

	function generateThumbnail(image, size = 64) {
		const scale = Math.min(size / image.naturalWidth, size / image.naturalHeight);
		const scaledWidth = image.naturalWidth * scale;
		const scaledHeight = image.naturalHeight * scale;

		const canvas = document.createElement('canvas');
		canvas.width = size;
		canvas.height = size;

		const ctx = canvas.getContext('2d');
		ctx.clearRect(0, 0, size, size);
		ctx.drawImage(
			image, 
			(size - scaledWidth) / 2, //x
			(size - scaledHeight) / 2, //y
			scaledWidth, 
			scaledHeight
		);
		
		return canvas.toDataURL();
	}

	function selectSprite(event, modelName) {
		sprite = modelName;

		gallery.querySelectorAll('.thumbnail-item').forEach(item => {
			item.classList.remove('selected');
		});

		gallery.querySelectorAll('.thumbnail-item').forEach(item => {
			if (item.querySelector('.thumb-label').textContent === modelName) {
				item.classList.add('selected');
			}
		});

		event.currentTarget.classList.add('selected');
		
		saveSettings();
	}

    const activeTab = galleryTabs.querySelector('.active-tab');
    const category = activeTab.id.replace("{uniqueID}_","");
    
    gallery.innerHTML = '';
    if (categorizedModels[category]) {

        categorizedModels[category].forEach(modelName => {
            const model = models[modelName];
            if (!model) return;
            
            // Generate or get cached thumbnail
            if (!thumbnailCache[modelName]) {
                thumbnailCache[modelName] = generateThumbnail(model);
            }
            
            // Create thumbnail element
            const thumbDiv = document.createElement('div');
            thumbDiv.className = 'thumbnail-item';

            if (sprite === modelName)
				thumbDiv.classList.add('selected');
            
            thumbDiv.innerHTML = `
                <img src="${thumbnailCache[modelName]}" alt="${modelName}">
                <span class="thumb-label">${modelName}</span>
            `;
            
            thumbDiv.addEventListener('click', (event) => selectSprite(event, modelName));
            gallery.appendChild(thumbDiv);
        });
    }
}

function setActiveCategory(element){
	galleryTabs.querySelectorAll('.active-tab').forEach(item => {
		item.classList.remove('active-tab');
	});

	element.classList.add('active-tab');
	buildThumbnailGallery();
}

galleryTabs.addEventListener('click', (event) => {
	if(event.target != null && event.target.classList.contains("tablinks")){
		setActiveCategory(event.target);
		drawRobot();
	}	
});

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawRobot();
}

window.addEventListener("tf_fixed_frame_changed", drawRobot);
window.addEventListener("tf_changed", ()=>{
	if(frame != tf.fixed_frame){
		drawRobot();
	}
});

window.addEventListener("view_changed", drawRobot);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

// TF frame list
function setFrameList(){
	let framelist = "";
	for (const key of tf.frame_list.values()) {
		framelist += "<option value='"+key+"'>"+key+"</option>"
	}
	frameSelector.innerHTML = framelist;

	if(tf.transforms.hasOwnProperty(frame)){
		frameSelector.value = frame;
	}else{
		framelist += "<option value='"+frame+"'>"+frame+"</option>"
		frameSelector.innerHTML = framelist
		frameSelector.value = frame;
	}

	if(models[sprite]){
		const element = document.getElementById("{uniqueID}_"+models[sprite].category);
		setActiveCategory(element);
	}
}

frameSelector.addEventListener("change", (event) => {
	frame = frameSelector.value;
	saveSettings();
});

lengthSelector.addEventListener("input", saveSettings);

frameSelector.addEventListener("click", setFrameList);
icon.addEventListener("click", setFrameList);

frameSelector.addEventListener("change", (event) =>{
	frame = frameSelector.value;
	drawRobot();
	saveSettings();
});

resizeScreen();

console.log("Model Widget Loaded {uniqueID}")