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
		const name = file.split('.png')[0].split("_").join(" ").trim();
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

const offsetXSelector = document.getElementById("{uniqueID}_offset_x");
const offsetYSelector = document.getElementById("{uniqueID}_offset_y");
const offsetYawSelector = document.getElementById("{uniqueID}_offset_yaw");

const opacitySlider = document.getElementById('{uniqueID}_opacity');
const opacityValue = document.getElementById('{uniqueID}_opacity_value');
opacitySlider.addEventListener('input', () =>  {
	opacityValue.textContent = opacitySlider.value;
	saveSettings();
});

offsetXSelector.addEventListener('input', saveSettings);
offsetYSelector.addEventListener('input', saveSettings);
offsetYawSelector.addEventListener('input', saveSettings);

let frame = find_base_frame();
let sprite = "4wd";

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	frame = loaded_data.frame;
	lengthSelector.value = loaded_data.length;

	offsetXSelector.value = loaded_data.offset_x ?? 0.0;
	offsetYSelector.value = loaded_data.offset_y ?? 0.0;
	offsetYawSelector.value = loaded_data.offset_yaw ?? 0.0;

	opacitySlider.value = loaded_data.opacity  ?? 1.0;
	opacityValue.innerText = opacitySlider.value;
	canvas.style.opacity = opacitySlider.value;

	sprite = loaded_data.sprite.trim() ?? "4wd";
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		frame: frame,
		sprite: sprite,
		opacity: opacitySlider.value,
		length: lengthSelector.value,
		offset_x: offsetXSelector.value,
		offset_y: offsetYSelector.value,
		offset_yaw: offsetYawSelector.value,
	}
	settings.save();

	canvas.style.opacity = opacitySlider.value;
}

function find_base_frame(){
	//try base_link first
	for (const key of tf.frame_list.values()) {
		if (key.includes("base_link")) {
			return key
		}
	}

	//maybe footprint?
	for (const key of tf.frame_list.values()) {
		if (key.includes("base_footprint")) {
			return key
		}
	}

	//ok just base then...?
	for (const key of tf.frame_list.values()) {
		if (key.includes("base")) {
			return key
		}
	}

	//eh screw it
	return "base_link";
}

async function drawRobot() {

	const unit = view.getMapUnitsInPixels(1.0);
	const length = lengthSelector.value * unit;

    const wid = canvas.width;
    const hei = canvas.height;

	ctx.setTransform(1,0,0,1,0,0);
    ctx.clearRect(0, 0, wid, hei);

	const robotframe = tf.absoluteTransforms[frame];
	const modelimg = models[sprite];

	if(robotframe && modelimg){

		const pos = view.fixedToScreen({
			x: robotframe.translation.x,
			y: robotframe.translation.y
		});

		const matrix = view.quaterionToProjectionMatrix(robotframe.rotation);

		let ratio = modelimg.naturalHeight/modelimg.naturalWidth;
		ctx.setTransform(matrix[0], matrix[1], matrix[2], matrix[3], pos.x, pos.y); //sx,0,0,sy,px,py

		const offset_x = parseFloat(offsetXSelector.value) * unit;
		const offset_y = parseFloat(offsetYSelector.value) * unit;
		const offset_yaw = (parseFloat(offsetYawSelector.value) * (Math.PI / 180.0)) + Math.PI;

		ctx.transform(1, 0, 0, 1,  offset_x, offset_y);
		ctx.rotate(offset_yaw)

		ctx.drawImage(modelimg, -length/2, -(length*ratio)/2, length, length*ratio);
		
		status.setOK();
	}else{
		if(robotframe){
			status.setError("Required robot sprite not found..?");
		}else{
			status.setError("Required transform frame \""+frame+"\" not found.");
		}
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

        categorizedModels[category].sort().forEach(modelName => {
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