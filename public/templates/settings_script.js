import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';
import { settings } from '/js/modules/persistent.js';

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const selectionbox = document.getElementById("{uniqueID}_fixedframe");

// Settings
if (settings.hasOwnProperty('{uniqueID}')) {
	const loadedData = settings['{uniqueID}'];
	tf.setFixedFrame(loadedData.fixed_frame);
}
else{
	saveSettings();
}

function saveSettings() {
	settings['{uniqueID}'] = {
		fixed_frame: tf.fixed_frame
	};
	settings.save();
}

// TF frame list

function setFrameList(){

	let framelist = "";
	Object.keys(tf.transforms).forEach(key => {
		framelist += "<option value='"+key+"'>"+key+"</option>"
	});
	selectionbox.innerHTML = framelist;

	if(tf.transforms.hasOwnProperty(tf.fixed_frame)){
		selectionbox.value = tf.fixed_frame;
	}else{
		framelist += "<option value='"+tf.fixed_frame+"'>"+tf.fixed_frame+"</option>"
		selectionbox.innerHTML = framelist
		selectionbox.value = tf.fixed_frame;
	}
}

selectionbox.addEventListener("change", (event) => {
	tf.setFixedFrame(selectionbox.value);
	saveSettings();
});

selectionbox.addEventListener("click", setFrameList);
icon.addEventListener("click", setFrameList);

// Buttons
const modal = document.getElementById("{uniqueID}_modal");

document.getElementById("{uniqueID}_reset_view").addEventListener("click", (event) =>{
	view.reset();
	modal.style.display = "none";
});

async function deletePersistent() {
	let del_ok = await confirm("Are you sure you want to delete your saved widget setup? This will refresh the page.");

	if(del_ok){
		localStorage.removeItem("settings");
		window.location.reload();
	}
}

document.getElementById("{uniqueID}_delete_persistent").addEventListener("click", deletePersistent);