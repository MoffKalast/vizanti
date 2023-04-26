import { settings } from './modules/persistent.js';
import { elementTemplatesPromise } from './modules/elements.js';

var uid = NaN;
var element_templates = {};
var loaded = false;

window.addEventListener("add_widget", (event) =>{
	if(isNaN(uid)){
		if(isNaN(settings.uid))
			uid = 0;
		else
			uid = settings.uid;
	}

	settings.navbar.push({
		type: event.id,
		id: "autoID"+uid
	});
	uid++;

	settings.uid = uid;
	settings.save();
	update_nav();	
});

window.addEventListener("remove_widget", (event) =>{
	settings.navbar.splice(event.ordinal,1);
	settings.save();
	update_nav();
});

function set_unique(string, uniqueid, navbarOrdinal) {
	return string
		.replaceAll('{uniqueID}', uniqueid)
		.replaceAll('{navbarOrdinal}', navbarOrdinal);
  }

function update_nav(){
	const icon_container = document.getElementById("icon_container");
	const modal_container = document.getElementById("modal_container");
	const view_container = document.getElementById("view_container");

	let iconHTML = "";
	let modalHTML = "";
	let viewHTML = "";

	let scripts = [];

	for(let i = 0; i < settings.navbar.length; i++){
		let element = settings.navbar[i].type;
		let eid = settings.navbar[i].id;
		let template = element_templates[element];

		iconHTML += set_unique(template.icon, eid, i);
		if(template.hasOwnProperty("modal"))
			modalHTML += set_unique(template.modal, eid, i);

		if(template.hasOwnProperty("view"))
			viewHTML += set_unique(template.view, eid, i);

		if(template.hasOwnProperty("script")){
			let script = document.createElement("script");
			script.type = "module";
			script.textContent = set_unique(template.script, eid, i);
			scripts.push(script);
		}

		uid++;
	};

	modalHTML += element_templates["add"].modal; 
	iconHTML += element_templates["add"].icon; 
	
	icon_container.innerHTML = iconHTML;
	modal_container.innerHTML = modalHTML;
	view_container.innerHTML = viewHTML;

	scripts.forEach(element => {
		icon_container.appendChild(element);
	});
}

document.addEventListener("DOMContentLoaded", (event) =>{
	Promise.all([elementTemplatesPromise]).then((values) => {
		element_templates = values[0];
		loaded = true;
		update_nav();
	});	
});

