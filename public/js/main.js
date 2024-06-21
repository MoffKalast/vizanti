import { settings } from './modules/persistent.js';
import { elementTemplatesPromise } from './modules/elements.js';

var uid = NaN;
var element_templates = {};

function set_unique(string, uniqueid) {
	return string.replaceAll('{uniqueID}', uniqueid);
}

function createElement(template, eid){
	let div = document.createElement("div");
	div.dataset.uniqueid = eid;
	div.innerHTML = set_unique(template, eid);
	return div;
}

function createScript(template, eid){
	let script = document.createElement("script");
	script.dataset.uniqueid = eid;
	script.type = "module";
	script.textContent = set_unique(template, eid);
	return script;
}

function initializeNav() {
	const icon_container = document.getElementById("icon_container");
	const modal_container = document.getElementById("modal_container");
	const view_container = document.getElementById("view_container");
	const script_container = document.getElementById("script_container");

	let scripts = [];

	for(let i = 0; i < settings.navbar.length; i++){
		const type = settings.navbar[i].type;
		const eid = settings.navbar[i].id;
		const container = settings.navbar[i].container_id;
		const template = element_templates[type];

		if(typeof container === 'undefined' || container == "icon_container")
			icon_container.appendChild(createElement(template.icon, eid));
		else if(typeof container !== 'undefined' && container !== "icon_container"){
			const container_element = document.getElementById(container);
			container_element.insertBefore(createElement(template.icon, eid), container_element.querySelector("[data-uniqueid='addbutton']"));
		}

		if (template.hasOwnProperty("modal"))
			modal_container.appendChild(createElement(template.modal, eid));

		if (template.hasOwnProperty("view"))
			view_container.appendChild(createElement(template.view, eid));

		if(template.hasOwnProperty("script"))
			scripts.push(createScript(template.script, eid));
	};

	const add = element_templates["add"];
	modal_container.appendChild(createElement(add.modal, "addbutton"));
	icon_container.appendChild(createElement(add.icon, "addbutton"));
	scripts.push(createScript(add.script, "addbutton"));

	scripts.forEach(element => {
		script_container.appendChild(element);
	});

	window.dispatchEvent(new Event("icons_changed"));
}


function loadAll(){
	Promise.all([elementTemplatesPromise]).then((values) => {
		element_templates = values[0];

		const modal_container = document.getElementById("modal_container");
		const view_container = document.getElementById("view_container");
		const script_container = document.getElementById("script_container");
	
		window.addEventListener("add_widget", (event) => {
			if(isNaN(uid)){
				if(isNaN(settings.uid))
					uid = 0;
				else
					uid = settings.uid;
			}

			const template = element_templates[event.widget_type];
			const eid = event.widget_type+"_autoID_" + uid++;
			settings.navbar.push({ type: event.widget_type, id: eid, container_id: event.container_target });
			settings.uid = uid;
			settings.save();
	
			const icon_element = createElement(template.icon, eid);
			icon_element.dataset.topic = event.widget_topic;

			const icon_container = document.getElementById(event.container_target);
			icon_container.insertBefore(icon_element, icon_container.querySelector("[data-uniqueid='addbutton']"));

			if (template.hasOwnProperty("modal"))
				modal_container.appendChild(createElement(template.modal, eid));

			if (template.hasOwnProperty("view"))
				view_container.appendChild(createElement(template.view, eid));
	
			if(template.hasOwnProperty("script"))
				script_container.appendChild(createScript(template.script, eid));

			window.dispatchEvent(new Event("icons_changed"));
		});
	
		window.addEventListener("remove_widget", (event) => {
			const uniqueID = event.uniqueID;
			const elementIndex = settings.navbar.findIndex((item) => item.id === uniqueID);
	
			if (elementIndex !== -1) {
				const elements = document.querySelectorAll(`[data-uniqueid="${uniqueID}"]`);
				elements.forEach((element) => {
					element.remove();
				});

				settings.navbar.splice(elementIndex, 1);
				delete settings[uniqueID];

				settings.save();
				window.dispatchEvent(new Event("icons_changed"));
			}
		});

		initializeNav();
	});
}

if (document.readyState !== 'loading') {
    loadAll();
} else {
    document.addEventListener('DOMContentLoaded', function () {
        loadAll();
    });
}