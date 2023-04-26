export const elements =  {
	"settings": {
        "icon": "settings_icon.html",
		"modal": "settings_modal.html",
		"script": "settings_script.js"
    },
    "rosbridge": {
        "icon": "rosbridge_icon.html",
		"modal": "rosbridge_modal.html",
		"script": "rosbridge_script.js"
    },
	"battery": {
        "icon": "battery_icon.html",
		"modal": "battery_modal.html",
		"script": "battery_script.js"
    },
	"grid": {
        "icon": "grid_icon.html",
		"modal": "grid_modal.html",
		"view": "grid_view.html",
		"script": "grid_script.js"
    },
	"add": {
        "icon": "add_icon.html",
		"modal": "add_modal.html",
    }
}

export const elementTemplatesPromise = loadElementTemplates();

async function loadElementTemplates() {
	let templates = {};

	const fetchPromises = Object.keys(elements).map(async key => {
		const element = elements[key];

		templates[key] = {}

		// Icons are mandatory
		const icon_response = await fetch(`/templates/${element.icon}`);
		templates[key]["icon"] = await icon_response.text();

		if(element.hasOwnProperty("modal")){
			const modal_response = await fetch(`/templates/${element.modal}`);
			templates[key]["modal"] = await modal_response.text();
		}

		if(element.hasOwnProperty("view")){
			const view_response = await fetch(`/templates/${element.view}`);
			templates[key]["view"] = await view_response.text();
		}

		if(element.hasOwnProperty("script")){
			const script_response = await fetch(`/templates/${element.script}`);
			templates[key]["script"] = await script_response.text();
		}

	});

	await Promise.all(fetchPromises);
	return templates;
}