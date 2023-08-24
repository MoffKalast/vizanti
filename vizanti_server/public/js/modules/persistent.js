export function saveJsonToFile(data, filename) {
	const jsonData = JSON.stringify(data, null, 2);
	const blob = new Blob([jsonData], { type: 'application/json' });
	const url = URL.createObjectURL(blob);
  
	const a = document.createElement('a');
	a.href = url;
	a.download = filename;
	a.click();
  
	URL.revokeObjectURL(url);
}

export class Settings {

	constructor() {
		if (localStorage.hasOwnProperty("settings")) {
			this.fromJSON(localStorage.getItem("settings"));
		}else{
			this.navbar = [
				{type:"settings", id:"settings_default"},
				{type:"rosbridge", id:"rosbridge_default"},
				{type:"grid", id:"grid_default"},
				{type:"tf", id:"tf_default"}
			];
			this.view = {
				center: {
					x: 0,
					y: 0
				},
				scale: 50.0
			};
		}
	}

	fromJSON(settings_object){
		let storedSettings = JSON.parse(settings_object);
		Object.assign(this, storedSettings);
	}

	save() {
		localStorage.setItem("settings", JSON.stringify(this));
	}
}

export let settings = new Settings();