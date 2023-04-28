export class Settings {
	constructor() {
		if (localStorage.hasOwnProperty("settings")) {
			let storedSettings = JSON.parse(localStorage.getItem("settings"));
			Object.assign(this, storedSettings);
		}
		else{
			this.navbar = [
				{type:"settings", id:"settings_default"},
				{type:"rosbridge", id:"rosbridge_default"},
				{type:"grid", id:"grid_default"},
				{type:"tf", id:"tf_default"}
			];
			this.view = {
				scale: 50.0
			};
		}
	}

	save() {
		localStorage.setItem("settings", JSON.stringify(this));
	}
}

export let settings = new Settings();