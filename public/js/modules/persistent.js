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