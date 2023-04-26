import { settings } from './persistent.js';

function clamp(val, from, to){
    if(val > to)
        return to;
    if(val < from)
        return from;
    return val;
}

const MAX_SCALE = 10000;
const MIN_SCALE = 1.0;
const ZOOM_FACTOR = 1.05;

export class View {

	constructor() {
		this.center = {
			x: 0,
			y: 0
		};
		this.scale = settings.view.scale;

		document.addEventListener("DOMContentLoaded", () => {
			this.addListeners();
		});
	}

	mapToScreen(coords) {
		return {
			x: (coords.x - this.center.x) * this.scale + window.innerWidth / 2,
			y: (coords.y - this.center.y) * this.scale + window.innerHeight / 2,
		}
	}
	
	screenToMap(coords) {
		return {
			x: (coords.x - window.innerWidth / 2) / this.scale + this.center.x,
			y: (coords.y - window.innerHeight / 2) / this.scale + this.center.y
		}
	}

	addListeners(){
		let view = document.getElementById("view_container");
		
		view.addEventListener('wheel', (event) => {
			let new_scale = this.scale * (-event.deltaY < 0 ? 1 / ZOOM_FACTOR : ZOOM_FACTOR);
			new_scale = clamp(new_scale, MIN_SCALE, MAX_SCALE);
		
			const screenPoint = {
				x: event.clientX,
				y: event.clientY,
			};
		
			const mapPoint = this.screenToMap(screenPoint);
			const scaleChange = new_scale / this.scale;
		
			const newCenter = {
				x: this.center.x + (mapPoint.x - this.center.x) * (1 - 1 / scaleChange),
				y: this.center.y + (mapPoint.y - this.center.y) * (1 - 1 / scaleChange),
			};
		
			this.center = newCenter;
			this.scale = new_scale;
		
			settings.view.scale = this.scale;
			settings.save();
		
			window.dispatchEvent(new Event("view_changed"));
		});
		
		view.addEventListener('mousedown', (event) => {
			this.drag_start = {
				x: event.pageX,
				y: event.pageY,
				ref_center_x: this.center.x,
				ref_center_y: this.center.y,
			};
		})

		view.addEventListener('mousemove', (event) => {
			if(this.drag_start === undefined)
				return;

			let delta = {
				x: this.drag_start.x - event.pageX,
				y: this.drag_start.y - event.pageY
			};

			this.center = {
				x: this.drag_start.ref_center_x + delta.x / this.scale,
				y: this.drag_start.ref_center_y + delta.y / this.scale,
			};

			window.dispatchEvent(new Event("view_changed"));
		})

		view.addEventListener('mouseup', (event) => {
			this.drag_start = undefined;
		})
	}

}

export let view = new View();