import { settings } from './persistent.js';

function clamp(val, from, to){
    if(val > to)
        return to;
    if(val < from)
        return from;
    return val;
}

function hasClass(element, className) {
	if (element.classList) {
		return element.classList.contains(className);
	} else {
		return new RegExp('(^| )' + className + '( |$)', 'gi').test(element.className);
	}
}

function hasClassInParentChain(element, className) {
	if (!element) {
		return false;
	}

	if (hasClass(element, className)) {
		return true;
	}

	return hasClassInParentChain(element.parentElement, className);
}

const MAX_SCALE = 10000;
const MIN_SCALE = 3.0;
const ZOOM_FACTOR = 1.05;

export class View {

	constructor() {
		this.center = settings.view.center ?? {
			x: 0,
			y: 0
		};
		this.scale = settings.view.scale ?? 50.0;

		this.drag_start = undefined;
		this.input_movement = true;

		document.addEventListener("DOMContentLoaded", () => {
			this.addListeners();
		});

		this.event_timestamp = performance.now();
		this.event_timeout = undefined;
	}

	async sendUpdateEvent(){

		if (this.event_timeout !== undefined) {
			clearTimeout(this.event_timeout);
		}
		const delta = performance.now() - this.event_timestamp

		if(delta > 12){
			window.dispatchEvent(new Event("view_changed"));
			this.event_timestamp = performance.now();
		}else{
			this.event_timeout = setTimeout(() => {
				window.dispatchEvent(new Event("view_changed"));
			}, 12 - delta);
		}
	}

	setInputMovementEnabled(value){
		this.input_movement = value;
	}

	reset(){
		this.center = { x: 0,y: 0};
		this.scale = 50.0;
		this.drag_start = undefined;

		settings.view.center = this.center;
		settings.view.scale = this.scale;
		settings.save();
		this.sendUpdateEvent();
	}

	fixedToScreen(coords) {
		return {
			x: (coords.x - this.center.x) * this.scale + window.innerWidth / 2,
			y: (-coords.y - this.center.y) * this.scale + window.innerHeight / 2,
			z: 0
		}
	}
	
	screenToFixed(coords) {
		return {
			x: (coords.x - window.innerWidth / 2) / this.scale + this.center.x,
			y: -(coords.y - window.innerHeight / 2) / this.scale - this.center.y,
			z: 0
		}
	}

	getPixelsInMapUnits(length){
		let p1 = this.screenToFixed({
			x: 0,
			y: 0,
		});
	
		let p2 = this.screenToFixed({
			x: length,
			y: 0,
		});
	
		return Math.abs(p1.x-p2.x);
	}

	getMapUnitsInPixels(length){
		let p1 = this.fixedToScreen({
			x: 0,
			y: 0,
		});
	
		let p2 = this.fixedToScreen({
			x: length,
			y: 0,
		});
	
		return Math.abs(p1.x-p2.x);
	}
	

	handleDragStart(event) {
		if(!this.input_movement)
			return;

		const { clientX, clientY } = event.touches ? event.touches[0] : event;
		this.drag_start = {
			x: clientX,
			y: clientY,
			ref_center_x: this.center.x,
			ref_center_y: this.center.y,
		};
	}
	
	handleDragMove(event) {
		if (this.drag_start === undefined) return;

		if(!this.input_movement){
			this.drag_start = undefined;
			return;
		}

		if (hasClassInParentChain(event.target, 'inputelement')) {
			return;
		}
	
		const { clientX, clientY } = event.touches ? event.touches[0] : event;
		const delta = {
			x: this.drag_start.x - clientX,
			y: this.drag_start.y - clientY,
		};
	
		this.center = {
			x: this.drag_start.ref_center_x + delta.x / this.scale,
			y: this.drag_start.ref_center_y + delta.y / this.scale,
		};

		settings.view.center = this.center;
		settings.save();
		this.sendUpdateEvent();
	}
	
	handleDragEnd() {
		this.drag_start = undefined;
	}

	handleZoom(event) {
		if(!this.input_movement)
			return;

		let scaleChange;
		let centerX, centerY;
	
		if (event.touches) {
			if (event.touches.length !== 2) return;
			const [touch1, touch2] = event.touches;
			centerX = (touch1.clientX + touch2.clientX) / 2;
			centerY = (touch1.clientY + touch2.clientY) / 2;
			const dx = touch1.clientX - touch2.clientX;
			const dy = touch1.clientY - touch2.clientY;
			const distance = Math.sqrt(dx * dx + dy * dy);
			scaleChange = distance / this.initialTouchDistance;
			this.initialTouchDistance = distance;
			const new_scale = this.scale * scaleChange;
			this.scale = clamp(new_scale, MIN_SCALE, MAX_SCALE);
		} else {
			const new_scale = this.scale * (-event.deltaY < 0 ? 1 / ZOOM_FACTOR : ZOOM_FACTOR);
			const clamped_scale = clamp(new_scale, MIN_SCALE, MAX_SCALE);
			scaleChange = clamped_scale / this.scale;
			centerX = event.clientX;
			centerY = event.clientY;
			this.scale = clamped_scale;
		}
	
		const screenPoint = { x: centerX, y: centerY };
		const mapPoint = this.screenToFixed(screenPoint);
	
		const newCenter = {
			x: this.center.x + (mapPoint.x - this.center.x) * (1 - 1 / scaleChange),
			y: this.center.y + (-mapPoint.y - this.center.y) * (1 - 1 / scaleChange),
		};
	
		this.center = newCenter;
		settings.view.center = this.center;
		settings.view.scale = this.scale;
		settings.save();
		this.sendUpdateEvent();
	}

	addListeners(){
		let view = document.getElementById("view_container");
		
		view.addEventListener('mousedown', this.handleDragStart.bind(this));
		view.addEventListener('mousemove', this.handleDragMove.bind(this));
		view.addEventListener('mouseup', this.handleDragEnd.bind(this));
		
		view.addEventListener('touchstart', (event) => {
			if (event.touches.length === 2) {
				if (!this.touch1 || !this.touch2) {
					this.touch1 = {
						clientX: event.touches[0].clientX,
						clientY: event.touches[0].clientY,
					};
					this.touch2 = {
						clientX: event.touches[1].clientX,
						clientY: event.touches[1].clientY,
					};
					const dx = this.touch1.clientX - this.touch2.clientX;
					const dy = this.touch1.clientY - this.touch2.clientY;
					this.initialTouchDistance = Math.sqrt(dx * dx + dy * dy);
				}
			} else {
				this.touch1 = null;
				this.touch2 = null;
				this.handleDragStart(event);
			}
		});	

		view.addEventListener('touchmove', (event) => {
			if (event.touches.length === 2) {
				this.handleZoom(event);
			} else {
				this.handleDragMove(event);
			}
		});

		view.addEventListener('touchend', (event) => {
			this.touch1 = null;
			this.touch2 = null;
			this.handleDragEnd(event);
		});

		view.addEventListener('wheel', this.handleZoom.bind(this));
	}

}

export let view = new View();