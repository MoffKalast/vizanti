import { rosbridge } from './rosbridge.js';

export class TF {

	constructor() {
		this.frame = "/odom";

	}

	set_frame(newframe){
		this.frame= newframe;
	}

}

export let tf = new TF();