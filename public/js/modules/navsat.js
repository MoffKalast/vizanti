import { IndexedDatabase } from './database.js';

const db = new IndexedDatabase('tiles');
await db.openDB();

export class Navsat {

	constructor (){
		this.live_cache = {};
		this.queue = new Set();

		this.loadingloop = setInterval(async ()=>{

			if(this.queue.size == 0)
				return;

			let items = Array.from(this.queue);
			let tile = items[Math.floor(Math.random() * items.length)];

			let loaded = await this.loadTile(tile);
			if(loaded){
				this.live_cache[tile] = loaded;
				this.queue.delete(tile);
			}

		}, 1000);
	}

	enqueue(keyurl){
		this.queue.add(keyurl);
	}

	//https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
	coordToTile(lon, lat, zoom) {
		return {
			latitude:(Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) + 1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * Math.pow(2, zoom))),
			longitude: (Math.floor((lon + 180) / 360 * Math.pow(2, zoom)))
		};
	}

	tileToCoord(x, y, z) {
		var n = Math.PI - 2 * Math.PI * y / Math.pow(2, z);
		return {
			latitude:(180 / Math.PI * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)))),
			longitude: (x / Math.pow(2, z) * 360 - 180)
		};
	}

	async fetchURL(url){
		if(Boolean(await db.keyExists(url))){
			const arrayBuffer = await db.getObject(url);
			return new Blob([arrayBuffer], { type: 'image/png' });
		}

		console.log("Fetching tile data:",url)

		const response = await fetch(url);
		if (!response.ok) {
			return undefined;
		}

		const blob = await response.blob();
		const arrayBuffer = await blob.arrayBuffer();
		db.setObject(url, arrayBuffer);

		return blob;
	}

	async loadTile(keyurl) {
		const blob = await this.fetchURL(keyurl);
		let image;
		if(blob){
			console.log("Loading tile",keyurl);
			image = new Image();
			image.src = URL.createObjectURL(blob);
			return new Promise((resolve, reject) => {
				image.onload = () => resolve(image);
				image.onerror = () => resolve(undefined);
			});
		}
		return undefined;
	}

	tileSizeInMeters(latitude, zoom) {
		const earthCircumference = 40075016.686; // Earth circumference in meters
		const tileSizePixels = 256; // Tile size in pixels (assuming 256x256 tiles)
		
		// Calculate the horizontal distance per pixel in meters
		const distancePerPixel = (earthCircumference * Math.cos(latitude * Math.PI / 180)) / (Math.pow(2, zoom + 8));
	
		return tileSizePixels * distancePerPixel;
	}

	haversine(lat1, lon1, lat2, lon2) {
		const toRad = (value) => (value * Math.PI) / 180;
		const R = 6371000; // Earth radius in meters
		const dLat = toRad(lat2 - lat1);
		const dLon = toRad(lon2 - lon1);
		const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
		const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
		return R * c;
	}
	
}
export let navsat = new Navsat();