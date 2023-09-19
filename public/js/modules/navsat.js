import { IndexedDatabase } from './database.js';
import { imageToDataURL } from './util.js';

const db = new IndexedDatabase('tile_data');
await db.openDB();

async function dataToImage(data){
	return new Promise((resolve, reject) => {
		let image = new Image();
		image.onload = () => resolve(image);
		image.onerror = reject;
		image.src = data;
	})
}

export class Navsat {

	constructor (){
		this.tile_size = 256;
		this.live_cache = {};
		this.queue = new Set();
		this.queue_history = new Set();

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

		}, 300);
	}

	async enqueue(keyurl){
		if(this.queue_history.has(keyurl))
			return;

		this.queue_history.add(keyurl);

		if(Boolean(await db.keyExists(keyurl))){
			const data = await db.getObject(keyurl);
			this.live_cache[keyurl] = await dataToImage(data);
			//console.log("Tile loaded from DB",keyurl)
		}else{
			//console.log("Tile Queued",keyurl)
			this.queue.add(keyurl);
		}
	}

	//https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
	coordToTile(lon, lat, zoom) {
		return {
			y:(Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) + 1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * Math.pow(2, zoom))),
			x: (Math.floor((lon + 180) / 360 * Math.pow(2, zoom)))
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
		console.log("Fetching tile data:",url);

		const data = await imageToDataURL(url);
		if (!data) {
			return undefined;
		}

		db.setObject(url, data);
		return data;
	}

	async loadTile(keyurl) {
		const data = await this.fetchURL(keyurl);
		if(data){
			let image = new Image();
			image.src = data;
			return new Promise((resolve, reject) => {
				image.onload = () => resolve(image);
				image.onerror = () => resolve(undefined);
			});
		}
		return undefined;
	}

	metersToDegrees(meters, latitude, zoomLevel) {
		const earthRadius = 6378137; // Earth radius in meters
	
		// Calculate the number of meters per pixel at the given latitude and zoom level
		const metersPerPixel = (2 * Math.PI * earthRadius * Math.cos(latitude * Math.PI / 180)) / (this.tile_size * Math.pow(2, zoomLevel));
	
		// Convert the distance in meters to degrees	
		return meters / metersPerPixel;;
	}

	tileSizeInMeters(latitude, zoom) {
		const earthCircumference = 40075016.686; // Earth circumference in meters
		
		// Calculate the horizontal distance per pixel in meters
		const distancePerPixel = (earthCircumference * Math.cos(latitude * Math.PI / 180)) / (Math.pow(2, zoom + 8));
	
		return this.tile_size * distancePerPixel;
	}

	tileSizeInDegrees(latitude, zoom) {
		const degreesPerPixel = 360 / Math.pow(2, zoom); // Calculate the degrees per pixel for longitude
	
		// Calculate the latitude degrees per pixel based on the latitude
		const latRadians = (latitude * Math.PI) / 180;
		const latRadiansPerPixel = Math.PI / (Math.pow(2, zoom) * this.tile_size);
		const latDegreesPerPixel = (180 / Math.PI) * (2 * Math.atan(Math.exp(latRadians + latRadiansPerPixel)) - Math.PI / 2) - latitude;
	
		return {
			latitude: this.tile_size * latDegreesPerPixel,
			longitude: this.tile_size * degreesPerPixel,
		};
	}

	haversine(lat1, lon1, lat2, lon2) {
		const toRad = (value) => (value * Math.PI) / 180;
		const R = 6378137; // Earth radius in meters
		const dLat = toRad(lat2 - lat1);
		const dLon = toRad(lon2 - lon1);
		const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
		const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
		return R * c;
	}	
}
export let navsat = new Navsat();