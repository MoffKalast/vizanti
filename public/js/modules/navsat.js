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

		this.loadingloop = async ()=>{

			while(this.queue.size > 0){
				let items = Array.from(this.queue);
				let tile_url = items[0];
				let loaded = undefined;

				//we already got it?
				if(this.live_cache[tile_url] !== undefined){
					this.queue.delete(tile_url);
					continue;
				}

				//check the indexed DB
				if(Boolean(await db.keyExists(tile_url))){
					const data = await db.getObject(tile_url);
					loaded = await dataToImage(data);
				}else{
					//download from tile server, in case there's no internet we don't hang forever
					const timeout = new Promise(resolve => setTimeout(() => resolve(undefined), 4000));
					const data = await Promise.race([imageToDataURL(tile_url), timeout]);

					if(data){
						//info.downloaded++;
						db.setObject(tile_url, data);
						loaded = await dataToImage(data);
					}
				}

				if(loaded){
					this.live_cache[tile_url] = loaded;
					this.queue.delete(tile_url);
				}
			}

			setTimeout(this.loadingloop, 500);
		}
		this.loadingloop();
	}

	async enqueue(keyurl){
		if(this.queue_history.has(keyurl))
			return;

		this.queue_history.add(keyurl);
		this.queue.add(keyurl);
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