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

export async function exportDatabase(filename) {
    const allData = [];
    
    async function dumpData() {
        const keylist = await db.getAllKeys();
        await Promise.all(keylist.map(async (url) => {
            const value = await db.getObject(url);
            allData.push({
                key: url,
                value: value
            });
        }));
    }
    
    await dumpData();
  
    const dataBlob = new Blob([JSON.stringify(allData)], {type: 'application/json'});
    
    const url = URL.createObjectURL(dataBlob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    a.click();
    
    URL.revokeObjectURL(url);
}

export async function importDatabase(file) {
	const data = JSON.parse(file);
	try {
		await Promise.all(data.map(item => db.setObject(item.key, item.value)));
		alert("Tiles imported successfully! (it might take a bit for them to save to disk and become available, be patient)");
	} catch (error) {
		console.error(error);
		alert("An error occurred during import. Please check the console for details.");
	}
}
  
export class Navsat {

	constructor (){
		this.tile_size = 256;
		this.live_cache = {};
		this.queue = new Set();

		this.download_queue = new Set();
		this.currently_downloading = new Set();

		this.MAX_CONCURRENT_DOWNLOADS = 6;
		this.loopRunning = false;

		this.kickLoadLoop = async () => {
			if (this.loopRunning)
				return;

			this.loopRunning = true;
			while (this.queue.size > 0 || this.download_queue.size > 0) {

				if (this.queue.size > 0) {
					const dbChecks = Array.from(this.queue).map(async (tile_url) => {
						if (this.live_cache[tile_url] !== undefined) {
							this.queue.delete(tile_url);
							return;
						}
						if (Boolean(await db.keyExists(tile_url))) {
							const data = await db.getObject(tile_url);
							this.live_cache[tile_url] = await dataToImage(data);
							this.queue.delete(tile_url);
							window.dispatchEvent(new Event("navsat_tilecache_updated"));
							return;
						}
						this.download_queue.add(tile_url);
						this.queue.delete(tile_url);
					});
					await Promise.all(dbChecks);
				}

				const slots = this.MAX_CONCURRENT_DOWNLOADS - this.currently_downloading.size;
				if (slots > 0) {
					const pending = Array.from(this.download_queue).filter(url => !this.currently_downloading.has(url)).slice(0, slots);
					for (const tile_url of pending) {
						this.currently_downloading.add(tile_url);
						this.attemptDownload(tile_url);
					}
				}

				// yield to the event loop between batches so we don't block rendering
				await new Promise(resolve => setTimeout(resolve, 0));
			}

			this.loopRunning = false;
		};
	}

	async attemptDownload(tile_url, attempt = 0) {
		const MAX_ATTEMPTS = 3;
		const timeout = new Promise(resolve => setTimeout(() => resolve(undefined), 4000));
		const data = await Promise.race([imageToDataURL(tile_url), timeout]);

		if (data) {
			await db.setObject(tile_url, data);
			this.live_cache[tile_url] = await dataToImage(data);
			this.download_queue.delete(tile_url);
			this.currently_downloading.delete(tile_url);
			window.dispatchEvent(new Event("navsat_tilecache_updated"));
			return;
		}

		if (attempt < MAX_ATTEMPTS - 1) {
			setTimeout(() => this.attemptDownload(tile_url, attempt + 1), 1000 * (attempt + 1));
		} else {
			// give up, remove from all queues so it can be re-enqueued
			this.download_queue.delete(tile_url);
			this.currently_downloading.delete(tile_url);
		}
	}

	enqueue(keyurl) {
		if (this.live_cache[keyurl] !== undefined || this.queue.has(keyurl) || this.download_queue.has(keyurl) || this.currently_downloading.has(keyurl))
			return;

		this.queue.add(keyurl);
		this.kickLoadLoop();
	}

	clear_queue() {
		this.queue = new Set();
		this.download_queue = new Set();
		this.currently_downloading = new Set();
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
		return meters / metersPerPixel;
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