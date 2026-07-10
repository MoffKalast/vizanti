import { IndexedDatabase } from './database.js';
import { imageToDataURL } from './util.js';

const db = new IndexedDatabase('tile_data');
await db.openDB();

// WGS84 ellipsoid constants
const WGS84_A = 6378137.0;
const WGS84_F = 1.0 / 298.257223563;
const WGS84_E2 = WGS84_F * (2.0 - WGS84_F);
const WGS84_B = WGS84_A * (1.0 - WGS84_F);
const WGS84_EP2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B);
const D2R = Math.PI / 180.0;
const R2D = 180.0 / Math.PI;

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
	static coordToTile(lon, lat, zoom) {
		return {
			y:(Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) + 1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * Math.pow(2, zoom))),
			x: (Math.floor((lon + 180) / 360 * Math.pow(2, zoom)))
		};
	}

	static tileToCoord(x, y, z) {
		var n = Math.PI - 2 * Math.PI * y / Math.pow(2, z);
		return {
			latitude:(180 / Math.PI * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)))),
			longitude: (x / Math.pow(2, z) * 360 - 180)
		};
	}

	/* ---------------------------------------------------------------------------
	   WGS84 geodetic <-> ECEF <-> local ENU (topocentric tangent plane).

	   This is the same projection a typical GNSS backend uses
	   (pyproj +proj=cart +step +proj=topocentric, geodesy "small" libraries, etc.),
	   verified to agree with pyproj to < 1e-9 m. Using it for both tile placement
	   (forward) and screen culling (inverse) makes the renderer exactly
	   self-consistent with a metric ENU TF tree anchored at the fix origin.
	--------------------------------------------------------------------------- */

	static llaToEcef(latDeg, lonDeg, alt = 0) {
		const lat = latDeg * D2R;
		const lon = lonDeg * D2R;
		const sLat = Math.sin(lat), cLat = Math.cos(lat);
		const sLon = Math.sin(lon), cLon = Math.cos(lon);
		const N = WGS84_A / Math.sqrt(1.0 - WGS84_E2 * sLat * sLat);
		return {
			x: (N + alt) * cLat * cLon,
			y: (N + alt) * cLat * sLon,
			z: (N * (1.0 - WGS84_E2) + alt) * sLat
		};
	}

	// Bowring's closed-form approximation, sub-millimeter accurate near the surface.
	static ecefToLla(x, y, z) {
		const p = Math.hypot(x, y);
		const th = Math.atan2(WGS84_A * z, WGS84_B * p);
		const sth = Math.sin(th), cth = Math.cos(th);
		const lat = Math.atan2(
			z + WGS84_EP2 * WGS84_B * sth * sth * sth,
			p - WGS84_E2 * WGS84_A * cth * cth * cth
		);
		const lon = Math.atan2(y, x);
		const sLat = Math.sin(lat);
		const N = WGS84_A / Math.sqrt(1.0 - WGS84_E2 * sLat * sLat);
		const alt = p / Math.cos(lat) - N;
		return { latitude: lat * R2D, longitude: lon * R2D, altitude: alt };
	}

	// Build an ENU origin object to pass into llaToEnu / enuToLla / enuGroundToLla.
	// Each satellite_script instance should hold its own origin and pass it in,
	// so multiple instances never clobber each other.
	static buildEnuOrigin(latDeg, lonDeg, alt = 0) {
		const lat = latDeg * D2R;
		const lon = lonDeg * D2R;
		return {
			latitude: latDeg,
			longitude: lonDeg,
			altitude: alt,
			ecef: Navsat.llaToEcef(latDeg, lonDeg, alt),
			sLat: Math.sin(lat), cLat: Math.cos(lat),
			sLon: Math.sin(lon), cLon: Math.cos(lon)
		};
	}

	// geodetic -> ENU meters relative to the provided origin.
	// Returns {x: east, y: north, z: up}.
	static llaToEnu(latDeg, lonDeg, alt, origin) {
		if (alt === undefined) alt = origin.altitude;
		const p = Navsat.llaToEcef(latDeg, lonDeg, alt);
		const dx = p.x - origin.ecef.x;
		const dy = p.y - origin.ecef.y;
		const dz = p.z - origin.ecef.z;
		return {
			x: -origin.sLon * dx + origin.cLon * dy,                                          // east
			y: -origin.sLat * origin.cLon * dx - origin.sLat * origin.sLon * dy + origin.cLat * dz,  // north
			z:  origin.cLat * origin.cLon * dx + origin.cLat * origin.sLon * dy + origin.sLat * dz   // up
		};
	}

	// Full 3D ENU -> geodetic.
	static enuToLla(e, n, u, origin) {
		const dx = -origin.sLon * e - origin.sLat * origin.cLon * n + origin.cLat * origin.cLon * u;
		const dy =  origin.cLon * e - origin.sLat * origin.sLon * n + origin.cLat * origin.sLon * u;
		const dz =                    origin.cLat * n               + origin.sLat * u;
		return Navsat.ecefToLla(origin.ecef.x + dx, origin.ecef.y + dy, origin.ecef.z + dz);
	}

	// Horizontal ENU (e, n) -> geodetic lat/lon of the point at the given
	// ellipsoidal height (defaults to origin height). This is the exact inverse
	// of llaToEnu for the horizontal components; converges to sub-mm in 2 iterations.
	static enuGroundToLla(e, n, origin, alt = undefined) {
		if (alt === undefined) alt = origin.altitude;
		// initial guess: requested height minus tangent-plane curvature drop
		let u = (alt - origin.altitude) - (e * e + n * n) / (2.0 * WGS84_A);
		let lla = Navsat.enuToLla(e, n, u, origin);
		for (let i = 0; i < 2; i++) {
			u += alt - lla.altitude;
			lla = Navsat.enuToLla(e, n, u, origin);
		}
		return lla;
	}

	/* --------------------------------------------------------------------------- */

	static metersToDegrees(meters, latitude, zoomLevel, tile_size = 256) {
		const earthRadius = 6378137;
		const metersPerPixel = (2 * Math.PI * earthRadius * Math.cos(latitude * Math.PI / 180)) / (tile_size * Math.pow(2, zoomLevel));
		return meters / metersPerPixel;
	}

	// Approximate (spherical) tile ground size; fine for zoom heuristics.
	static tileSizeInMeters(latitude, zoom, tile_size = 256) {
		const earthCircumference = 40075016.686;
		const distancePerPixel = (earthCircumference * Math.cos(latitude * Math.PI / 180)) / (Math.pow(2, zoom + 8));
		return tile_size * distancePerPixel;
	}

	static tileSizeInDegrees(latitude, zoom, tile_size = 256) {
		const degreesPerPixel = 360 / Math.pow(2, zoom);
		const latRadians = (latitude * Math.PI) / 180;
		const latRadiansPerPixel = Math.PI / (Math.pow(2, zoom) * tile_size);
		const latDegreesPerPixel = (180 / Math.PI) * (2 * Math.atan(Math.exp(latRadians + latRadiansPerPixel)) - Math.PI / 2) - latitude;
		return {
			latitude: tile_size * latDegreesPerPixel,
			longitude: tile_size * degreesPerPixel,
		};
	}

	static haversine(lat1, lon1, lat2, lon2) {
		const toRad = (value) => (value * Math.PI) / 180;
		const R = 6378137;
		const dLat = toRad(lat2 - lat1);
		const dLon = toRad(lon2 - lon1);
		const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
		const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
		return R * c;
	}
}
export let navsat = new Navsat();