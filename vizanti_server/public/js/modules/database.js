export class IndexedDatabase {
	constructor(storeName) {
		this.storeName = storeName;
		this.db = null;
	}

	async openDB() {
		return new Promise((resolve, reject) => {
			const request = indexedDB.open("vizantiDB", 1);

			request.onupgradeneeded = (event) => {
				const db = event.target.result;
				db.createObjectStore(this.storeName);
			};

			request.onsuccess = (event) => {
				this.db = event.target.result;
				resolve();
			};

			request.onerror = (event) => {
				reject(event.error);
			};
		});
	}

	async keyExists(key) {
		return await this._performTransaction('readonly', (store) => {
			return store.getKey(key);
		});
	}

	async setObject(key, value) {
		await this._performTransaction('readwrite', (store) => {
			store.put(value, key);
		});
	}

	async getObject(key) {
		return await this._performTransaction('readonly', (store) => {
			return store.get(key);
		});
	}

	async _performTransaction(mode, operation) {
		return new Promise((resolve, reject) => {
			if (!this.db) {
				reject('IndexedDB not opened');
				return;
			}

			const transaction = this.db.transaction(this.storeName, mode);
			const store = transaction.objectStore(this.storeName);

			const request = operation(store);
			if (request) {
				request.onsuccess = () => {
					resolve(request.result);
				};
			} else {
				resolve();
			}

			transaction.onerror = () => {
				reject(transaction.error);
			};
		});
	}
}