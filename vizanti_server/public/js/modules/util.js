//webkit http compatible fetch-free implementation
export function imageToDataURL(url) {
	if(url.toLowerCase().endsWith(".svg")){
		return new Promise((resolve, reject) => {
			const object = document.createElement("object");
			object.style.position = "absolute";
  			object.style.left = "-9999px";//trick chrome to hide the elements, since it doesn't fetch if display is set to none
			object.style.visibility = "hidden";
			object.type = "image/svg+xml";
			object.data = url;
			object.onload = () => {
				const svgElement = object.contentDocument.documentElement;
				const svgCode = new XMLSerializer().serializeToString(svgElement);

				const dataURL = "data:image/svg+xml;base64," + btoa(svgCode);
				resolve(dataURL);
				document.body.removeChild(object);
			};
			document.body.appendChild(object);
		});
	}
	return new Promise((resolve, reject) => {
		const img = new Image();	
		img.crossOrigin = "anonymous";
		img.onload = () => {
			const canvas = document.createElement("canvas");

			canvas.width = img.width;
			canvas.height = img.height;

			const ctx = canvas.getContext("2d");
			ctx.drawImage(img, 0, 0);
			resolve(canvas.toDataURL());
		};
		img.onerror = (error) => {
			setTimeout(() => {
				img.src = url;
			}, 1000);
		};

		img.src = url;
	});
}

export function groupStringsByPrefix(strings, minPrefixLength=2) {

	function getPrefix(str, minPrefixLength) {
		const prefixEnd = str.indexOf('/') === -1 ? str.indexOf('_') : str.indexOf('/');
		const prefix = prefixEnd === -1 ? str : str.slice(0, prefixEnd);
		return prefix.length >= minPrefixLength ? prefix : null;
	}

	const prefixMap = new Map();
	for (const str of strings) {
		const prefix = getPrefix(str, minPrefixLength);
		if (!prefixMap.has(prefix)) {
			prefixMap.set(prefix, []);
		}
		prefixMap.get(prefix).push(str);
	}

	let result = [];
	for (const [prefix, group] of prefixMap) {
		if(group.length == 1){
			result.push([group[0]]);
		}else{
			result.push([prefix, ...group]);
		}
	}
	 
	return result.sort((a,b) => {
		if (a[0] < b[0]) return -1;
		if (a[0] > b[0]) return 1;
		return 0;
	});
}