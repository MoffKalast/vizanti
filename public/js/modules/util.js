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

	function splitPrefix(string_list) {		
		class Node {
			constructor() {
				this.list = {};
			}
		}

		const root = new Node();
		for (const i of string_list) {
			const replaced = i.replaceAll("/","@@/")
							.replaceAll("_","@@_")
							.replaceAll("-","@@-");

			const path = replaced.split("@@");

			let currentNode = root;
			for(let j = 0; j < path.length; j++){
				const segment = path[j];
				if (!(segment in currentNode.list))
					currentNode.list[segment] = new Node();
				currentNode = currentNode.list[segment];
			}

			currentNode.link_name = i;
		}		
		return root;
	}

	function getLinks(tree){
		let links = "link_name" in tree ? [tree.link_name] : [];

		for (const [key, value] of Object.entries(tree.list)) {
			links = links.concat(getLinks(value));
		}

		return links;
	}

	function getGroups(tree){
		let groups = {};
		for (const [key, value] of Object.entries(tree.list)) {
			const links = getLinks(value);

			if(links.length < 8){
				groups[key] = links;
			}else{
				let subgroups = getGroups(value);
				for (const [subkey, subvalue] of Object.entries(subgroups)) {
					if(subvalue.length > 2){
						groups[key+subkey] = subvalue;
					}else{
						if(key in groups)
							groups[key] = groups[key].concat(subvalue);
						else
							groups[key] = subvalue;
					}					
				}
			}
		}
		return groups;
	}

	let fragmented = splitPrefix(strings);
	let dict = getGroups(fragmented);

	//format for rendering
	let result = [];
	for (const [key, value] of Object.entries(dict)) {
		if (value.length > 1)
			result.push([key, ...value])
		else
			result.push(value)
	}

	return result.sort((a,b) => {
		if (a[0] < b[0]) return -1;
		if (a[0] > b[0]) return 1;
		return 0;
	});
}

export function hexColourToIconFilter(hex) {
	hex = hex.replace(/^#/, '');
	let bigint = parseInt(hex, 16);
	let r = (bigint >> 16) & 255;
	let g = (bigint >> 8) & 255;
	let b = bigint & 255;

	r /= 255;
	g /= 255;
	b /= 255;

	let max = Math.max(r, g, b);
	let min = Math.min(r, g, b);
	let delta = max - min;
	let h, s, v = max;
	let l = (min+max)/2.0;

	if (delta === 0) {
		h = 0;
	} else if (max === r) {
		h = ((g - b) / delta) % 6;
	} else if (max === g) {
		h = (b - r) / delta + 2;
	} else {
		h = (r - g) / delta + 4;
	}

	h = Math.round(h * 60);
	if (h < 0) h += 360;

	s = max === 0 ? 0 : delta / max;

	let saturate = s * 100;
	let brightness = l * 600;

	//icons are assumed have hue=0 (red) saturation=100 value=100
	return `hue-rotate(${h}deg) saturate(${saturate}%) brightness(${brightness}%) contrast(250%)`;
}