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

function get_prefix(words) {
	if (!words[0] || words.length == 1) return words[0] || '';
	let i = 0;
	while (words[0][i] && words.every(w => w[i] === words[0][i])) {
		i++;
	}
	return words[0].substr(0, i);
}

export function groupStringsByPrefix(strings, min_prefix_len=4, min_group_size=3) {

	let sorted = strings.sort();
	const prefixes = new Set();

	for (let i = 0; i < strings.length - min_group_size+1; i++) {
		let group = [];
		for (let j = 0; j < min_group_size; j++) {
			group.push(sorted[i + j]);
		}

		const prefix = get_prefix(group);
		if(prefix.length >= min_prefix_len){
			prefixes.add(prefix);
		}
	}

	const longest_prefix = Array.from(prefixes).sort((a, b) => b.length - a.length);

	let groups = {};

	for (let i = 0; i < longest_prefix.length; i++) {
		const prefix = longest_prefix[i];
		let namelist = [];

		for (let j = 0; j < sorted.length; j++) {
			if(sorted[j].indexOf(prefix) == 0){
				namelist.push(sorted[j]);
				sorted.splice(j, 1);
				j--;
			}
		}

		groups[prefix] = namelist;
	}
	
	sorted = sorted.concat(longest_prefix).sort();
	
	let returnarray = [];
	for (let i = 0; i < sorted.length; i++) {
		const name = sorted[i];
		if(name in groups){
			returnarray.push([name].concat(groups[name]));
		}  else{
			returnarray.push([name]);
		}
	}

	return returnarray;
}