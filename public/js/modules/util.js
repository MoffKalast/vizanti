export async function toDataURL(url) {
	const response = await fetch(url);
	const blob = await response.blob();
	return new Promise((resolve, reject) => {
		const reader = new FileReader();
		reader.onloadend = () => resolve(reader.result);
		reader.onerror = reject;
		reader.readAsDataURL(blob);
	});
}

//webkit http compatible fetch-free implementation
export function imageToDataURL(url) {
	if(url.toLowerCase().endsWith(".svg")){
		return new Promise((resolve, reject) => {
			const object = document.createElement("object");
			object.style.position = "absolute";
  			object.style.left = "-9999px";//trick chrome to hide the elements, since it doesn't fetch if display is set to none
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
		img.onerror = reject;
		img.src = url;
	});
}