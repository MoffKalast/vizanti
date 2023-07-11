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
	return new Promise((resolve, reject) => {
		const img = new Image();		
		img.onload = () => {
			const canvas = document.createElement("canvas");
			canvas.width = img.width;
			canvas.height = img.height;

			const ctx = canvas.getContext("2d");
			ctx.drawImage(img, 0, 0);

			const dataURL = canvas.toDataURL();
			resolve(dataURL);
		};
		img.onerror = reject;
		img.src = url;
	});
}
  