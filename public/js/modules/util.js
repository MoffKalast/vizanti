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