import files from '../../templates/files';

async function loadElementTemplates() {
	let templates = {};

	files.forEach(file => {
		const filePathParts = file.path.split('/');
		const fileName = filePathParts[filePathParts.length - 1];
		const [category, typeWithExtension] = fileName.split('_');
		const type = typeWithExtension.split('.')[0];

		if (!templates[category]) {
			templates[category] = {};
		}

		templates[category][type] = file.content;
	});

	console.table("Loaded templates:",templates)

	return templates;
}

export const elementTemplatesPromise = loadElementTemplates();