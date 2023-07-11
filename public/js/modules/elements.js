import fileList from '/templates/files' assert {type: 'json'};

async function loadElementTemplates() {
    let templates = {};    

    const fetchPromises = fileList.map(async file => {
        const filePathParts = file.split('/');
        const fileName = filePathParts[filePathParts.length - 1];
        const [category, typeWithExtension] = fileName.split('_');
        const type = typeWithExtension.split('.')[0];

        if (!templates[category]) {
            templates[category] = {};
        }

        const response = await fetch(`/templates/${file}`);
        if (response.ok) {
            templates[category][type] = await response.text();
        }
    });

    await Promise.all(fetchPromises);
	console.table("Loaded templates:",templates)

    return templates;
}

export const elementTemplatesPromise = loadElementTemplates();