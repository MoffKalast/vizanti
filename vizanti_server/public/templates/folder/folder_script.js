let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let rosbridge = rosbridgeModule.rosbridge;
let settings = persistentModule.settings;
let imageToDataURL = utilModule.imageToDataURL;
let Status = StatusModule.Status;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const icondiv = document.getElementById("{uniqueID}_icon");
const icon = icondiv.getElementsByTagName('img')[0];
const icon_text = icondiv.getElementsByTagName('p')[0];
const icon_container = document.getElementById("{uniqueID}_widget_container");
const remove_button = document.getElementById("{uniqueID}_remove");

const subicons = [
	document.getElementById("{uniqueID}_subicon0"),
	document.getElementById("{uniqueID}_subicon1")
];

const subtextss = [
	document.getElementById("{uniqueID}_subtext0"),
	document.getElementById("{uniqueID}_subtext1")
];

const observers = [];

function set_icons(){

	for(const observer of observers) {
		observer.disconnect();
	}
	observers.length = 0;

	let sources = [];
	const icon_list = icon_container.getElementsByClassName('icon');
	for(let i = 0; i < icon_list.length; i++){
		const firstChild = icon_list[i].firstElementChild;
        if (firstChild) {
            if (firstChild.tagName.toLowerCase() === 'img') {
                sources.push([firstChild, firstChild.src]);
            } else if (firstChild.tagName.toLowerCase() === 'object') {
                sources.push([firstChild, firstChild.data]);
            }
        }
	}

	for(let i = 0; i < sources.length && i < 2; i++){
		const source_obj = sources[i][0];
		const source_img_string = sources[i][1];

		subicons[i].style.display = "none";
		if(!source_img_string.endsWith("add.svg")){
			subicons[i].style.display = "block";

			//change colour if defined
			if(subicons[i].data != source_img_string){
				subicons[i].data = source_img_string;
				if(source_obj.hasAttribute("data-color")){
					subicons[i].onload = ()=>{
						utilModule.setIconColor(subicons[i], source_obj.dataset.color);
					};
				}
			}

			//display text if defined
			if(source_obj.dataset.text){
				subtextss[i].innerText = source_obj.dataset.text;
			}else{
				subtextss[i].innerText = "";
			}

			//watch for changes of all renderable params
			const observer = new MutationObserver((mutationsList) => {
				for (const mutation of mutationsList) {
					if (mutation.type === 'attributes' && mutation.attributeName === 'data-color') {
						utilModule.setIconColor(subicons[i], source_obj.dataset.color);
					}
					else if (mutation.type === 'attributes' && mutation.attributeName === 'data-text') {
						subtextss[i].innerText = source_obj.dataset.text;
					}
					else if (mutation.type === 'attributes' && mutation.attributeName === 'src') {
						subicons[i].data = source_obj.src;
					}
					else if (mutation.type === 'attributes' && mutation.attributeName === 'data') {
						subicons[i].data = source_obj.data;
					}
				}
			});
			observer.observe(source_obj, { attributes: true, attributeFilter: ['src', 'data-color', 'data-text', 'data'] });
			observers.push(observer);
		}
	}
}

//updating the two icons when any are added
const observer = new MutationObserver(set_icons);
observer.observe(icon_container, { childList: true });

//on click
icon.addEventListener("click", set_icons);

//and once after start
setTimeout(set_icons, 200);

remove_button.addEventListener('click', async () => {
	if(await confirm("Are you sure you want to delete this folder (all widgets inside it will be deleted too)?")){

		Array.from(icon_container.children).forEach(child => {
			const id = child.getAttribute('data-uniqueid');
			if(id != "addbutton"){
				removeWidget(id);
			}
		});
		removeWidget('{uniqueID}');
	}
});

console.log("Folder Loaded {uniqueID}")