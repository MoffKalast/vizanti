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

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const icon_container = document.getElementById("{uniqueID}_widget_container");
const remove_button = document.getElementById("{uniqueID}_remove");

const subicons = [
	document.getElementById("{uniqueID}_subicon0"),
	document.getElementById("{uniqueID}_subicon1")
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
		subicons[i].style.display = "none";
		if(!sources[i][1].endsWith("add.svg")){
			subicons[i].style.display = "block";

			if(subicons[i].data != sources[i][1]){
				subicons[i].data = sources[i][1];
				if(sources[i][0].hasAttribute("data-color")){
					subicons[i].onload = ()=>{
						utilModule.setIconColor(subicons[i], sources[i][0].dataset.color);
						console.log(subicons[i].dataset.color)
					};
				}
			}

			const observer = new MutationObserver((mutationsList) => {
				for(const mutation of mutationsList) {
					if(mutation.type === 'attributes' && mutation.attributeName === 'data-color') {
						utilModule.setIconColor(subicons[i], sources[i][0].dataset.color);
					}
				}
			});

			observer.observe(sources[i][0], {attributes: true, attributeFilter: ['data-color']});
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