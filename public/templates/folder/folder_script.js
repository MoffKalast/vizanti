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

function set_icons(){
	const icon_list = icon_container.getElementsByTagName('img');
	for(let i = 0; i < icon_list.length && i < 2; i++){
		subicons[i].style.display = "none";
		let entry = icon_list[i];
		if(!entry.src.endsWith("add.svg")){
			subicons[i].src = entry.src;
			subicons[i].style.display = "block";
			subicons[i].style.filter = entry.style.filter;
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
