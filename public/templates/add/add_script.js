let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);

let rosbridge = rosbridgeModule.rosbridge;

const typeButton = document.getElementById('add_set_type');
const topicButton = document.getElementById('add_set_topics');

const topicDiv = document.getElementById('add_topics_container');
const typeDiv = document.getElementById('add_types_container');

let types = {};

//gather the cards
const widgets = typeDiv.children;
for (let i = 0; i < widgets.length; i++) {
	let topics = widgets[i].dataset.topic;
	if(topics !== undefined){
		topics.split(",").forEach(topic => {
			if(topic !== undefined){
				if(topic in types){
					types[topic].push(widgets[i]);
				}else{
					types[topic] = [widgets[i]];
				}
			}
		});
	}
}

//tabs swapping
typeButton.addEventListener("click", (event) => {
	topicButton.classList.remove("active-tab");
	typeButton.classList.add("active-tab");

	typeDiv.style.display = "block";
	topicDiv.style.display = "none";
});

topicButton.addEventListener("click", (event) => {
	update_topics();
	typeButton.classList.remove("active-tab");
	topicButton.classList.add("active-tab");

	typeDiv.style.display = "none";
	topicDiv.style.display = "block";
});

// rebuild topic list
async function update_topics(){
	let result = await rosbridge.get_all_topics();

	topicDiv.innerHTML = "";
	for (let i = 0; i < result.types.length; i++) {
		let type = result.types[i];
		if(type in types){
			types[type].forEach(element => {
				let newnode = element.cloneNode(true);
				let title = newnode.querySelector('.card_title');
				let desc = newnode.querySelector('.card_desc');
				
				newnode.setAttribute('onclick', newnode.getAttribute("onclick").replace(",\'\')",",\'"+result.topics[i]+"\')"));

				desc.innerText = title.innerText + " ["+type + "]";
				title.innerText = result.topics[i];
				topicDiv.appendChild(newnode);
			});
		}
	}
}


update_topics();

console.log("Add Widgets Loaded.");