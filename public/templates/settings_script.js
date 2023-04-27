import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';

const modal = document.getElementById("{uniqueID}_modal");

document.getElementById("{uniqueID}_reset_view").addEventListener("click", (event) =>{
	view.reset();
	modal.style.display = "none";
});

async function deletePersistent() {
	let del_ok = await confirm("Are you sure you want to delete your saved widget setup? This will refresh the page.");

	if(del_ok){
		localStorage.removeItem("settings");
		window.location.reload();
	}
}

document.getElementById("{uniqueID}_delete_persistent").addEventListener("click", deletePersistent);