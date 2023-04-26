import { view } from '/js/modules/view.js';
import { tf } from '/js/modules/tf.js';

const modal = document.getElementById("{uniqueID}_modal");

document.getElementById("{uniqueID}_reset_view").addEventListener("click", function(event){
    view.reset();
    modal.style.display = "none";
});