let viewModule = await import(`${base_url}/js/modules/view.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);

let view = viewModule.view;
let settings = persistentModule.settings;
let Status = StatusModule.Status;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d');

let grid_size = 1.0;
let grid_thickness = 1;
let grid_colour = "#3e556a";

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	grid_size = loaded_data.size;
	grid_thickness = loaded_data.thickness;
	grid_colour = loaded_data.colour;
}else{
	saveSettings();
}

function saveSettings(){
	settings["{uniqueID}"] = {
		size: grid_size,
		thickness: grid_thickness,
		colour: grid_colour
	}
	settings.save();
}

async function drawGrid() {
    const wid = canvas.width;
    const hei = canvas.height;

    ctx.strokeStyle = grid_colour;
	ctx.lineWidth = grid_thickness;

	const topLeft = view.screenToFixed({ x: 0, y: 0 });
	const bottomRight = view.screenToFixed({ x: wid, y: hei });
  
	const minX = topLeft.x - (topLeft.x % grid_size) - grid_size;
	const maxX = bottomRight.x + (grid_size - (bottomRight.x % grid_size));
  
	const minY = bottomRight.y - (bottomRight.y % grid_size) - grid_size;
	const maxY = topLeft.y + (grid_size - (topLeft.y % grid_size));

	const linesX = (maxX-minX)/grid_size;
	const linesY = (maxY-minY)/grid_size;

	if(linesX > 200 || linesY > 200){
		ctx.clearRect(0, 0, wid, hei);
		status.setWarn("Too many lines to render, increase step size.");
		return;
	}

	ctx.beginPath();

    // Draw vertical lines
    for (let x = minX; x <= maxX; x += grid_size) {
        let from = view.fixedToScreen({
            x: x,
            y: minY,
        });

        let to = view.fixedToScreen({
            x: x,
            y: maxY,
        });

        ctx.moveTo(parseInt(from.x), parseInt(from.y));
        ctx.lineTo(parseInt(to.x), parseInt(to.y));
    }

    // Draw horizontal lines
    for (let y = minY; y <= maxY; y += grid_size) {
        let from = view.fixedToScreen({
            x: minX,
            y: y,
        });

        let to = view.fixedToScreen({
            x: maxX,
            y: y,
        });

        ctx.moveTo(parseInt(from.x), parseInt(from.y));
        ctx.lineTo(parseInt(to.x), parseInt(to.y));
    }

	ctx.clearRect(0, 0, wid, hei);
	ctx.stroke();

	status.setOK();
}

function resizeScreen(){
	canvas.height = window.innerHeight;
	canvas.width = window.innerWidth;
	drawGrid();
}

window.addEventListener("view_changed", drawGrid);
window.addEventListener('resize', resizeScreen);
window.addEventListener('orientationchange', resizeScreen);

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
const linethickness = document.getElementById("{uniqueID}_thickness");
const gridstep = document.getElementById("{uniqueID}_step");

linethickness.value = grid_thickness;
colourpicker.value = grid_colour;
gridstep.value = grid_size;

colourpicker.addEventListener("input", (event) =>{
	grid_colour = colourpicker.value;
	drawGrid();
	saveSettings();
});

linethickness.addEventListener("input", (event) =>{
	if(linethickness.value > 20)
		grid_thickness = 20;
	else if(linethickness.value < 1)
		grid_thickness = 1;	
	else if(isNaN(linethickness.value))
		grid_thickness = 1;
	else
		grid_thickness = parseFloat(linethickness.value);	

	drawGrid();
	saveSettings();
});

gridstep.addEventListener("input", (event) =>{
	if(gridstep.value > 10000)
		grid_size = 10000;	
	else if(gridstep.value < 0.01)
		grid_size = 0.01;	
	else if(isNaN(gridstep.value))
		grid_size = 1.0;
	else
		grid_size = parseFloat(gridstep.value);	

	drawGrid();
	saveSettings();
});

resizeScreen();

console.log("Grid Widget Loaded {uniqueID}")