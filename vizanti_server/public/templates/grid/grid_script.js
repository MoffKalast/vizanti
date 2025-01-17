let viewModule = await import(`${base_url}/js/modules/view.js`);
let persistentModule = await import(`${base_url}/js/modules/persistent.js`);
let StatusModule = await import(`${base_url}/js/modules/status.js`);
let utilModule = await import(`${base_url}/js/modules/util.js`);

let view = viewModule.view;
let settings = persistentModule.settings;
let Status = StatusModule.Status;

let status = new Status(
	document.getElementById("{uniqueID}_icon"),
	document.getElementById("{uniqueID}_status")
);

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('object')[0];
const canvas = document.getElementById('{uniqueID}_canvas');
const ctx = canvas.getContext('2d', { colorSpace: 'srgb' });

let grid_size = 1.0;
let grid_thickness = 1;
let grid_colour = "#3e556a";
let grid_colour_sub = "#294056";
let grid_autoscale = true;
let grid_subdivisions = 2;

const colourpicker = document.getElementById("{uniqueID}_colorpicker");
const colourpicker_sub = document.getElementById("{uniqueID}_colorpicker_sub");
const autoscale = document.getElementById("{uniqueID}_autoscale");
const linethickness = document.getElementById("{uniqueID}_thickness");
const subdivisions = document.getElementById("{uniqueID}_subdivisions");
const gridstep = document.getElementById("{uniqueID}_step");

if(settings.hasOwnProperty("{uniqueID}")){
	const loaded_data  = settings["{uniqueID}"];
	grid_size = loaded_data.size;
	grid_thickness = loaded_data.thickness;
	grid_colour = loaded_data.colour;
	grid_colour_sub = loaded_data.colour_sub;
	grid_autoscale = loaded_data.autoscale;
	grid_subdivisions = loaded_data.subdivisions;
}else{
	saveSettings();
}

linethickness.value = grid_thickness;
colourpicker.value = grid_colour;
autoscale.checked = grid_autoscale;
colourpicker_sub.value = grid_colour_sub;
gridstep.value = grid_size;
subdivisions.value = grid_subdivisions;

//update the icon colour when it's loaded or when the image source changes
icon.onload = () => {
	utilModule.setIconColor(icon, colourpicker.value);
};
if (icon.contentDocument) {
	utilModule.setIconColor(icon, colourpicker.value);
}


function saveSettings(){
	settings["{uniqueID}"] = {
		size: grid_size,
		thickness: grid_thickness,
		colour: grid_colour,
		colour_sub: grid_colour_sub,
		autoscale: grid_autoscale,
		subdivisions: grid_subdivisions
	}
	settings.save();
}

function calculateScale(value) {
    let magnitude = Math.floor(Math.log10(value));
    value /= Math.pow(10, magnitude);

    if (value < 1.5) {
        value = 1.0;
    } else if (value < 3.5) {
        value = 2.0;
    } else if (value < 7.5) {
        value = 5.0;
    } else {
        value = 10.0;
    }

    value *= Math.pow(10, magnitude);
    return value;
}

function drawFixedLine(start_x, start_y, end_x, end_y, color, line_width) {
	ctx.beginPath();
	ctx.strokeStyle = color;
	ctx.lineWidth = line_width;

    let from = view.fixedToScreen({x: start_x, y: start_y});
    let to = view.fixedToScreen({x: end_x, y: end_y});

    ctx.moveTo(parseInt(from.x), parseInt(from.y));
    ctx.lineTo(parseInt(to.x), parseInt(to.y));

	ctx.stroke();
}

function drawScreenLine(start_x, start_y, end_x, end_y, color, line_width) {
	ctx.beginPath();
	ctx.strokeStyle = color;
	ctx.lineWidth = line_width;
	
    ctx.moveTo(parseInt(start_x), parseInt(start_y));
    ctx.lineTo(parseInt(end_x), parseInt(end_y));

	ctx.stroke();
}


function drawGridLines(minX, minY, maxX, maxY, grid_size, subdivisions) {
	let subdivision_size = grid_size/subdivisions;

	// Draw vertical subdivision lines
    for (let x = minX; x <= maxX; x += grid_size) {
		for (let sub_x = 1; sub_x < subdivisions; sub_x += 1) {
			let cur_sub_x = sub_x*subdivision_size + x;
			drawFixedLine(cur_sub_x, minY, cur_sub_x, maxY, grid_colour_sub, 1);
		}
    }

    // Draw horizontal lines
    for (let y = minY; y <= maxY; y += grid_size) {
		drawFixedLine(minX, y, maxX, y, grid_colour, grid_thickness);

		// draw subdivisions
		for (let sub_y = 1; sub_y < subdivisions; sub_y += 1) {
			let cur_sub_y = sub_y*subdivision_size + y;
			drawFixedLine(minX, cur_sub_y, maxX, cur_sub_y, grid_colour_sub, 1);
		}
    }

	// Draw main vertical lines so they are above subdivisions
	for (let x = minX; x <= maxX; x += grid_size) {
		drawFixedLine(x, minY, x, maxY, grid_colour, grid_thickness);
	}
}

function drawGridScale(grid_size, wid, hei) {
	// Draw scale info in bottom right corner
	let scale_to = view.screenToFixed({ x: wid-100, y: hei-40 });
	let xscale_start = view.fixedToScreen({x: scale_to.x-grid_size, y: 0}).x;
	drawScreenLine(xscale_start, parseInt(hei-40), parseInt(wid-100), parseInt(hei-40), "#7990A6", 2);
	drawScreenLine(xscale_start, parseInt(hei-35), xscale_start, parseInt(hei-45), "#7990A6", 2);
	drawScreenLine(parseInt(wid-100), parseInt(hei-35), parseInt(wid-100), parseInt(hei-45), "#7990A6", 2);

	let line_length = parseInt(wid-100) - xscale_start;

	let scale_text = String(grid_size) + ' m';
	if(grid_size > 1000)
		scale_text = String(grid_size/1000) + ' km';

	ctx.font = "16px Sans-serif";
	ctx.textAlign = "center";
	ctx.fillStyle = "#698096";
	ctx.fillText(scale_text, parseInt(xscale_start + line_length/2), parseInt(hei-23));
}

async function drawGrid() {
    const wid = canvas.width;
    const hei = canvas.height;

    ctx.strokeStyle = grid_colour;
	ctx.lineWidth = grid_thickness;

	const topLeft = view.screenToFixed({ x: 0, y: 0 });
	const bottomRight = view.screenToFixed({ x: wid, y: hei });

	const width_meters = Math.abs(bottomRight.x - topLeft.x);
	const height_meters = Math.abs(bottomRight.y - topLeft.y);

	const max_lines = 15;

	if(grid_autoscale)
		grid_size = calculateScale(Math.min(width_meters, height_meters)/max_lines);
		
	const minX = topLeft.x - (topLeft.x % grid_size) - grid_size;
	const maxX = bottomRight.x + (grid_size - (bottomRight.x % grid_size));
	
	const minY = bottomRight.y - (bottomRight.y % grid_size) - grid_size;
	const maxY = topLeft.y + (grid_size - (topLeft.y % grid_size));
	
	ctx.clearRect(0, 0, wid, hei);

	if(!grid_autoscale) {
		const linesX = (maxX-minX)/(grid_size/grid_subdivisions);
		const linesY = (maxY-minY)/(grid_size/grid_subdivisions);
	
		if(linesX > 200 || linesY > 200){
			ctx.clearRect(0, 0, wid, hei);
			status.setWarn("Too many lines to render, increase step size.");
			return;
		}
	}

    drawGridLines(minX, minY, maxX, maxY, grid_size, grid_subdivisions);

	if(grid_autoscale) {
		drawGridScale(grid_size, wid, hei);
	}
	
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
	if(gridstep.value > 1000000)
		grid_size = 1000000;	
	else if(gridstep.value < 0.01)
		grid_size = 0.01;	
	else if(isNaN(gridstep.value))
		grid_size = 1.0;
	else
		grid_size = parseFloat(gridstep.value);	

	drawGrid();
	saveSettings();
});

colourpicker.addEventListener("input", (event) =>{
	grid_colour = colourpicker.value;
	utilModule.setIconColor(icon, grid_colour);
	drawGrid();
	saveSettings();
});

colourpicker_sub.addEventListener("input", (event) =>{
	grid_colour_sub = colourpicker_sub.value;
	drawGrid();
	saveSettings();
});

autoscale.addEventListener("input", (event) =>{
	grid_autoscale = autoscale.checked;
	if(gridstep.value > 1000000)
		grid_size = 1000000;	
	else if(gridstep.value < 0.01)
		grid_size = 0.01;	
	else if(isNaN(gridstep.value))
		grid_size = 1.0;
	else
		grid_size = parseFloat(gridstep.value);	
	drawGrid();
	saveSettings();
});

subdivisions.addEventListener("input", (event) =>{
	grid_subdivisions = subdivisions.value;
	drawGrid();
	saveSettings();
});

resizeScreen();

console.log("Grid Widget Loaded {uniqueID}")
