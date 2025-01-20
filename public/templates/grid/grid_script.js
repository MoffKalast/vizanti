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
let grid_autoscale = 'Coarse';
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
	grid_colour_sub = loaded_data.colour_sub ?? "#294056"; //for legacy config compatibility
	grid_autoscale = loaded_data.autoscale ?? 'Off'; 
	grid_subdivisions = loaded_data.subdivisions ?? 1;
}else{
	saveSettings();
}

linethickness.value = grid_thickness;
colourpicker.value = grid_colour;
autoscale.value = grid_autoscale;
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
    const magnitude = Math.floor(Math.log10(value));
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

/* function drawFixedLine(start_x, start_y, end_x, end_y, color, line_width) {
	ctx.beginPath();
	ctx.strokeStyle = color;
	ctx.lineWidth = line_width;

    let from = view.fixedToScreen({x: start_x, y: start_y});
    let to = view.fixedToScreen({x: end_x, y: end_y});

    ctx.moveTo(parseInt(from.x), parseInt(from.y));
    ctx.lineTo(parseInt(to.x), parseInt(to.y));

	ctx.stroke();
} */

function drawScreenLine(start_x, start_y, end_x, end_y, color, line_width) {
	ctx.beginPath();
	ctx.strokeStyle = color;
	ctx.lineWidth = line_width;
	
    ctx.moveTo(parseInt(start_x), parseInt(start_y));
    ctx.lineTo(parseInt(end_x), parseInt(end_y));

	ctx.stroke();
}

function drawGridLines(minX, minY, maxX, maxY, grid_size, subdivisions) {
    const subdivision_size = grid_size/subdivisions;
    
    // First, calculate screen positions of all grid intersections we'll need
    // This avoids recalculating the same points multiple times, fixedToScreen is kinda expensive
    const xPositions = new Map();
    const yPositions = new Map();
    
    // Calculate main grid positions
    for (let x = minX; x <= maxX; x += grid_size) {
        const screenPos = view.fixedToScreen({x: x, y: 0}).x;
        xPositions.set(x, parseInt(screenPos));
    }
    
    for (let y = minY; y <= maxY; y += grid_size) {
        const screenPos = view.fixedToScreen({x: 0, y: y}).y;
        yPositions.set(y, parseInt(screenPos));
    }

	if(subdivisions > 1)
	{
		// Calculate subdivision positions
		for (let x = minX; x <= maxX; x += grid_size) {
			for (let sub_x = 1; sub_x < subdivisions; sub_x++) {
				const cur_sub_x = x + sub_x * subdivision_size;
				const screenPos = view.fixedToScreen({x: cur_sub_x, y: 0}).x;
				xPositions.set(cur_sub_x, parseInt(screenPos));
			}
		}
		
		for (let y = minY; y <= maxY; y += grid_size) {
			for (let sub_y = 1; sub_y < subdivisions; sub_y++) {
				const cur_sub_y = y + sub_y * subdivision_size;
				const screenPos = view.fixedToScreen({x: 0, y: cur_sub_y}).y;
				yPositions.set(cur_sub_y, parseInt(screenPos));
			}
		}
	}
    
    // Draw subdivision lines
    ctx.beginPath();
    ctx.globalAlpha = 0.65;
    ctx.strokeStyle = grid_colour_sub;
    ctx.lineWidth = 1;

    // Vertical subdivisions
    for (let x = minX; x <= maxX; x += grid_size) {
        for (let sub_x = 1; sub_x < subdivisions; sub_x++) {
            const cur_sub_x = x + sub_x * subdivision_size;
            const screenX = xPositions.get(cur_sub_x);
            ctx.moveTo(screenX, 0);
            ctx.lineTo(screenX, canvas.height);
        }
    }

    // Horizontal subdivisions
    for (let y = minY; y <= maxY; y += grid_size) {
        for (let sub_y = 1; sub_y < subdivisions; sub_y++) {
            const cur_sub_y = y + sub_y * subdivision_size;
            const screenY = yPositions.get(cur_sub_y);
            ctx.moveTo(0, screenY);
            ctx.lineTo(canvas.width, screenY);
        }
    }
    
    ctx.stroke();

    // Draw main grid lines
    ctx.beginPath();
    ctx.globalAlpha = 1.0;
    ctx.strokeStyle = grid_colour;
    ctx.lineWidth = grid_thickness;

    // Vertical main lines
    for (let x = minX; x <= maxX; x += grid_size) {
        const screenX = xPositions.get(x);
        ctx.moveTo(screenX, 0);
        ctx.lineTo(screenX, canvas.height);
    }

    // Horizontal main lines
    for (let y = minY; y <= maxY; y += grid_size) {
        const screenY = yPositions.get(y);
        ctx.moveTo(0, screenY);
        ctx.lineTo(canvas.width, screenY);
    }

    ctx.stroke();
}

function drawGridScale(grid_size, wid, hei) {

	const xoffset = canvas.width < canvas.height? 40 : 100; //compact on vertical/mobile
	const yoffset = 40;

	// Draw scale info in bottom right corner
	const scale_to = view.screenToFixed({ x: wid-xoffset, y: hei-yoffset });
	const xscale_start = view.fixedToScreen({x: scale_to.x-grid_size, y: 0}).x;
	drawScreenLine(xscale_start, parseInt(hei-yoffset), parseInt(wid-xoffset), parseInt(hei-yoffset), grid_colour, 2);
	drawScreenLine(xscale_start, parseInt(hei-yoffset-5), xscale_start, parseInt(hei-yoffset+5), grid_colour, 2);
	drawScreenLine(parseInt(wid-xoffset), parseInt(hei-yoffset-5), parseInt(wid-xoffset), parseInt(hei-yoffset+5), grid_colour, 2);

	const line_length = parseInt(wid-xoffset) - xscale_start;

	let scale_text = String(grid_size) + ' m';
	if(grid_size >= 1000)
		scale_text = String(grid_size/1000) + ' km';
	else if(grid_size < 1)
		scale_text = String(grid_size*100) + ' cm';

	ctx.font = "16px Monospace";
	ctx.textAlign = "center";
	ctx.fillStyle = grid_colour;
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

	if(grid_autoscale === 'Very Fine')
		grid_size = calculateScale(Math.min(width_meters, height_meters)/21);
	else if(grid_autoscale === 'Fine')
		grid_size = calculateScale(Math.min(width_meters, height_meters)/14);
	else if(grid_autoscale === 'Coarse')
		grid_size = calculateScale(Math.min(width_meters, height_meters)/7);
	else if(grid_autoscale === 'Rough')
		grid_size = calculateScale(Math.min(width_meters, height_meters)/3);
	
	const minX = topLeft.x - (topLeft.x % grid_size) - grid_size;
	const maxX = bottomRight.x + (grid_size - (bottomRight.x % grid_size));
	
	const minY = bottomRight.y - (bottomRight.y % grid_size) - grid_size;
	const maxY = topLeft.y + (grid_size - (topLeft.y % grid_size));
	
	ctx.clearRect(0, 0, wid, hei);

	let temp_subdivisions = grid_subdivisions;

	if (grid_autoscale == 'Off') {
		let linesX = (maxX - minX) / (grid_size / (temp_subdivisions + 1));
		let linesY = (maxY - minY) / (grid_size / (temp_subdivisions + 1));
		
		// While we have too many lines and can still reduce subdivisions
		while ((linesX > 300 || linesY > 300) && temp_subdivisions > 0) {
			temp_subdivisions--;
			linesX = (maxX - minX) / (grid_size / (temp_subdivisions + 1));
			linesY = (maxY - minY) / (grid_size / (temp_subdivisions + 1));
		}
	
		// If we still have too many lines even with no subdivisions
		if (linesX > 300 || linesY > 300) {
			ctx.clearRect(0, 0, wid, hei);
			status.setWarn("Too many lines to render, increase step size");
			return;
		}
	}

    drawGridLines(minX, minY, maxX, maxY, grid_size, temp_subdivisions);

	if(grid_autoscale != 'Off'){
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
	grid_autoscale = autoscale.value;
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
	grid_subdivisions = parseInt(subdivisions.value);
	drawGrid();
	saveSettings();
});

resizeScreen();

console.log("Grid Widget Loaded {uniqueID}")
