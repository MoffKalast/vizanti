let canvas = undefined

//fast lookup tables for pixel mapping
const COSTMAP_R = new Uint8Array(256);
const COSTMAP_G = new Uint8Array(256);
const COSTMAP_B = new Uint8Array(256);
const COSTMAP_A = new Uint8Array(256);

const MAP_R = new Uint8Array(256);
const MAP_G = new Uint8Array(256);
const MAP_B = new Uint8Array(256);
const MAP_A = new Uint8Array(256);

const ALPHA_LUT = new Uint8Array(256);

//Alpha power curve
for (let i = 0; i < 256; i++) {
    ALPHA_LUT[i] = Math.round(255 * Math.pow(i / 255, 0.5));
}

//Costmap
for (let i = 0; i < 256; i++) {
    let r, g, b, a;
    
    if (i === 0) {
        [r, g, b, a] = [0, 0, 0, 0]; // Black for value 0
    } else if (i >= 1 && i <= 98) {
        let v = (255 * i) / 100;
        [r, g, b, a] = [v, 0, 255 - v, 255]; // Gradient from blue to green
    } else if (i === 99) {
        [r, g, b, a] = [0, 255, 255, 255]; // Cyan for obstacle values
    } else if (i === 100) {
        [r, g, b, a] = [255, 0, 255, 255]; // Purple for lethal obstacle values
    } else if (i === 255) { // -1 value after +256
        [r, g, b, a] = [0x70, 0x89, 0x86, 15]; // Legal negative value -1
    } else {
        [r, g, b, a] = [0, 255, 0, 0]; // Green for illegal positive values
    }

    COSTMAP_R[i] = r;
    COSTMAP_G[i] = g;
    COSTMAP_B[i] = b;
    COSTMAP_A[i] = a;
}

//Map
for (let i = 0; i < 256; i++) {
    let [r, g, b, a] = [255, 255, 255, 255]; // White for clear

    if (i >= 0 && i <= 100) {
        let v = 255 - (255 * i) / 100;
        [r, g, b, a] = [v, v, v, 255];
    } else if (i < 0) {
        [r, g, b, a] = [0x70, 0x89, 0x86, 30]; // Legal negative value -1
    } else if (i > 100) {
        [r, g, b, a] = [0, 255, 0, 30]; // Illegal positive value
    }

    MAP_R[i] = r;
    MAP_G[i] = g;
    MAP_B[i] = b;
    MAP_A[i] = a;
}


self.addEventListener('message', function(event) {

    if(event.data.canvas){
        canvas = event.data.canvas;
        return;
    }

    const msg = event.data.map_msg;
    const colour_scheme = event.data.colour_scheme;

    const width = msg.info.width;
    const height = msg.info.height;

    canvas.width = width;
    canvas.height = height;

    const mapctx = canvas.getContext('2d', { colorSpace: 'srgb' });
    
    const data = msg.data;
    
    let map_img = mapctx.createImageData(width, height);

    if(colour_scheme == "costmap")
    {
        // Iterate through the data array and set the canvas pixel colors
        for (let i = 0; i < data.length; i++) {
            let occupancyValue = data[i];
            if(occupancyValue < 0) occupancyValue += 256;
            map_img.data[i * 4] = COSTMAP_R[occupancyValue];
            map_img.data[i * 4 + 1] = COSTMAP_G[occupancyValue];
            map_img.data[i * 4 + 2] = COSTMAP_B[occupancyValue];
            map_img.data[i * 4 + 3] = COSTMAP_A[occupancyValue];
        }
    }
    else if(colour_scheme == "map")
    {
        for (let i = 0; i < data.length; i++) {
            let occupancyValue = data[i];
            if(occupancyValue < 0) occupancyValue += 256;
            map_img.data[i * 4] = MAP_R[occupancyValue];
            map_img.data[i * 4 + 1] = MAP_G[occupancyValue];
            map_img.data[i * 4 + 2] = MAP_B[occupancyValue];
            map_img.data[i * 4 + 3] = MAP_A[occupancyValue];
        }
    }
    else if(colour_scheme == "raw_transparent")
    {
        for (let i = 0; i < data.length; i++) {
            let val = data[i];
            if(val < 0) val += 256;
            
            map_img.data[i * 4] = val;     // R
            map_img.data[i * 4 + 1] = val; // G
            map_img.data[i * 4 + 2] = val; // B
            map_img.data[i * 4 + 3] = ALPHA_LUT[255 - val]; // A
        }
    }
    else if(colour_scheme == "raw_transparent_black")
    {
        for (let i = 0; i < data.length; i++) {
            let val = data[i];
            if(val < 0) val += 256;
            
            map_img.data[i * 4] = val;     // R
            map_img.data[i * 4 + 1] = val; // G
            map_img.data[i * 4 + 2] = val; // B
            map_img.data[i * 4 + 3] = ALPHA_LUT[val]; // A
        }
    }
    else //raw
    {
        for (let i = 0; i < data.length; i++) {
            let val = data[i];

            if(val < 0)
                val += 256;

            map_img.data[i * 4] = val; // R
            map_img.data[i * 4 + 1] = val; // G
            map_img.data[i * 4 + 2] = val; // B
            map_img.data[i * 4 + 3] = 255; // A
        }
    }

    self.postMessage({image: map_img});

}, false);