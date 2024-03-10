let canvas = undefined

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

    const mapctx = canvas.getContext('2d');
    
    const data = msg.data;
    
    let map_img = mapctx.createImageData(width, height);
    
    if(colour_scheme == "costmap")
    {
        // Iterate through the data array and set the canvas pixel colors
        for (let i = 0; i < data.length; i++) {
            let occupancyValue = data[i];
            let color = [0, 255, 0, 0]; // Green for illegal positive values
        
            if (occupancyValue === 0) {
                color = [0, 0, 0, 0]; // Black for value 0
            } else if (occupancyValue >= 1 && occupancyValue <= 98) {
                let v = (255 * occupancyValue) / 100;
                color = [v, 0, 255 - v, 255]; // Gradient from blue to green
            } else if (occupancyValue === 99) {
                color = [0, 255, 255, 255]; // Cyan for obstacle values
            } else if (occupancyValue === 100) {
                color = [255, 0, 255, 255]; // Purple for lethal obstacle values
            } else if (occupancyValue < 0) {
                color = [0x70, 0x89, 0x86, 15]; // Legal negative value -1
            }
        
            map_img.data[i * 4] = color[0]; // R
            map_img.data[i * 4 + 1] = color[1]; // G
            map_img.data[i * 4 + 2] = color[2]; // B
            map_img.data[i * 4 + 3] = color[3]; // A
        }
    }
    else if(colour_scheme == "map")
    {
        for (let i = 0; i < data.length; i++) {
            let occupancyValue = data[i];
            let color = [255, 255, 255, 255]; // White for clear
            if (occupancyValue >= 0 && occupancyValue <= 100) {
                let v = 255 - (255 * occupancyValue) / 100;
                color = [v, v, v, 255];
            } else if (occupancyValue < 0) {
                color = [0x70, 0x89, 0x86, 30]; // Legal negative value -1
            } else if (occupancyValue > 100) {
                color = [0, 255, 0, 30]; // Illegal positive value
            }
        
            map_img.data[i * 4] = color[0]; // R
            map_img.data[i * 4 + 1] = color[1]; // G
            map_img.data[i * 4 + 2] = color[2]; // B
            map_img.data[i * 4 + 3] = color[3]; // A
        }
    }
    else
    {
        for (let i = 0; i < data.length; i++) {
            let val = data[i];

            if(val < 0)
                val = 255;

            map_img.data[i * 4] = val; // R
            map_img.data[i * 4 + 1] = val; // G
            map_img.data[i * 4 + 2] = val; // B
            map_img.data[i * 4 + 3] = 255; // A
        }
    }

    //mapctx.putImageData(map_img, 0, 0);

    self.postMessage({image: map_img});

}, false);