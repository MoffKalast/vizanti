let canvas = undefined

self.addEventListener('message', function(event) {

    if(event.data.canvas){
        canvas = event.data.canvas;
        return;
    }

    const msg = event.data.map_msg;
    const is_costmap = event.data.is_costmap;

    const width = msg.info.width;
    const height = msg.info.height;

    canvas.width = width;
    canvas.height = height;

    const mapctx = canvas.getContext('2d');
    
    const data = msg.data;
    
    let map_img = mapctx.createImageData(width, height);
    
    if(is_costmap)
    {
        // Iterate through the data array and set the canvas pixel colors
        for (let i = 0; i < data.length; i++) {
            let occupancyValue = data[i];
            let color = 255; // White for unknown

            if(occupancyValue < 0)
                occupancyValue = 0;

            color = (occupancyValue * 255) / 100;

            if(occupancyValue == 100){
                map_img.data[i * 4] = 255; // R
                map_img.data[i * 4 + 1] = 0; // G
                map_img.data[i * 4 + 2] = 128; // B
                map_img.data[i * 4 + 3] = 255; // A
            }
            else if(occupancyValue > 80){
                map_img.data[i * 4] = 0; // R
                map_img.data[i * 4 + 1] = 255; // G
                map_img.data[i * 4 + 2] = 255; // B
                map_img.data[i * 4 + 3] = 255; // A
            }
            else{
                map_img.data[i * 4] = color; // R
                map_img.data[i * 4 + 1] = 0; // G
                map_img.data[i * 4 + 2] = 255-color; // B
                map_img.data[i * 4 + 3] = parseInt(occupancyValue*2.55); // A
            }
        }
    }
    else
    {
        // Iterate through the data array and set the canvas pixel colors
        for (let i = 0; i < data.length; i++) {
            let occupancyValue = data[i];
            let color = 255; // White for unknown

            if(occupancyValue < 0)
                occupancyValue = 50;

            if (occupancyValue >= 0 && occupancyValue <= 100) {
                color = 255 - (occupancyValue * 255) / 100;
            }

            map_img.data[i * 4] = color; // R
            map_img.data[i * 4 + 1] = color; // G
            map_img.data[i * 4 + 2] = color; // B
            map_img.data[i * 4 + 3] = 255; // A
        }
    }

    mapctx.putImageData(map_img, 0, 0);

    self.postMessage({});

}, false);