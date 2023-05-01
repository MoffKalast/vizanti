 ![OutdooROS](public/assets/logo.png)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

OutdooROS is a web-based visualization tool developed for more convenient operation and control of outdoor robots, particularly UGVs and USVs. The application attempts to replicate RViz's orthographic 2D view as closely as possible with a smartphone friendly interface. The second goal is to allow planning and executing movement and mission commands, i.e. goals and routes, with custom buttons and parameter reconfigure.

## Installation

 ```bash
pip install Flask
sudo apt install ros-noetic-rosbridge-suite
 ```
 
Flask and Jinja2 are used for templating, rosbridge and rosapi are required for socket communication.

## Run
```bash
roslaunch outdooros server.launch
```
The web app can be accessed at `http://<host_ip>:5000`. Client settings are automatically saved in localStorage.

If you're using a mobile device connected to a robot's hotspot that doesn't have internet access, make sure to turn off mobile data. This will prevent Android from sending packets to the wrong gateway.

## Feature list

<img src="public/assets/tf.svg" alt="" title="Optional title" width="30" height="30"/> TF

<img src="public/assets/grid.svg" alt="" title="Optional title" width="30" height="30"/> Grid

<img src="public/assets/battery_100.svg" alt="" title="Optional title" width="30" height="30"/> Battery

<img src="public/assets/temp_warm.svg" alt="" title="Optional title" width="30" height="30"/> Temperature

<img src="public/assets/joystick.svg" alt="" title="Optional title" width="30" height="30"/> Joystick

<img src="public/assets/robotmodel.svg" alt="" title="Optional title" width="30" height="30"/> Robot Model

<img src="public/assets/map.svg" alt="" title="Optional title" width="30" height="30"/> Map

<img src="public/assets/markerarray.svg" alt="" title="Optional title" width="30" height="30"/> Marker Array

<img src="public/assets/simplegoal.svg" alt="" title="Optional title" width="30" height="30"/> Simple goal

<img src="public/assets/initialpose.svg" alt="" title="Optional title" width="30" height="30"/> Initialpose

<img src="public/assets/satelite.svg" alt="" title="Optional title" width="30" height="30"/> Satelite Tiles

<img src="public/assets/reconfigure.svg" alt="" title="Optional title" width="30" height="30"/> Dynamic Reconfigure

<img src="public/assets/add.svg" alt="" title="Optional title" width="30" height="30"/> Add new visualizer/widget

