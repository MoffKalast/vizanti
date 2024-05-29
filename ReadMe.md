# <img src="public/assets/icon/512.png" alt="Icon" title="Grid" width="50" height="50"/> Vizanti - Web Visualizer & Mission Planner for ROS

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.png)](https://opensource.org/licenses/BSD-3-Clause) [![Build Status](https://build.ros.org/buildStatus/icon?job=Ndev__vizanti__ubuntu_focal_amd64&build=4)](https://build.ros.org/job/Ndev__vizanti__ubuntu_focal_amd64/4/)

Vizanti is a web-based visualization and control tool developed for more convenient operation of outdoor robots running the Robot Operating System (ROS). The application attempts to replicate RViz's orthographic 2D view as closely as possible with a smartphone friendly interface. The second goal is to allow planning and executing movement and mission commands, i.e. goals and waypoints, with custom buttons and parameter reconfigure.

<img src="public/assets/icon/preview.jpg" alt=""/> 

## Installation

As a field tool, Vizanti is designed to operate just as well without internet access, and as such the intended way is to host it on a robot, with rosbridge autoconnecting to the host IP. 

 ```bash
cd ~/catkin_ws/src
git clone https://github.com/MoffKalast/vizanti.git
cd ..
rosdep install -i --from-path src/vizanti -y
catkin_make
```

Or if rosdep fails for some reason, these are the main two deps:
```
sudo apt install ros-noetic-rosbridge-suite python3-flask
```
 
Flask and Jinja2 are used for templating, rosbridge is required for socket communication.

## Run
```bash
roslaunch vizanti server.launch
```
The web app can be accessed at `http://<host_ip>:5000`. Client settings are automatically saved in localStorage. The satelite imagery renderer also uses the indexedDB to store tiles for offline use (note that this is IP specific). By default the rosbridge instance also occupies port 5001.

If you're using a mobile device connected to a robot's hotspot that doesn't have internet access, make sure to turn off mobile data. This will prevent Android from sending packets to the wrong gateway.

####  Check [the wiki](https://github.com/MoffKalast/vizanti/wiki) for usage and configuration instructions, as well as feature and compatibility info.

## Contributing

Please see [Contributing.md](Contributing.md) for more information.