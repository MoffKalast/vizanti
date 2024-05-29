# <img src="vizanti_server/public/assets/icon/512.png" alt="Icon" title="Grid" width="50" height="50"/> Vizanti - Web Visualizer & Mission Planner for ROS

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.png)](https://opensource.org/licenses/BSD-3-Clause) [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__vizanti__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__vizanti__ubuntu_jammy_amd64/)

Vizanti is a web-based visualization and control tool developed for more convenient operation of outdoor robots running the Robot Operating System (ROS). The application attempts to replicate RViz's orthographic 2D view as closely as possible with a smartphone friendly interface. The second goal is to allow planning and executing movement and mission commands, i.e. goals and waypoints, with custom buttons and parameter reconfigure.

<img src="vizanti_server/public/assets/icon/preview.jpg" alt=""/> 

## Installation

As a field tool, Vizanti is designed to operate just as well without internet access, and as such the intended way is to host it on a robot, with rosbridge autoconnecting to the host IP.

```bash
cd ~/colcon_ws/src
git clone -b ros2 https://github.com/MoffKalast/vizanti.git

cd ..
rosdep install -i --from-path src/vizanti -y
colcon build
```


## Run
```bash
ros2 launch vizanti_server vizanti_server.launch.py
```
The web app can be accessed at `http://<host_ip>:5000`. Client settings are automatically saved in localStorage. The satelite imagery renderer also uses the indexedDB to store tiles for offline use (note that this is IP specific). By default the rosbridge instance also occupies port 5001.

If you're using a mobile device connected to a robot's hotspot that doesn't have internet access and can't load the page, turn off mobile data. This will prevent the browser from sending packets to the wrong gateway.

####  Check [the wiki](https://github.com/MoffKalast/vizanti/wiki) for usage and configuration instructions, as well as feature and compatibility info.

## Optional - Experimental RWS Backend

With rosbridge being a Tornado python based package and rclpy being overly CPU heavy, this cpp drop-in replacement server should result in a ~5x lower overhead and faster response times. It works with CycloneDDS out of the box, and for FastDDS it requires the `rmw_fastrtps_dynamic_cpp` version which includes interface introspection.

```bash
cd ~/colcon_ws/src
git clone -b humble https://github.com/v-kiniv/rws.git

cd ..
rosdep install -i --from-path src/vizanti --rosdistro humble -y
colcon build
```

If using FastDDS:

```bash
sudo apt install ros-humble-rmw-fastrtps-dynamic-cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp
```

Then run the RWS launch instead:

```bash
ros2 launch vizanti_server vizanti_rws.launch.py
```

----

## Contributing

Please see [Contributing.md](Contributing.md) for more information.
