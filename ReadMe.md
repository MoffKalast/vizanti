 ![OutdooROS](public/assets/icon/logo_background.png)

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.png)](https://opensource.org/licenses/BSD-3-Clause)

OutdooROS is a web-based visualization and control tool that was developed for more convenient operation of outdoor robots running the Robot Operating System (ROS). The application attempts to replicate RViz's orthographic 2D view as closely as possible with a smartphone friendly interface. The second goal is to allow planning and executing movement and mission commands, i.e. goals and routes, with custom buttons and parameter reconfigure.

## Installation

As a field tool, OutdooROS is designed to operate just as well without internet access, and as such the intended way is to run it off the robot, with rosbridge autoconnecting to the host IP and loading cached data from the robot. 

 ```bash
pip install Flask
sudo apt install ros-noetic-rosbridge-suite

cd ~/catkin_ws/src
git clone https://github.com/MoffKalast/outdooros.git
cd ..
catkin_make

 ```
 
Flask and Jinja2 are used for templating, rosbridge is required for socket communication.

## Run
```bash
roslaunch outdooros server.launch
```
The web app can be accessed at `http://<host_ip>:5000`. Client settings are automatically saved in localStorage. The satelite imagery renderer also uses the indexedDB to store tiles for offline use (note that this is IP specific).

If you're using a mobile device connected to a robot's hotspot that doesn't have internet access, make sure to turn off mobile data. This will prevent Android from sending packets to the wrong gateway.

----

## Feature list

Aside from the required ones, custom widgets can be added to the navbar to customize functionality for a given robot and test setup.

**Note: Some icons open setup modals instantly, while others use a single press to trigger actions and use a long press to open the modal.**

### <img src="wiki_assets/settings.png" alt="" title="Grid" width="30" height="30"/> Global Settings 

Set the background color and the fixed TF frame. Also has a button to reset the camera view to zero and default zoom.


### <img src="wiki_assets/grid.png" alt="" title="Grid" width="30" height="30"/> Grid 

The adjustable metric grid. Currently renders only in the fixed frame.


### <img src="wiki_assets/tf.png" alt="" title="TF" width="30" height="30"/> TF 

Renders TF frames, same options as in RViz for the most part.

### <img src="wiki_assets/robotmodel.png" alt="" title="Robot Model" width="30" height="30"/> Robot Model 

Renders a 2D sprite to represent the robot model or any specific TF link. 

### <img src="wiki_assets/reconfigure.png" alt="" title="Dynamic Reconfigure" width="30" height="30"/> Dynamic Reconfigure

Adjust configurations of all nodes supporting dynamic reconfigure params. Currently rather slow to load and update, but will make sure parameters are current. It treats ints as floats due to type autodetection problems.

### <img src="wiki_assets/add.png" alt="" title="Add new visualizer/widget" width="30" height="30"/> Add new visualizer/widget

Self explanatory.

----

### <img src="wiki_assets/joystick.png" alt="" title="Teleop Joystick" width="30" height="30"/> Teleop Joystick

Joystick used for publishing Twist messages, can be positioned anywhere on the screen and switched into holonomic mode.

### <img src="wiki_assets/initialpose.png" alt="" title="2D Pose Estimate" width="30" height="30"/> 2D Pose Estimate

Send the /initialpose for navigation startup. Long press to open setup menu.

### <img src="wiki_assets/simplegoal.png" alt="" title="2D Nav Goal" width="30" height="30"/> 2D Nav Goal 

Send a /move_base_simple/goal. Long press to open setup menu.

### <img src="wiki_assets/waypoints.png" alt="" title="Waypoint Mission" width="30" height="30"/> Waypoint Mission 

Create missions with multiple waypoints, then send them as a Path message. Long press to open setup menu.

### <img src="wiki_assets/area.png" alt="" title="Area Mission" width="30" height="30"/> Area Mission

Drag to select an area and publish it to a 3DBoundingBox topic. Long press to open setup menu.

### <img src="wiki_assets/button.png" alt="" title="Button" width="30" height="30"/> Button

A button with customizable text that displays the last message sent on a Bool topic and sends the inverse to toggle it when pressed. Long press to open setup menu.

----

### <img src="wiki_assets/map.png" alt="" title="Map" width="30" height="30"/> Map

Display an OccupancyGrid. Also has some experimental map_server controls for saving and loading maps.

### <img src="wiki_assets/satelite.png" alt="" title="Satellite Tiles" width="30" height="30"/> Satellite Tiles

Display satelite imagery, by default from OpenStreetMap. Requires a Fix origin with the correct frame in its header.

----

### <img src="wiki_assets/battery.png" alt="" title="Battery" width="30" height="30"/> Battery

Display a BatteryState message.

### <img src="wiki_assets/markerarray.png" alt="" title="Marker Array" width="30" height="30"/> Marker Array

Visualize a MarkerArray. Currently supported types are ARROW, CUBE, SPHERE, CYLIDER, LINE_STRIP and TEXT_VIEW_FACING. Since each of these widgets adds another canvas layer, it makes more sense to aggregate regular Marker messages into a Marker Array to avoid some of that overhead.

### <img src="wiki_assets/path.png" alt="" title="Path" width="30" height="30"/> Path

Render a Path message for navigation debugging.

### <img src="wiki_assets/temp.png" alt="" title="Temperature" width="30" height="30"/> Temperature

Display a Temperature message. Only as a widget for now, not on the view itself.