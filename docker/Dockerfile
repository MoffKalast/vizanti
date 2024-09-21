FROM osrf/ros:noetic-desktop

SHELL ["/bin/bash", "-c"]

# Update and build the vizanti package
RUN apt-get update
RUN useradd -ms /bin/bash vizanti
RUN mkdir -p /home/vizanti/catkin_ws/src
COPY . /home/vizanti/catkin_ws/src/vizanti/.
RUN apt install ros-noetic-rosbridge-suite ros-noetic-move-base-msgs python3-flask -y
RUN source /opt/ros/noetic/setup.bash && cd /home/vizanti/catkin_ws && catkin_make

USER vizanti
# Launch Vizanti
CMD source /home/vizanti/catkin_ws/devel/setup.bash && roslaunch vizanti server.launch