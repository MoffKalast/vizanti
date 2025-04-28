FROM osrf/ros:noetic-desktop

SHELL ["/bin/bash", "-c"]

# Update and build the vizanti package
RUN apt-get update
RUN useradd -ms /bin/bash vizanti
RUN mkdir -p /home/vizanti/catkin_ws/src
COPY . /home/vizanti/catkin_ws/src/vizanti/.
WORKDIR "/home/vizanti/catkin_ws"
RUN source /opt/ros/noetic/setup.bash && rosdep update && rosdep install -i --from-path src/vizanti -y
RUN source /opt/ros/noetic/setup.bash && catkin_make

USER vizanti
# Launch Vizanti
CMD source /home/vizanti/catkin_ws/devel/setup.bash && roslaunch vizanti server.launch