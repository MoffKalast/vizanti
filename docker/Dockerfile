ARG ROS_VERSION=jazzy

FROM osrf/ros:${ROS_VERSION}-desktop

SHELL ["/bin/bash", "-c"]

# Username
ENV user=vizanti

# Update and build the vizanti package
RUN apt-get update

# Need openssh-client for git to work properly, can be removed when rosbridge has binaries properly updated.
RUN apt-get install openssh-client -y

# Only add our user if does not exist.
RUN id -u $user &>/dev/null || useradd -ms /bin/bash $user
RUN chown $user: /home
RUN usermod -o -u 1000 -g 1000 $user

USER $user
RUN mkdir -p /home/$user/colcon_ws/src
COPY . /home/$user/colcon_ws/src/vizanti/.

# Note: The binary release of rosbridge does not work correctly. The fixes are in the regular version of software so we need to clone and build it.
RUN cd /home/$user/colcon_ws/src/ && git clone -b ros2 https://github.com/RobotWebTools/rosbridge_suite.git
RUN rosdep update

USER root
RUN apt-get update
RUN cd /home/$user/colcon_ws && rosdep install -i --from-path src/vizanti -y
USER $user

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /home/$user/colcon_ws && colcon build

# Launch Vizanti
CMD source /home/$user/colcon_ws/install/setup.bash && ros2 launch vizanti_server vizanti_server.launch.py