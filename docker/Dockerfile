ARG ROS_VERSION=jazzy

FROM osrf/ros:${ROS_VERSION}-desktop

SHELL ["/bin/bash", "-c"]

# Username
ENV user=vizanti

RUN apt-get update
RUN apt-get install openssh-client -y
RUN apt-get install ros-$ROS_DISTRO-rmw-fastrtps-dynamic-cpp -y
RUN apt-get install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y

# Only add our user if does not exist.
RUN id -u $user &>/dev/null || useradd -ms /bin/bash $user
RUN chown $user: /home
RUN usermod -o -u 1000 -g 1000 $user

USER $user
RUN mkdir -p /home/$user/colcon_ws/src
COPY . /home/$user/colcon_ws/src/vizanti/.
    
# Install the RWS implementation alternative to ROSbridge.
RUN cd /home/$user/colcon_ws/src/ && git clone -b $ROS_DISTRO https://github.com/v-kiniv/rws.git ;

RUN rosdep update

USER root
RUN apt-get update
RUN cd /home/$user/colcon_ws && rosdep install -i --from-path src -y ;
USER $user

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /home/$user/colcon_ws && colcon build

# Launch the Vizanti with RWS or ROSBridge
CMD source /home/$user/colcon_ws/install/setup.bash \
    && \
    if [ "${USE_RWS}" == "true" ] ; then \
      ros2 launch vizanti_server vizanti_rws.launch.py ; \
    else \
      ros2 launch vizanti_server vizanti_server.launch.py ; \
    fi
