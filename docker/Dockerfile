ARG ROS_VERSION=jazzy

FROM osrf/ros:${ROS_VERSION}-desktop

ARG USE_RWS=

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

# Note: There are breaking changes in the rosbridge suite for ROS versions. We need to switch based on version.
RUN if [ "${ROS_DISTRO}" == "humble" ] ; then \
      cd /home/$user/colcon_ws/src/ \
      && git clone -b humble https://github.com/RobotWebTools/rosbridge_suite.git ; \
    else \
      cd /home/$user/colcon_ws/src/ \
      && git clone -b ros2 https://github.com/RobotWebTools/rosbridge_suite.git ; \
    fi

RUN echo ${RMW_IMPLEMENTATION}

RUN rosdep update

RUN if [ "${USE_RWS}" == "true" ] ; then \
      cd /home/$user/colcon_ws/src/ \
      && git clone -b $ROS_DISTRO https://github.com/v-kiniv/rws.git ; \
    fi

USER root
RUN apt-get update
RUN cd /home/$user/colcon_ws && rosdep install -i --from-path src/vizanti -y && rosdep install -i --from-path src/rws -y ;
USER $user

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /home/$user/colcon_ws && colcon build

ENV USE_RWS=$USE_RWS
#ENV RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp

# Launch the Vizanti with RWS or ROSBridge
CMD source /home/$user/colcon_ws/install/setup.bash \
    && \
    if [ "${USE_RWS}" == "true" ] ; then \
      ros2 launch vizanti_server vizanti_rws.launch.py ; \
    else \
      ros2 launch vizanti_server vizanti_server.launch.py ; \
    fi