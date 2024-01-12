FROM osrf/ros:galactic-desktop

RUN apt-get update && apt-get install -y \
  git \
  ros-galactic-joint-state-publisher \
  ros-galactic-robot-state-publisher \
  ros-galactic-cartographer \
  ros-galactic-cartographer-ros \
  ros-galactic-gazebo-plugins \
  ros-galactic-teleop-twist-keyboard \
  ros-galactic-teleop-twist-joy \
  ros-galactic-xacro \
  ros-galactic-nav2* \
  ros-galactic-urdf \
  ros-galactic-v4l2-camera

# Make the prompt a little nicer
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc 

RUN mkdir -p /galactic_ws/src

RUN git clone --recursive https://github.com/rigbetellabs/tortoisebot.git -b ros2-galactic /galactic_ws/src/tortoisebot

COPY tortoisebot_waypoints/tortoisebot_waypoints /galactic_ws/src/tortoisebot_waypoints

RUN rm -rf /galactic_ws/src/tortoisebot/tortoisebot_control
# RUN rm -rf /galactic_ws/src/tortoisebot/ydlidar-ros2

ADD ros_entrypoint.sh /

WORKDIR /galactic_ws

RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/galactic/setup.bash && cd /galactic_ws && colcon build 2>&1 > /galactic_ws/build.log ; exit 0"

RUN echo "source /galactic_ws/install/setup.bash" >> ~/.bashrc

RUN rm -f /usr/bin/python && ln -s /usr/bin/python3 /usr/bin/python

ENTRYPOINT ["bash", "/ros_entrypoint.sh"]

CMD ["ros2", "launch tortoisebot_waypoints tortoisebot_waypoints.launch.py"]
