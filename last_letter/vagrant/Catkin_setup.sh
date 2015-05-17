#!/usr/bin/env bash

# Create a new catkin workspace. Must be done as non-root
# http://answers.ros.org/question/113786/catkin-command-not-found/
source /opt/ros/indigo/setup.bash
mkdir -p /home/vagrant/catkin_ws/src
cd /home/vagrant/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
echo "source /home/vagrant/catkin_ws/devel/setup.bash" >> /home/vagrant/.bashrc
