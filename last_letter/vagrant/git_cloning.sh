#!/usr/bin/env bash

# Clone and build last_letter. Must be done as non-root
# http://answers.ros.org/question/113786/catkin-command-not-found/
source /home/vagrant/catkin_ws/devel/setup.bash
roscd
cd ../src #Navigate in your ROS user source files directory
git clone https://github.com/Georacer/last_letter.git #Clone the simulator files
roscd
cd .. #Navigate in your ROS workspace
catkin_make #Compile the files
rosdep update
rosdep install --from-paths /path/to/catkin/ws/src/last_letter --ignore-src
