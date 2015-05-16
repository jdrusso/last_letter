!/usr/bin/env bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y libgl1-mesa-dev-lts-utopic
sudo apt-get install -y ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> /home/vagrant/.bashrc # Absolute path might be mandatory
source /home/vagrant/.bashrc
sudo apt-get install -y python-rosinstall

source /home/vagrant/.bashrc
mkdir -p /home/vagrant/catkin_ws/src
chown -R vagrant:vagrant catkin_ws
cd /home/vagrant/catkin_ws/src
whoami
cat $ROS_PACKAGE_PATH
catkin_init_workspace
cd ..
catkin_make
echo "source /home/vagrant/catkin_ws/devel/setup.bash" >> /home/vagrant/.bashrc
source /home/vagrant/.bashrc

# roscd
# cd ../src #Navigate in your ROS user source files directory
# git clone https://github.com/Georacer/last_letter.git #Clone the simulator files
# roscd
# cd .. #Navigate in your ROS workspace
# catkin_make #Compile the files
# rosdep update
# rosdep install --from-paths /path/to/catkin/ws/src/last_letter --ignore-src

