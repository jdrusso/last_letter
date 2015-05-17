#!/usr/bin/env bash

echo "export LC_ALL="C"" >> /home/vagrant/.bashrc # Used to enable autocompletion via ssh

# Install ROS Indigo in the system as root
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y libgl1-mesa-dev-lts-utopic
sudo apt-get install -y ros-indigo-desktop-full
sudo rosdep init
su -c "rosdep update" vagrant
echo "source /opt/ros/indigo/setup.bash" >> /home/vagrant/.bashrc # Absolute path might be mandatory
sudo apt-get install -y python-rosinstall

su -c "source /vagrant/Catkin_setup.sh" vagrant

# su -c "source /vagrant/git_cloning.sh" vagrant