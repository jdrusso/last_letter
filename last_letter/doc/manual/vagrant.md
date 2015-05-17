##Create a virtual machine using Vagrant

One might have the reasonable requirement to use a virtual machine to try out ROS and `last_letter`, before fully commiting into installing those pieces of software permanently to his computer.

_Vagrant_ is a software that can use _Virtual Box_ to automate the procedure of downloading an image of a specific virtual machine, booting it and performing the necessary steps to bring that machine up to the point where it is ready to be used for its intended goal.
In our case, Vagrant can perform all of the `last_letter` [installation steps](ll_installation.md) in a virtual machine automatically.

The Vagrant file provided with `last_letter` will automatically download an Ubuntu Trusty x64 image, make a virtual machine with it, configured with the desired characteristics, and after booting it up will run the `configure_script.sh` provision file which configures the installation.

In order to raise the Vagrant environment, first install _Virtual Box_ and _Vagrant_ in your machine, according to these instructions:
http://docs.vagrantup.com/v2/getting-started/index.html

Next copy all the necessary configuration files from here:
https://github.com/Georacer/last_letter/tree/devel/last_letter/vagrant
to any directory.

Change your working directory besides the `Vagrantfile` you just downloaded and execute
```bash
vagrant up
```

Vagrant will download and install the full ROS Indigo distribution in the virtual machine, as well as download and compile the `last_letter` code. This is **very** network traffic intensive and is expected to last 10-20 minutes.

###Limitations
The Vagrant cloud doesn't provide an official Ubuntu Trusty distribution with a graphical environemnt. As a result, `rviz` won't be able to run. Extra download and installation steps are required to fix this, which are very lengthy in time and large in data. Overall, the user experiance is not smooth.

Moreover, during the conducted tests with a graphical environment enabled, the resulting frame-rate of the simulation was very poor.

For these reasons, the only case where the use of a Vagrant/VirtualBox virtual machine solution is recommended is to use `last_letter` as the physics engine for _ardupilot_, where a graphical environment is not needed.

[back to table of contents](../../../README.md)