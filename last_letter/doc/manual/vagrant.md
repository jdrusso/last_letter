##Create a virtual machine using Vagrant

One might have the reasonable requirement to use a virtual machine to try out ROS and `last_letter`, before fully commiting into installing those pieces of software permanently to his computer.

_Vagrant_ is a software that can use _Virtual Box_ to automate the procedure of downloading an image of a specific virtual machine, booting it and perform the necessary steps to bring that machine up to the point where it is ready to be used for its intended goal.
In our case, Vagrant can perform all of the `last_letter` [installation steps](ll_installation.md) in a virtual machine automatically.

The Vagrant file provided with `last_letter` will automatically download an Ubuntu Trusty x64 image, make a virtual machine with it, configured with the desired characteristics, and after booting it up will run the `configure_script.sh` provision file which configures the installaiton.

In order to raise the Vagrant environment, change your working directory besides the `Vagrantfile` and execute
```bash
vagrant up
```


[back to table of contents](../../../README.md)