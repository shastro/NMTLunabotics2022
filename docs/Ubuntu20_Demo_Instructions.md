# Webots/ROS Setup Installation and Demo Instructions

## Assumptions
- System is Ubuntu 20.04 (Focal Fossa)

## Installing ROS

For all our ROS distributions we are going to use **noetic**. Since it is the most recent major release we can take advantage of the latest improvements and it should work easier on the Raspberry Pi. 

To install ROS on Ubuntu execute the following commands (Taken from [here](http://wiki.ros.org/noetic/Installation/Ubuntu))

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```
sudo apt update
sudo apt install ros-noetic-desktop
```

Remember you can always install a specific ROS package using:
```
sudo apt install ros-noetic-PACKAGE
```
e.g.
```
sudo apt install ros-noetic-slam-gmapping
```
To setup your enviornment use:
```
source /opt/ros/noetic/setup.bash
```
You **must** source this in every terminal you use. To make this automatic upon startup of any terminal execute:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
**Note:** If you have previous ROS installations, remove ROS and the source commands from your `.bashrc`. 

To uninstall any old ros packages run:
```
sudo apt-get remove ros-*
```
And be sure to clear your `.bashrc` from sourcing any old setup scripts. 

