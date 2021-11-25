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

## Installing Webots

In order to use Webots with the latest features we need to use a nightly build of Webots. The version I am using works with the lidar demo, but may have other issues. Sadly there is a bug preventing lidar activation with the stable release of Webots (R2021b) that is fixed in the nightly releases. (I confirmed this through testing as well as talking with the developers). The nice thing is that Webots working is technically non-critical to the mission so we can afford to use bleeding edge builds with it.

In any case the actual install process could not be simpler. Simply go to [this link](https://github.com/cyberbotics/webots/releases) and choose a nightly build. To do this click the Assets tab and download the "webots-R2021b-rev1-x86-64.tar.bz2" file, extract it using `tar -xvf` and run the executable stored inside. 

You can add this to your system path if you would like to be able to run it from anyway. 

## Installing webots_ros

**webots_ros** is the package we need in order to correctly interface between ros and webots. I have found that in order to get this to work you need to build it from source. Thankfully the default instructions to do so work really well for this. The following instructions were taken from [here](http://wiki.ros.org/webots_ros). 

Create a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```
Now to build and install:
```
# You need to be in the source directory
cd /path/to/catkin_ws/src

git clone -b noetic https://github.com/cyberbotics/webots_ros.git
# if you are not using the latest version of Webots, you need to checkout the tag corresponding to the Webots version: https://github.com/cyberbotics/webots_ros/releases
 
cd /path/to/catkin_ws

# checking dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic
 
# building
catkin_make

# source this workspace (careful when also sourcing others)
source /path/to/catkin_ws/devel/setup.bash
```
Just like before you can prevent yourself from having to source this constantly by executing:
```
echo "source /path/to/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
## Running the Demo
Whew! After all that you should be ready to run the demo and control the robot with lidar!

The first thing to do is to build Alex's controller code located under `/ros/robot_kbd` (in the Git directory). The instructions to do so are inside his readme `README.md`. I have copied the contents here for convenience. If in the future these instructions do not work please refer to his original readme for possible updates. 

### robot_kbd

`robot_kbd` is the manual keyboard controller for the NMT Lunabotics
2022 team's robot.

#### Building

If your environment is set up correctly, the following sequence of
commands should cause the program to build:
```sh
# (from the robot_kbd main directory, in which this README file is)
mkdir build
cd build
cmake ..
make
```
The build executable should then be placed in
`devel/lib/robot_kbd/robot_kbd` from the `build` directory. If that
doesn't work, talk to Alex (Alex Bethel#8751,
Alexander.B.Bethel@student.nmt.edu).

