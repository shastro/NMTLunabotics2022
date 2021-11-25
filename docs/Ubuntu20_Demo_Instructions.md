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

### Building Alex's Robot Controller
The first thing to do is to build Alex's controller code located under `/ros/robot_kbd` (in the Git directory). The instructions to do so are inside his readme `README.md`. I have copied the contents here for convenience. If in the future these instructions do not work please refer to his original readme for possible updates. 

#### robot_kbd

`robot_kbd` is the manual keyboard controller for the NMT Lunabotics
2022 team's robot.

##### Building

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

### Starting the Sim

The first step is to start **rosmaster** by executing 
```
roscore
```

To start the sim, all you need to do is open webots (run the executable wherever you extracted the tar file), and open the world file called `lidardemo.wbt` which is located under `/webots/worlds/` on the git. (Do this in a separte terminal to the one you ran roscore). 

In the **webots console** you should see something like this (If you do not see a console press `Ctrl+J` in webots):
```
INFO: ros: Starting controller: /home/shastro/webots/projects/default/controllers/ros/ros
[ INFO] [1637800993.392768988]: Robot's unique name is david_<your_uniqe_name>.
[ INFO] [1637800993.398387064]: The controller is now connected to the ROS master.
```
If you do not see this and instead see an error like: `[FATAL] [1637801224.231388704]: Failed to contact master at http://localhost:11311. Please start ROS master and restart this controller.` You forgot to run roscore. Restart the simulation by pressing `Ctrl+Shift+T`. 

Everytime you start webots it will assign the robot a unique name, (eg **david_42656_15IMH05H**). Though it will always start with david (since we called the robot that). This name is given to you in the webots console when you start the sim, and is the name of the ros node you will interface with. (Try running `rosnode list` to see the robot id). 

Now to start the camera and lidar execute (in a new terminal):
```
rosservice call /david_<your_unique_name>/camera/enable 32
rosservice call /david_<your_unique_name>/Hokuyo_UTM_30LX/enable 32
rosservice call /david_<your_unique_name>/Hokuyo_UTM_30LX/enable_point_cloud True
```
Btw if you want to see all the services published by the robot run `rosservice list`, try also `rostopic` and `rosnode`. The ros commands support **tab completion** so its much easier to just type `rosservice call /d<TAB>/ca<TAB>/Ho<TAB>/enable 32` to fill out service calls. Pressing tab a few times will also return lists of possible values so **make sure to use tab completion**. 

Now you should have lidar and cameras showing! If not please enable the lidar options under View/Optional Rendering/ in the Webots menu. 

#### Moving the robot. 

To move the robot execute the robot_kbd program that we built earlier, and provide it the name of the robot ros node. EG david_42656_15IMH05H. 

Have fun!

#### Extra stuff
Try running `rosrun rqt_image_view rqt_image_view` and selecting both the camera image and the lidar range image. Very cool to see the real image topics coming from Webots!


