### Jan 31 Cross Network Comms Test

Edit /etc/hosts to include a line that maps the current ip addr of the pi to its hostname as given by "hostname" eg (lunabotics-pi)

Use this name in the following tutorial: http://wiki.ros.org/ROS/Tutorials/MultipleMachines

```
export ROS_MASTER_URI=http://lunabotics-pi:11311
```
