TL;DR for robot & simulation setups. This assumes you already have
ROS, Webots, etc. set up on your machine.

# Navigation simulation

- `cd` into the root git directory.
- Source the development shell script: `. ./ros/catkin_ws/devel/setup.bash`.
- Set up the navigation nodes, etc.: `roslaunch david_conf temp_sim.sh robot_path:=david_sim`.
- In another terminal, set up the main world simulation: `webots webots/worlds/lunabotics_ai.wbt`.
- In another terminal, enable the lidar mechanism: `rosservice call '/david_sim/Hokuyo_URG_04LX/enable' 64`.
- Enable the lidar point cloud: `rosservice call /david_sim/Hokuyo_URG_04LX/enable_point_cloud 64`.
- Launch rviz: `rviz`.
- Add a viewer for the map and whatever other topics should be visualized.
  - To enable the map, click `Add` in the lower left, then select `Map`, and set the `Topic` in the added node to the only option, `/map`.
  - To enable transform viewing, click `Add`, then select `TF`. This should work without any further configuration.
  - To speed up this process, when running rviz you can click `File`, then select `Open Config` and select the config under `./ros/catkin_ws/src/david_conf/rviz/sim_hector.rviz`
