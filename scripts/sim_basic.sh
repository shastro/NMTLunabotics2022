#!/bin/sh

# Basic shell script for starting up the simulation. Important: to
# exit the simulation and clean up resources properly, close RViz, not
# Webots or the terminal.

# Exit on error, rather than continuing.
set -e

cd $(dirname "$0")

# wait_for_ros : Waits for ROS to launch.
wait_for_ros () {
    while ! rosservice list > /dev/null 2>&1; do
        echo 'waiting for ROS'
        sleep 1
    done
}

# wait_and_call <ros_service> <params ...> : Waits for the ROS service
# to exist, then calls it, avoiding startup race conditions.
wait_and_call () {
    # ROS has to be initialized; wait for `rosservice` to successfully
    # connect.
    wait_for_ros

    # The service has to exist.
    while ! rosservice list | grep -q "^$1\$"; do
        echo 'waiting for service'
        sleep 1
    done

    # Now it's safe to call the service.
    rosservice call "$@"
}

# Start up ROS.
. ../ros/catkin_ws/devel/setup.sh
roslaunch david_conf temp_sim.launch robot_path:=david_sim &

# Connect the Webots world.
wait_for_ros
webots ../webots/worlds/lunabotics_TEST_FLAT.wbt &

# Enable the lidar.
wait_and_call /david_sim/laser/enable 64
wait_and_call /david_sim/laser/enable_point_cloud 64

# Start up RVIZ; use `true` because we still need to clean up, even if
# it fails.
rviz -d ../ros/catkin_ws/src/david_conf/rviz/sim_hector.rviz || true

# Kill the entire simulation.
killall webots || true
killall -SIGINT roslaunch || true
