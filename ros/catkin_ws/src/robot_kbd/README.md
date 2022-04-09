# robot_kbd

This package is deprecated as it uses raw motor functions rather than
interfacing with `base_controller`. Use a terminal-based controller
like `turtlebot3_teleop_key` for similar functionality, if needed.

`robot_kbd` is the manual keyboard controller for the NMT Lunabotics
2022 team's robot.

## Building

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
