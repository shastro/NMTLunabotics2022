# `david_motor`

`david_motor` is the ROS node responsible for directly controlling the
robot's physical motors (specifically the Teknic ones, not the Zoom
Industrial pitch motor ones), both in the simulation and on the
physical robot.

The node controls each motor differently and independently:
- Dispatches `geometry_msgs.msg.Twist` commands sent on `/cmd_vel` (by
  the navigation stack) into motor commands for the left and right
  locomotion motors. Only the X component of the linear component
  (forward-backward motion) and the Z component of the angular
  component (left-right rotation) are interpreted; other components
  are ignored.
- (TODO) Dispatches `david_motor.msg.AugerCmd` commands sent on
  `/cmd_auger` into motor commands for the auger and the depth motors.
- (TODO) Dispatches `david_motor.msg.DumperCmd` commands sent on
  `/cmd_auger` into motor commands for the dumper motors.
