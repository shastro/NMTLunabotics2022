// Header file for 8Bitdoh enum and mappings from the joystick to the robot
// control axis'

enum class Pro2Button {
  B = 0,
  A = 1,
  Y = 2,
  X = 3,
  leftBumper = 4,
  rightBumper = 5,
  select = 6,
  start = 7,
  thumbLeft = 8,
  thumbRight = 9,
};

enum class Pro2Axis {
  leftThumbX = 0,
  leftThumbY = 1,
  leftTrigger = 2,
  rightThumbX = 3,
  rightThumbY = 4,
  rightTrigger = 5,
  dpadX = 6,
  dpadY = 7,
};
