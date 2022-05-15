# Auger & auger assembly control messages.

# Velocity of the depth motors, on a scale from -1 (moving down at
# full throttle) to 1 (moving up at full throttle).
float64 depth_vel

# Velocity of the auger, on a scale from -1 (spinning backwards at
# full throttle) to 1 (spinning forwards at full throttle).
float64 spin

# We may or may not change this to `depth` as a position instead of
# `depth_vel` as a velocity at some point, depending...
