# Pose.msg

std_msgs/Header header

# Position. m
float64 easting
float64 northing
float64 height

# Attitude. Radians. Yaw is counterclockwise from east
float64 roll
float64 pitch
float64 yaw

# Velocity. Vehicle frame. m/s. x is forward, z is up.
float64 velocity_x
float64 velocity_y
float64 velocity_z

# Angular velocity. rad/s
float64 pitch_rate
float64 roll_rate
float64 yaw_rate

# Acceleration. Vehicle frame. m/s^2.
float64 accel_x
float64 accel_y
float64 accel_z

