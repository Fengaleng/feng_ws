Header header

# state
geometry_msgs/Vector3 position_W
geometry_msgs/Vector3 velocity_W
geometry_msgs/Vector3 euler_angle
geometry_msgs/Vector3 rotation_speed_B

# control
float64 commanded_thrust
geometry_msgs/Vector3 moment

# ref trajectory
geometry_msgs/Vector3 position_ref
geometry_msgs/Vector3 velocity_ref
geometry_msgs/Vector3 accel_ref
float64 yaw_ref
float64 speed
bool launch_flag
