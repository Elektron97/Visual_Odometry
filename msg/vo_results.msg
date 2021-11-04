#Custom msg for VO results
#Estimated Pose
geometry_msgs/Vector3 estimate_pos  #{ENU} -> {C_k} in {ENU}    [m]
geometry_msgs/Vector3 estimate_rpy  #{ENU} -> {C_k}             [rad]

#Estimated Twist
geometry_msgs/Twist estimate_twist  #[m/s] and [rad/s]

#Error
geometry_msgs/Vector3 error_pos #[m]
geometry_msgs/Vector3 error_rpy #[rad]
geometry_msgs/Twist error_twist #[m/s] and [rad/s]

#Failure Detection and CheckIfMoving history
#int8 fail_msg   #{0, 1, 2, 3}
bool fail_detect
bool not_moving
bool fail_pose
bool success
