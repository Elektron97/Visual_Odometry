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

#Scale Factor
float64 SF 
