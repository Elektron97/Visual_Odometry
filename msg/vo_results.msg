#Custom msg for VO results
#Estimated Pose
geometry_msgs/Vector3 estimate_pos  #{ENU} -> {C_k} in {ENU}
geometry_msgs/Vector3 estimate_rpy  #{ENU} -> {C_k}

#Estimated Twist
geometry_msgs/Twist estimate_twist

#Error
geometry_msgs/Vector3 error_pos
geometry_msgs/Vector3 error_rpy
geometry_msgs/Twist error_twist
