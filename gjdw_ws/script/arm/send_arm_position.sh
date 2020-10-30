rostopic pub --once /l_arm_controller/tool_point geometry_msgs/Twist "{linear:  {x: 0.1, y: 0.2, z: 0.3}, angular: {x: 0.5,y: 0.6, z: 0.7}}"
rostopic pub --once /r_arm_controller/tool_point geometry_msgs/Twist "{linear:  {x: 0.4, y: 0.5, z: 0.6}, angular: {x: 0.5,y: 0.6, z: 0.7}}"
