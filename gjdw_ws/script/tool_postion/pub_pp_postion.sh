rostopic pub --once /l_arm_controller/tool_point geometry_msgs/TwistStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base'
twist:
  linear:
    x: 0.2
    y: 0.1
    z: 2.0
  angular:
    x: 1.0
    y: 0.3
    z: 0.1" 

