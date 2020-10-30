#rostopic pub --once /l_arm_controller/ur_driver/io_states ur_msgs/IOStates "digital_in_states:
rostopic pub --once /l_arm_controller/ur_driver/io_states ur_msgs/IOStates "digital_in_states:
- {pin: 0, state: false}
digital_out_states:
- {pin: 0, state: true}
flag_states:
- {pin: 0, state: false}
analog_in_states:
- {pin: 0, state: 0.0}
analog_out_states:
- {pin: 0, state: 0.0}" 

