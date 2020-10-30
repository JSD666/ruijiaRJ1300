rostopic pub --once /nari/szrd/dnrobot/yx public_pkg/status_digital_msg "header:
  srcNodeType: 'ClawTool'
  srcNodeName: 'ClawTool'
  msgType: 'public_pkg/status_digital_msg'
  msgTime:
    secs: 0
    nsecs: 0
content: '{\"digitalArray\": [{\"seq\": 3, \"value\": 0}]}'"
