rostopic pub --once /nari/szrd/dnrobot/yx public_pkg/status_digital_msg "header:
  srcNodeType: 'CutWireTool'
  srcNodeName: 'CutWireTool'
  msgType: 'public_pkg/status_digital_msg'
  msgTime:
    secs: 0
    nsecs: 0
content: '{\"digitalArray\": [{\"seq\": 4, \"value\": 2}]}'"
