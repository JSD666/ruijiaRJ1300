rostopic pub --once /nari/szrd/dnrobot/yc public_pkg/status_analog_msg "header:
  srcNodeType: 'Slide'
  srcNodeName: 'Slide'
  msgType: 'public_pkg/status_analog_msg'
  msgTime:
    secs: 0
    nsecs: 0
content: '{\"analogArray\": [{\"seq\":1,\"value\":100}]}'"
