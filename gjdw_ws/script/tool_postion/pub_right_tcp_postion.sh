rostopic pub --once /nari/szrd/dnrobot/yc public_pkg/status_analog_msg "header:
  srcNodeType: 'arm'
  srcNodeName: 'armsInfoAnalog'
  msgType: 'public_pkg/status_analog_msg'
  msgTime:
    secs: 0
    nsecs: 0
content: '{\"analogArray\": [{\"seq\":101,\"type\":\"double\",\"value\":1.2590765953063965},{\"seq\":102,\"type\":\"double\",\"value\":-2.003519360219137},{\"seq\":103,\"type\":\"double\",\"value\":-2.037025753651754},{\"seq\":104,\"type\":\"double\",\"value\":-0.8542559782611292},{\"seq\":105,\"type\":\"double\",\"value\":-1.6500862280475062},{\"seq\":106,\"type\":\"double\",\"value\":0.34261253476142886}]}'"
