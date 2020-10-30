rostopic pub --once /nari/szrd/dnrobot/yc public_pkg/status_analog_msg "header:
  srcNodeType: 'arm'
  srcNodeName: 'armsInfoAnalog'
  msgType: 'public_pkg/status_analog_msg'
  msgTime:
    secs: 0
    nsecs: 0
content: '{\"analogArray\":[{\"seq\":1,\"type\":\"double\",\"value\":1.2590765953063965},{\"seq\":2,\"type\":\"double\",\"value\":-2.003519360219137},{\"seq\":3,\"type\":\"double\",\"value\":-2.037025753651754},{\"seq\":4,\"type\":\"double\",\"value\":-0.8542559782611292},{\"seq\":5,\"type\":\"double\",\"value\":-1.6500862280475062},{\"seq\":6,\"type\":\"double\",\"value\":0.34261253476142886}]}'"
