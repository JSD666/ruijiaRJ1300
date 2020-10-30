rostopic pub -r 1 /nari/szrd/dnrobot/yx public_pkg/status_digital_msg  "header:
  srcNodeType: 'arm'
  srcNodeName: 'armsInfoDigital'
  msgType: 'public_pkg/status_digital_msg'
  msgTime:
    secs: 0
    nsecs: 0
content: '{\"digitalArray\":[{\"seq\":8,\"type\":\"uint\",\"value\":1},{\"seq\":10,\"type\"\
  :\"uint\",\"value\":1},{\"seq\":10,\"type\":\"uint\",\"value\":1},{\"seq\":108,\"type\"\
  :\"uint\",\"value\":1}]}'" 
