#rostopic pub -r 1 /nari/szrd/dnrobot/cmd public_pkg/cmd_msg "header:
rostopic pub --once /nari/szrd/dnrobot/cmd public_pkg/cmd_msg "header:
  msgID: 0
  srcNodeType: 'RemoteControlTest'
  srcNodeName: 'RemoteControlTest'
  dstNodeType: 'MainControl'
  dstNodeName: 'MainControl'
  msgType: 'public_pkg/cmd_msg'
  msgTime: {secs: 0, nsecs: 0}
object: 200
type: 0
content: '{\"cmdArray\": [{\"seq\":1,\"type\":\"uint8_t[]\",\"value\":[3,1,1,1,4,1,56,0,0,0],\"name\":\"cmd\"}]}'"
