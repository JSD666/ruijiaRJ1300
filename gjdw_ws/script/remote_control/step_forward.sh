#rostopic pub -r 1 /nari/szrd/dnrobot/cmd public_pkg/cmd_msg "header:
rostopic pub --once /nari/szrd/dnrobot/cmd public_pkg/cmd_msg "header:
  msgID: 0
  srcNodeType: 'RemoteControl'
  srcNodeName: 'RemoteControl'
  dstNodeType: 'MainControl'
  dstNodeName: 'MainControl'
  msgType: 'public_pkg/cmd_msg'
  msgTime: {secs: 0, nsecs: 0}
object: 200
type: 0
content: '{\"cmdArray\": [{\"seq\":15,\"type\":\"uint8\",\"value\":1,\"name\":\"cmd\"}]}'"
