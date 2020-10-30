send_cmd_to_power_manager()
{
rostopic pub --once /nari/szrd/dnrobot/cmd public_pkg/cmd_msg "header:
  msgID: 0
  srcNodeType: 'RemoteControl'
  srcNodeName: 'RemoteControl'
  dstNodeType: 'MainControl'
  dstNodeName: 'MainControl'
  msgType: 'public_pkg/cmd_msg'
  msgTime: {secs: 0, nsecs: 0}
object: 100
type: 0
content: '{\"cmdArray\": [{\"seq\":$1,\"type\":\"uint8\",\"value\":1,\"name\":\"cmd\"}]}'"
}

send_cmd_to_power_manager 1
#send_cmd_to_power_manager 2
#send_cmd_to_power_manager 3
#send_cmd_to_power_manager 4
