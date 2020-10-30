send_cmd()
{
rostopic pub --once /nari/szrd/dnrobot/cmd public_pkg/cmd_msg "header:
  msgID: 0
  srcNodeType: 'RemoteControl'
  srcNodeName: 'RemoteControl'
  dstNodeType: 'MainControl'
  dstNodeName: 'MainControl'
  msgType: 'public_pkg/cmd_msg'
  msgTime: {secs: 0, nsecs: 0}
object: 301
type: 0
content: '{\"cmdArray\": [{\"seq\":$1,\"type\":\"uint8\",\"value\":1,\"name\":\"cmd\"}]}'"
}

send_cmd 1
send_cmd 2
send_cmd 3
send_cmd 4
send_cmd 5
send_cmd 6
send_cmd 7
send_cmd 8
send_cmd 9
send_cmd 10
send_cmd 11
send_cmd 12

send_cmd 13
send_cmd 14
send_cmd 15
send_cmd 16
send_cmd 17
send_cmd 18

send_cmd 19
send_cmd 20
send_cmd 21
send_cmd 22
send_cmd 23
send_cmd 24

send_cmd 25
send_cmd 26
send_cmd 27
send_cmd 28
send_cmd 29
send_cmd 30

send_cmd 31
send_cmd 32
send_cmd 33
send_cmd 34
send_cmd 35
send_cmd 36

send_cmd 37

send_cmd 38
