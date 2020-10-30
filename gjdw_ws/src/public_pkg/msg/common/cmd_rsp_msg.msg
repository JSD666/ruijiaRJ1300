cmd_header_msg   header  #消息头
uint16           object  #控制对象
uint16           type    #控制类型
int32            result  #控制结果,例如成功或失败
int32            errCode #控制错误码
string           content #控制量返回消息内容,使用json格式
