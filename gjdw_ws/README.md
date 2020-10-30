# catkin_ws

ROS工作空间，用于研发和测试

如遇到找不到msg文件，可先单独编译对应消息文件
例如：
catkin_make -DCATKIN_WHITELIST_PACKAGES="public_pkg"

catkin_make -DCATKIN_WHITELIST_PACKAGES="fusion"

恢复所有
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
