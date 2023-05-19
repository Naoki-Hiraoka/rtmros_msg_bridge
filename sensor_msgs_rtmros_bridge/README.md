# sensor_msgs_rtmros_bridge
## JointStateROSBridge
### Subscribe
- `~input`(sensor_msgs/JointState)
### Publish
- `~output`(sensor_msgs/JointState)
### InPort
- `qIn`(RTC::TimedDoubleSeq)
### OutPort
- `qOut`(RTC::TimedDoubleSeq)
### ROS Param
- `~model`(String) : choreonoid file name

## JoyROSBridge
OutPort type is same as https://github.com/fkanehiro/hrpsys-base/tree/master/rtc/Joystick
### Subscribe
- `~input`(sensor_msgs/Joy)
### OutPort
- `axesOut`(RTC::TimedFloatSeq)
- `buttonsOut`(RTC::TimedBooleanSeq)
