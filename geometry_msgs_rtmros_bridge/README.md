# geometry_msgs_rtmros_bridge
## PoseStampedROSBridge
### Subscribe
- `~input`(geometry_msgs/PoseStamped)
### Publish
- `~output`(geometry_msgs/PoseStamped)
### InPort
- `poseIn`(RTC::TimedPose3D)
### OutPort
- `poseOut`(RTC::TimedPose3D)
### ROS Param
- `~frame_id`(String) : for `~output`
## WrenchStampedROSBridge
### Subscribe
- `~input`(geometry_msgs/WrenchStamped)
### Publish
- `~output`(geometry_msgs/WrenchStamped)
### InPort
- `wrenchIn`(RTC::TimedDoubleSeq)
### OutPort
- `wrenchOut`(RTC::TimedDoubleSeq)
### ROS Param
- `~frame_id`(String) : for `~output`
