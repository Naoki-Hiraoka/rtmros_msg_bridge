# nav_msgs_rtmros_bridge
## OdometryROSBridge
### Subscribe
- `~input`(nav_msgs/Odometry)
### Publish
- `~output`(nav_msgs/Odometry)
### InPort
- `poseIn`(RTC::TimedPose3D)
### OutPort
- `poseOut`(RTC::TimedPose3D)
### ROS Param
- `~frame_id`(String) : for `~output`
- `~child_frame_id`(String) : for `~output`