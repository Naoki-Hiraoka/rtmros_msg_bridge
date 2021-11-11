#ifndef OdometryROSBridge_H
#define OdometryROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class OdometryROSBridge : public RTC::DataFlowComponentBase{
protected:
  RTC::TimedPose3D m_poseRTM_;
  RTC::InPort<RTC::TimedPose3D> m_poseIn_;
  ros::Publisher pub_;

  RTC::TimedPose3D m_poseROS_;
  RTC::OutPort<RTC::TimedPose3D> m_poseOut_;
  ros::Subscriber sub_;

  std::string frame_id_;
  std::string child_frame_id_;

  void topicCb(nav_msgs::Odometry::ConstPtr msg);

public:
  OdometryROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

};


extern "C"
{
  void OdometryROSBridgeInit(RTC::Manager* manager);
};

#endif // OdometryROSBridge_H
