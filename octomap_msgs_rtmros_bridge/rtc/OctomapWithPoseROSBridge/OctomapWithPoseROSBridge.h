#ifndef OctomapWithPoseROSBridge_H
#define OctomapWithPoseROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <ros/ros.h>
#include <octomap_msgs_rtmros_bridge/idl/Octomap.hh>
#include <octomap_msgs/OctomapWithPose.h>

class OctomapWithPoseROSBridge : public RTC::DataFlowComponentBase{
protected:
  octomap_msgs_rtmros_bridge::TimedOctomapWithPose m_inData_;
  RTC::InPort<octomap_msgs_rtmros_bridge::TimedOctomapWithPose> m_In_;
  ros::Publisher pub_;

  octomap_msgs_rtmros_bridge::TimedOctomapWithPose m_outData_;
  RTC::OutPort<octomap_msgs_rtmros_bridge::TimedOctomapWithPose> m_Out_;
  ros::Subscriber sub_;

  std::string frame_id_;

  void topicCb(octomap_msgs::OctomapWithPose::ConstPtr msg);

public:
  OctomapWithPoseROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

};


extern "C"
{
  void OctomapWithPoseROSBridgeInit(RTC::Manager* manager);
};

#endif // OctomapWithPoseROSBridge_H
