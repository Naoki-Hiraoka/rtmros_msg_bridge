#ifndef PoseStampedROSBridge_H
#define PoseStampedROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class PoseStampedROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh; // これがないとうまく通信できなくなったり、CPU使用率100%になったりする

  RTC::TimedPose3D m_poseRTM_;
  RTC::InPort<RTC::TimedPose3D> m_poseIn_;
  ros::Publisher pub_;

  RTC::TimedPose3D m_poseROS_;
  RTC::OutPort<RTC::TimedPose3D> m_poseOut_;
  ros::Subscriber sub_;

  std::string frame_id_;

  void topicCb(geometry_msgs::PoseStamped::ConstPtr msg);

public:
  PoseStampedROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

};


extern "C"
{
  void PoseStampedROSBridgeInit(RTC::Manager* manager);
};

#endif // PoseStampedROSBridge_H
