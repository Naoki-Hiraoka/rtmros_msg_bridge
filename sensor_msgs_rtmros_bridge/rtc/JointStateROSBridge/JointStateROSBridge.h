#ifndef JointStateROSBridge_H
#define JointStateROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cnoid/Body>

class JointStateROSBridge : public RTC::DataFlowComponentBase{
protected:
  cnoid::BodyPtr robot_vrml_;

  RTC::TimedDoubleSeq m_qRTM_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
  ros::Publisher pub_;

  RTC::TimedDoubleSeq m_qROS_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_qOut_;
  ros::Subscriber sub_;

  void topicCb(sensor_msgs::JointState::ConstPtr msg);

public:
  JointStateROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

};


extern "C"
{
  void JointStateROSBridgeInit(RTC::Manager* manager);
};

#endif // JointStateROSBridge_H
