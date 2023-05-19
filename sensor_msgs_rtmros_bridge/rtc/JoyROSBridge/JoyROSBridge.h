#ifndef JoyROSBridge_H
#define JoyROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class JoyROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh; // これがないとうまく通信できなくなったり、CPU使用率100%になったりする

  RTC::TimedFloatSeq m_axesROS_;
  RTC::OutPort<RTC::TimedFloatSeq> m_axesOut_;
  RTC::TimedBooleanSeq m_buttonsROS_;
  RTC::OutPort<RTC::TimedBooleanSeq> m_buttonsOut_;
  ros::Subscriber sub_;

  void topicCb(sensor_msgs::Joy::ConstPtr msg);

public:
  JoyROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

};


extern "C"
{
  void JoyROSBridgeInit(RTC::Manager* manager);
};

#endif // JoyROSBridge_H
