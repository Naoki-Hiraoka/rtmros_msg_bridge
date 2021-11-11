#ifndef WrenchStampedROSBridge_H
#define WrenchStampedROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class WrenchStampedROSBridge : public RTC::DataFlowComponentBase{
protected:
  RTC::TimedDoubleSeq m_wrenchRTM_;
  RTC::InPort<RTC::TimedDoubleSeq> m_wrenchIn_;
  ros::Publisher pub_;

  RTC::TimedDoubleSeq m_wrenchROS_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_wrenchOut_;
  ros::Subscriber sub_;

  std::string frame_id_;

  void topicCb(geometry_msgs::WrenchStamped::ConstPtr msg);

public:
  WrenchStampedROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

};


extern "C"
{
  void WrenchStampedROSBridgeInit(RTC::Manager* manager);
};

#endif // WrenchStampedROSBridge_H
