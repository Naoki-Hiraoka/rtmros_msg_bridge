#include "JoyROSBridge.h"

JoyROSBridge::JoyROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_axesOut_("axesOut", m_axesROS_),
  m_buttonsOut_("buttonsOut", m_buttonsROS_)
{
}

RTC::ReturnCode_t JoyROSBridge::onInitialize(){
  addOutPort("axesOut", this->m_axesOut_);
  addOutPort("buttonsOut", this->m_buttonsOut_);

  ros::NodeHandle pnh("~");

  sub_ = pnh.subscribe("input", 1, &JoyROSBridge::topicCb, this);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t JoyROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();
  return RTC::RTC_OK;
}

void JoyROSBridge::topicCb(sensor_msgs::Joy::ConstPtr msg){
  m_axesROS_.data.length(msg->axes.size());
  for(int i=0;i<msg->axes.size();i++) m_axesROS_.data[i] = msg->axes[i];
  m_axesOut_.write();
  m_buttonsROS_.data.length(msg->buttons.size());
  for(int i=0;i<msg->buttons.size();i++) m_buttonsROS_.data[i] = msg->buttons[i]; // int32 -> bool
  m_buttonsOut_.write();
}

static const char* JoyROSBridge_spec[] = {
  "implementation_id", "JoyROSBridge",
  "type_name",         "JoyROSBridge",
  "description",       "JoyROSBridge component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void JoyROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(JoyROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<JoyROSBridge>, RTC::Delete<JoyROSBridge>);
    }
};
