#include "WrenchStampedROSBridge.h"

WrenchStampedROSBridge::WrenchStampedROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_wrenchIn_("wrenchIn", m_wrenchRTM_),
  m_wrenchOut_("wrenchOut", m_wrenchROS_)
{
}

RTC::ReturnCode_t WrenchStampedROSBridge::onInitialize(){
  addInPort("wrenchIn", this->m_wrenchIn_);
  addOutPort("wrenchOut", this->m_wrenchOut_);

  m_wrenchROS_.data.length(6);

  ros::NodeHandle pnh("~");
  pnh.param("frame_id", frame_id_, std::string(""));
  sub_ = pnh.subscribe("input", 1, &WrenchStampedROSBridge::topicCb, this);
  pub_ = pnh.advertise<geometry_msgs::WrenchStamped>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t WrenchStampedROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_wrenchIn_.isNew()){
    this->m_wrenchIn_.read();
    if(this->m_wrenchRTM_.data.length() == 6){
      geometry_msgs::WrenchStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = this->frame_id_;
      msg.wrench.force.x = this->m_wrenchRTM_.data[0];
      msg.wrench.force.y = this->m_wrenchRTM_.data[1];
      msg.wrench.force.z = this->m_wrenchRTM_.data[2];
      msg.wrench.torque.x = this->m_wrenchRTM_.data[3];
      msg.wrench.torque.y = this->m_wrenchRTM_.data[4];
      msg.wrench.torque.z = this->m_wrenchRTM_.data[5];
      this->pub_.publish(msg);
    }
  }
  return RTC::RTC_OK;
}

void WrenchStampedROSBridge::topicCb(geometry_msgs::WrenchStamped::ConstPtr msg){
  m_wrenchROS_.tm.sec = msg->header.stamp.sec;
  m_wrenchROS_.tm.nsec = msg->header.stamp.nsec;
  m_wrenchROS_.data[0] = msg->wrench.force.x;
  m_wrenchROS_.data[1] = msg->wrench.force.y;
  m_wrenchROS_.data[2] = msg->wrench.force.z;
  m_wrenchROS_.data[3] = msg->wrench.torque.x;
  m_wrenchROS_.data[4] = msg->wrench.torque.y;
  m_wrenchROS_.data[5] = msg->wrench.torque.z;
  m_wrenchOut_.write();
}

static const char* WrenchStampedROSBridge_spec[] = {
  "implementation_id", "WrenchStampedROSBridge",
  "type_name",         "WrenchStampedROSBridge",
  "description",       "WrenchStampedROSBridge component",
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
    void WrenchStampedROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(WrenchStampedROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<WrenchStampedROSBridge>, RTC::Delete<WrenchStampedROSBridge>);
    }
};
