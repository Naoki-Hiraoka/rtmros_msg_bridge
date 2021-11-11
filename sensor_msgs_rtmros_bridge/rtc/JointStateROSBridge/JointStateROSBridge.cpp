#include "JointStateROSBridge.h"

#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>

JointStateROSBridge::JointStateROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_qIn_("qIn", m_qRTM_),
  m_qOut_("qOut", m_qROS_)
{
}

RTC::ReturnCode_t JointStateROSBridge::onInitialize(){
  addInPort("qIn", this->m_qIn_);
  addOutPort("qOut", this->m_qOut_);

  cnoid::BodyLoader bodyLoader;

  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  this->robot_vrml_ = bodyLoader.load(fileName);
  if(!this->robot_vrml_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  m_qROS_.data.length(robot_vrml_->numJoints());
  for(int i=0;i<robot_vrml_->numJoints();i++) m_qROS_.data[i] = 0.0;

  ros::NodeHandle pnh("~");
  sub_ = pnh.subscribe("input", 1, &JointStateROSBridge::topicCb, this);
  pub_ = pnh.advertise<sensor_msgs::JointState>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t JointStateROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_qIn_.isNew()){
    this->m_qIn_.read();
    if(this->m_qRTM_.data.length() == robot_vrml_->numJoints()){
      sensor_msgs::JointState msg;
      msg.header.stamp = ros::Time::now();
      for(int i=0;i<robot_vrml_->numJoints();i++){
        msg.name.push_back(robot_vrml_->joint(i)->name());
        msg.position.push_back(m_qRTM_.data[i]);
      }
      this->pub_.publish(msg);
    }
  }
  return RTC::RTC_OK;
}

void JointStateROSBridge::topicCb(sensor_msgs::JointState::ConstPtr msg){
  if(msg->name.size() != msg->position.size()) return;
  for(int i=0;i<msg->name.size();i++){
    const cnoid::LinkPtr& link = robot_vrml_->link(msg->name[i]);
    if(!link) return;
    m_qROS_.data[link->jointId()] = msg->position[i];
  }
  m_qOut_.write();
}

static const char* JointStateROSBridge_spec[] = {
  "implementation_id", "JointStateROSBridge",
  "type_name",         "JointStateROSBridge",
  "description",       "JointStateROSBridge component",
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
    void JointStateROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(JointStateROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<JointStateROSBridge>, RTC::Delete<JointStateROSBridge>);
    }
};
