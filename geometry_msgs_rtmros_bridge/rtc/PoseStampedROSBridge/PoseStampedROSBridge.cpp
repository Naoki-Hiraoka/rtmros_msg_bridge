#include "PoseStampedROSBridge.h"
#include <Eigen/Eigen>
#include <tf2/utils.h>

PoseStampedROSBridge::PoseStampedROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_poseIn_("poseIn", m_poseRTM_),
  m_poseOut_("poseOut", m_poseROS_)
{
}

RTC::ReturnCode_t PoseStampedROSBridge::onInitialize(){
  addInPort("poseIn", this->m_poseIn_);
  addOutPort("poseOut", this->m_poseOut_);

  ros::NodeHandle pnh("~");
  pnh.param("frame_id", frame_id_, std::string(""));
  sub_ = pnh.subscribe("input", 1, &PoseStampedROSBridge::topicCb, this);
  pub_ = pnh.advertise<geometry_msgs::PoseStamped>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t PoseStampedROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_poseIn_.isNew()){
    this->m_poseIn_.read();
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = this->frame_id_;
    msg.pose.position.x = this->m_poseRTM_.data.position.x;
    msg.pose.position.y = this->m_poseRTM_.data.position.y;
    msg.pose.position.z = this->m_poseRTM_.data.position.z;
    tf2::Quaternion q;
    q.setRPY(m_poseRTM_.data.orientation.r,
             m_poseRTM_.data.orientation.p,
             m_poseRTM_.data.orientation.y);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    this->pub_.publish(msg);
  }
  return RTC::RTC_OK;
}

void PoseStampedROSBridge::topicCb(geometry_msgs::PoseStamped::ConstPtr msg){
  m_poseROS_.tm.sec = msg->header.stamp.sec;
  m_poseROS_.tm.nsec = msg->header.stamp.nsec;
  m_poseROS_.data.position.x = msg->pose.position.x;
  m_poseROS_.data.position.y = msg->pose.position.y;
  m_poseROS_.data.position.z = msg->pose.position.z;
  // tf2::Matrix3x3::getEulerYPRやtf2::Matrix3x3::getRPYにはバグがある https://github.com/ros/geometry2/issues/504
  Eigen::Vector3d ypr = Eigen::Quaterniond(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
  m_poseROS_.data.orientation.r = ypr[2];
  m_poseROS_.data.orientation.p = ypr[1];
  m_poseROS_.data.orientation.y = ypr[0];
  m_poseOut_.write();
}

static const char* PoseStampedROSBridge_spec[] = {
  "implementation_id", "PoseStampedROSBridge",
  "type_name",         "PoseStampedROSBridge",
  "description",       "PoseStampedROSBridge component",
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
    void PoseStampedROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(PoseStampedROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<PoseStampedROSBridge>, RTC::Delete<PoseStampedROSBridge>);
    }
};
