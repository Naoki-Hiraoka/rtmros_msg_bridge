#include "OctomapWithPoseROSBridge.h"
#include <Eigen/Eigen>

OctomapWithPoseROSBridge::OctomapWithPoseROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_In_("octomapWithPoseIn", m_inData_),
  m_Out_("octomapWithPoseOut", m_outData_)
{
}

RTC::ReturnCode_t OctomapWithPoseROSBridge::onInitialize(){
  addInPort("octomapWithPoseIn", this->m_In_);
  addOutPort("octomapWithPoseOut", this->m_Out_);

  ros::NodeHandle pnh("~");
  pnh.param("frame_id", frame_id_, std::string(""));
  sub_ = pnh.subscribe("input", 1, &OctomapWithPoseROSBridge::topicCb, this);
  pub_ = pnh.advertise<octomap_msgs::OctomapWithPose>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t OctomapWithPoseROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_In_.isNew()){
    this->m_In_.read();
    octomap_msgs::OctomapWithPose msg;
    ros::Time now = ros::Time::now();
    msg.header.stamp = now;
    msg.header.frame_id = this->frame_id_;
    msg.origin.position.x = this->m_inData_.data.origin.position.x;
    msg.origin.position.y = this->m_inData_.data.origin.position.y;
    msg.origin.position.z = this->m_inData_.data.origin.position.z;
    Eigen::Quaterniond q = Eigen::AngleAxisd(this->m_inData_.data.origin.orientation.y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(this->m_inData_.data.origin.orientation.p, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(this->m_inData_.data.origin.orientation.r, Eigen::Vector3d::UnitX());
    msg.origin.orientation.x = q.x();
    msg.origin.orientation.y = q.y();
    msg.origin.orientation.z = q.z();
    msg.origin.orientation.w = q.w();
    msg.octomap.header.stamp = now;
    msg.octomap.header.frame_id = this->frame_id_;
    msg.octomap.binary = this->m_inData_.data.octomap.binary;
    msg.octomap.id = this->m_inData_.data.octomap.id;
    msg.octomap.resolution = this->m_inData_.data.octomap.resolution;
    msg.octomap.data.resize(this->m_inData_.data.octomap.data.length());
    for(int i=0;i<this->m_inData_.data.octomap.data.length();i++){
      msg.octomap.data[i] = this->m_inData_.data.octomap.data[i];
    }
    this->pub_.publish(msg);
  }
  return RTC::RTC_OK;
}

void OctomapWithPoseROSBridge::topicCb(octomap_msgs::OctomapWithPose::ConstPtr msg){
  m_outData_.tm.sec = msg->header.stamp.sec;
  m_outData_.tm.nsec = msg->header.stamp.nsec;
  m_outData_.data.origin.position.x = msg->origin.position.x;
  m_outData_.data.origin.position.y = msg->origin.position.y;
  m_outData_.data.origin.position.z = msg->origin.position.z;
  // tf2::Matrix3x3::getEulerYPRやtf2::Matrix3x3::getRPYにはバグがある https://github.com/ros/geometry2/issues/504
  Eigen::Vector3d ypr = Eigen::Quaterniond(msg->origin.orientation.w,msg->origin.orientation.x,msg->origin.orientation.y,msg->origin.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
  m_outData_.data.origin.orientation.r = ypr[2];
  m_outData_.data.origin.orientation.p = ypr[1];
  m_outData_.data.origin.orientation.y = ypr[0];
  m_outData_.data.octomap.binary = msg->octomap.binary;
  m_outData_.data.octomap.id = msg->octomap.id.c_str();
  m_outData_.data.octomap.resolution = msg->octomap.resolution;
  m_outData_.data.octomap.data.length(msg->octomap.data.size());
  for(int i=0;i<msg->octomap.data.size();i++){
    m_outData_.data.octomap.data[i] =  msg->octomap.data[i];
  }
  m_Out_.write();
}

static const char* OctomapWithPoseROSBridge_spec[] = {
  "implementation_id", "OctomapWithPoseROSBridge",
  "type_name",         "OctomapWithPoseROSBridge",
  "description",       "OctomapWithPoseROSBridge component",
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
    void OctomapWithPoseROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(OctomapWithPoseROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<OctomapWithPoseROSBridge>, RTC::Delete<OctomapWithPoseROSBridge>);
    }
};
