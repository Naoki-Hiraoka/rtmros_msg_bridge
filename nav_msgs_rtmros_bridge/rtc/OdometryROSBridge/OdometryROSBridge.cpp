#include "OdometryROSBridge.h"
#include <tf2/utils.h>

OdometryROSBridge::OdometryROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_poseIn_("poseIn", m_poseRTM_),
  m_poseOut_("poseOut", m_poseROS_)
{
}

RTC::ReturnCode_t OdometryROSBridge::onInitialize(){
  addInPort("poseIn", this->m_poseIn_);
  addOutPort("poseOut", this->m_poseOut_);

  ros::NodeHandle pnh("~");
  pnh.param("frame_id", frame_id_, std::string("odom"));
  pnh.param("child_frame_id", child_frame_id_, std::string("base_link"));
  sub_ = pnh.subscribe("input", 1, &OdometryROSBridge::topicCb, this);
  pub_ = pnh.advertise<nav_msgs::Odometry>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t OdometryROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_poseIn_.isNew()){
    this->m_poseIn_.read();
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = this->frame_id_;
    odom.child_frame_id = this->child_frame_id_;
    odom.pose.pose.position.x = m_poseRTM_.data.position.x;
    odom.pose.pose.position.y = m_poseRTM_.data.position.y;
    odom.pose.pose.position.z = m_poseRTM_.data.position.z;
    tf2::Quaternion q;
    q.setRPY(m_poseRTM_.data.orientation.r,
             m_poseRTM_.data.orientation.p,
             m_poseRTM_.data.orientation.y);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    for(int i=0;i<6;i++){
      for(int j=0;j<6;j++){
        if(i==j) odom.pose.covariance[i*6+j] = 0.0;
        else odom.pose.covariance[i*6+j] = 0.0;
      }
    }
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;
    for(int i=0;i<6;i++){
      for(int j=0;j<6;j++){
        if(i==j) odom.twist.covariance[i*6+j] = 0.0;
        else odom.twist.covariance[i*6+j] = 0.0;
      }
    }

    this->pub_.publish(odom);
  }
  return RTC::RTC_OK;
}

void OdometryROSBridge::topicCb(nav_msgs::Odometry::ConstPtr msg){
  m_poseROS_.tm.sec = msg->header.stamp.sec;
  m_poseROS_.tm.nsec = msg->header.stamp.nsec;
  m_poseROS_.data.position.x = msg->pose.pose.position.x;
  m_poseROS_.data.position.y = msg->pose.pose.position.y;
  m_poseROS_.data.position.z = msg->pose.pose.position.z;
  tf2::Quaternion quat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  tf2::Matrix3x3(quat).getRPY(m_poseROS_.data.orientation.r,m_poseROS_.data.orientation.p,m_poseROS_.data.orientation.y);
  m_poseOut_.write();

}

static const char* OdometryROSBridge_spec[] = {
  "implementation_id", "OdometryROSBridge",
  "type_name",         "OdometryROSBridge",
  "description",       "OdometryROSBridge component",
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
    void OdometryROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(OdometryROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<OdometryROSBridge>, RTC::Delete<OdometryROSBridge>);
    }
};
