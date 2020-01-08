#include "bluefox2/single_node.h"
#include "bluefox2/bluefox2_ros.h"

namespace bluefox2 {

SingleNode::SingleNode(const ros::NodeHandle& pnh)
    : CameraNodeBase(pnh),
      bluefox2_ros_(boost::make_shared<Bluefox2Ros>(pnh)),
      use_embedded_time_(true), tune_exposure_time_(-1.0),
      tune_interval_over_(true) {
  pnh.getParam("use_embedded_time", use_embedded_time_);
  if (use_embedded_time_)
    ROS_INFO("Use embedded device timestamps");
  else
    ROS_INFO("Use ros::Time::now() for timestamps");
  pnh.getParam("tune_exposure_time", tune_exposure_time_);
  if (tune_exposure_time_ > 0.0) {
    tune_interval_over_ = false;
    ROS_INFO("Enable auto-exposure for %f s after initialization, then disable it and freeze the exposure time", tune_exposure_time_);
  }
}

void SingleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    bluefox2_ros_->RequestSingle();
    const auto expose_duration = bluefox2_ros_->camera().GetExposeUs() * 1e-6 / 2;
    double expose_start = static_cast<double>(bluefox2_ros_->camera().GetExposeStartUs() * 1e-6);
    double hardware_cur_time = expose_start + static_cast<double>(expose_duration);
    SyncBaseTime(hardware_cur_time);
    ros::Time software_cur_time = ros::Time(hardware_cur_time + offset_time_);
    ros::Time from_ros_cur_time = ros::Time::now() + ros::Duration(expose_duration);
    double dt = (software_cur_time - from_ros_cur_time).toSec();
    if (!tune_interval_over_ && hardware_cur_time - init_hardware_time_ > tune_exposure_time_) {
      int exptime = bluefox2_ros_->camera().GetExposeUs();
      ROS_INFO("Disable auto-exposure, set expose us = %d", exptime);
      tune_interval_over_ = true;
      bluefox2_ros_->camera().SetOnlyAec(false);
    }
    if (!use_embedded_time_)
      software_cur_time = from_ros_cur_time;
    bluefox2_ros_->PublishCameraAndOffset(software_cur_time, dt);
    Sleep();
  }
}

void SingleNode::AcquireOnce() {
  if (is_acquire() && ros::ok()) {
    bluefox2_ros_->RequestSingle();
    const auto expose_duration = bluefox2_ros_->camera().GetExposeUs() * 1e-6 / 2;
    double expose_start = static_cast<double>(bluefox2_ros_->camera().GetExposeStartUs() * 1e-6);
    double hardware_cur_time = expose_start + static_cast<double>(expose_duration);
    SyncBaseTime(hardware_cur_time);
    ros::Time software_cur_time = ros::Time(hardware_cur_time + offset_time_);
    if (!use_embedded_time_)
      software_cur_time = ros::Time::now() + ros::Duration(expose_duration);
    bluefox2_ros_->PublishCamera(software_cur_time);
  }
}

void SingleNode::Setup(Bluefox2DynConfig& config) {
  bluefox2_ros_->set_fps(config.fps);
  bluefox2_ros_->camera().Configure(config);
}

void SingleNode::SyncBaseTime(const double& hardware_time) {
  if (base_time_set_)
    return;
  else {
    // offset_time_ = software base time - hardware base time
    offset_time_ = ros::Time::now().toSec() - hardware_time;
    ROS_INFO("Initial Sync: \nhardware time = %f\nsoftware time = %f\noffset time = %f",
             hardware_time, offset_time_ + hardware_time, offset_time_);
    if (hardware_time > 0.0)
      base_time_set_ = true;
    if (tune_exposure_time_ > 0.0) {
      ROS_INFO("Enable auto-exposure temporarily");
      init_hardware_time_ = hardware_time;
      bluefox2_ros_->camera().SetOnlyAec(true);
    }
  }
}

}  // namepace bluefox2
