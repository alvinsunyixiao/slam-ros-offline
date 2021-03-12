#pragma once

#include <functional>
#include <queue>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

class ZedSyncer {
 public:
  using CallbackType = std::function<void(const sensor_msgs::ImageConstPtr&,
                                          const sensor_msgs::ImageConstPtr&,
                                          const std::vector<sensor_msgs::ImuConstPtr>&)>;

  ZedSyncer();

  ~ZedSyncer();

  void RegisterCallback(const CallbackType& callback);

 private:
  void HandleIMU(const sensor_msgs::ImuConstPtr& msg);

  void HandleStereo(const sensor_msgs::ImageConstPtr& left_msg,
                    const sensor_msgs::ImageConstPtr& right_msg);

  void TryInvokeSync();

  void HandleSyncedSequence(const sensor_msgs::ImageConstPtr& left_img_msg,
                            const sensor_msgs::ImageConstPtr& right_img_msg,
                            const std::vector<sensor_msgs::ImuConstPtr>& imu_msgs);

  ros::NodeHandle n_;
  ros::Subscriber sub_imu_;
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left_;
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_stereo_;

  std::vector<sensor_msgs::ImuConstPtr> imu_q_;
  std::queue<std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>> stereo_q_;
  ros::AsyncSpinner spinner_;
  CallbackType callback_;
};

