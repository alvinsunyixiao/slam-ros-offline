#include <algorithm>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <System.h>
#include <ImuTypes.h>
#include <Optimizer.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "zed_syncer.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "slam_offline");

  std::mutex mtx_state;
  std::condition_variable cv_state;

  cv_bridge::CvImageConstPtr left_img, right_img;
  std::vector<ORB_SLAM3::IMU::Point> imu_meas;
  imu_meas.reserve(10);

  ZedSyncer syncer;
  syncer.RegisterCallback([&](const sensor_msgs::ImageConstPtr& left_img_msg,
                              const sensor_msgs::ImageConstPtr& right_img_msg,
                              const std::vector<sensor_msgs::ImuConstPtr>& imu_msgs) {
    if (imu_msgs.empty()) { return; }

    std::unique_lock<std::mutex> lock(mtx_state);
    if (!imu_meas.empty()) {
      ROS_WARN("Synced packed dropped");
      return;
    }

    left_img = cv_bridge::toCvShare(left_img_msg, sensor_msgs::image_encodings::BGR8);
    right_img = cv_bridge::toCvShare(right_img_msg, sensor_msgs::image_encodings::BGR8);

    cout << "----------------------" << endl;
    for (const auto& imu_msg: imu_msgs) {
      cout << imu_msg->header.stamp.toNSec() << endl;
      imu_meas.push_back(ORB_SLAM3::IMU::Point(imu_msg->linear_acceleration.x,
                                               imu_msg->linear_acceleration.y,
                                               imu_msg->linear_acceleration.z,
                                               imu_msg->angular_velocity.x,
                                               imu_msg->angular_velocity.y,
                                               imu_msg->angular_velocity.z,
                                               imu_msg->header.stamp.toSec()));
    }
    cout << "----------------------" << endl;

    lock.unlock();
    cv_state.notify_one();
  });

  bool use_inertial;
  ros::NodeHandle n("~");
  n.getParam("inertial", use_inertial);

  // slam system
  const auto mode = use_inertial ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO;
  ORB_SLAM3::System slam("/home/alvin/slam/ORB_SLAM3/Vocabulary/ORBvoc.txt",
                         "/home/alvin/catkin_ws/src/slam_offline/configs/zed_stereo_orbslam.yaml",
                         mode, true);

  std::ofstream fout("/tmp/trajectory.txt");

  while (ros::ok()) {
    std::unique_lock<std::mutex> lock(mtx_state);
    cv_state.wait(lock, [&]{ return !imu_meas.empty(); });

    const cv::Mat pose = slam.TrackStereo(left_img->image, right_img->image,
                                          left_img->header.stamp.toSec(), imu_meas);
    imu_meas.clear();
    lock.unlock();

    if (pose.empty()) { continue; }
    const Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> cam_T_world((float*)pose.data);
    const Eigen::Vector3f cam_t_world = cam_T_world.topRightCorner<3, 1>();
    const Eigen::Matrix3f cam_R_world = cam_T_world.topLeftCorner<3, 3>();
    const Eigen::Matrix3f world_R_cam = cam_R_world.transpose();
    const Eigen::Vector3f world_t_cam = -world_R_cam * cam_t_world;
    fout << left_img->header.stamp.toNSec() << ' ';
    fout << Eigen::Quaternionf(world_R_cam).coeffs().transpose() << ' ';
    fout << world_t_cam.transpose() << std::endl;
  }

  fout.close();

  return 0;
}

