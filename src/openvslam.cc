#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <openvslam/config.h>
#include <openvslam/system.h>
#include <pangolin_viewer/viewer.h>
#include <yaml-cpp/yaml.h>

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

  // slam system
  const auto cfg = std::make_shared<openvslam::config>(
      "/home/alvin/catkin_ws/src/slam_offline/configs/zed_stereo_ovslam.yaml");
  openvslam::system slam(cfg, "/home/alvin/Downloads/orb_vocab/orb_vocab.dbow2");
  slam.startup();

  // pangolin viewer
  pangolin_viewer::viewer viewer(cfg, &slam, slam.get_frame_publisher(), slam.get_map_publisher());

  std::ofstream fout("/tmp/trajectory.txt");

  ZedSyncer syncer;
  syncer.RegisterCallback([&](const sensor_msgs::ImageConstPtr& left_img_msg,
                              const sensor_msgs::ImageConstPtr& right_img_msg,
                              const std::vector<sensor_msgs::ImuConstPtr>& imu_msgs) {
    const auto left_img = cv_bridge::toCvShare(left_img_msg, sensor_msgs::image_encodings::BGR8);
    const auto right_img = cv_bridge::toCvShare(right_img_msg, sensor_msgs::image_encodings::BGR8);

    const Eigen::Matrix4d cam_T_world = slam.feed_stereo_frame(left_img->image, right_img->image, left_img_msg->header.stamp.toSec());
    const Eigen::Vector3d cam_t_world = cam_T_world.topRightCorner<3, 1>();
    const Eigen::Matrix3d cam_R_world = cam_T_world.topLeftCorner<3, 3>();
    const Eigen::Matrix3d world_R_cam = cam_R_world.transpose();
    const Eigen::Vector3d world_t_cam = -world_R_cam * cam_t_world;
    fout << left_img_msg->header.stamp.toNSec() << ' ';
    fout << Eigen::Quaterniond(world_R_cam).coeffs().transpose() << ' ';
    fout << world_t_cam.transpose() << std::endl;
  });

  std::thread viewer_t([&]() {
    viewer.run();
  });

  ros::spin();
  viewer_t.join();
  slam.shutdown();

  return 0;
}
