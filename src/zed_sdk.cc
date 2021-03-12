#include <algorithm>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "zed_syncer.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "slam_offline");

  // slam system
  std::ofstream fout("/tmp/trajectory.txt");

  ros::NodeHandle node;
  auto sub = node.subscribe<geometry_msgs::PoseStamped>("/zed2/zed_node/pose", 10,
      [&](const geometry_msgs::PoseStampedConstPtr& msg) {
    const Eigen::Quaterniond world_R_cam(msg->pose.orientation.w,
                                         msg->pose.orientation.x,
                                         msg->pose.orientation.y,
                                         msg->pose.orientation.z);
    const Eigen::Vector3d world_t_cam(msg->pose.position.x,
                                      msg->pose.position.y,
                                      msg->pose.position.z);
    fout << msg->header.stamp.toNSec() << ' ';
    fout << Eigen::Quaternionf(world_R_cam).coeffs().transpose() << ' ';
    fout << world_t_cam.transpose() << std::endl;
  });

  ros::spin();

  return 0;
}


