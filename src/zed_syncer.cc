#include "zed_syncer.h"

static int img_cnt = 0;

ZedSyncer::ZedSyncer()
    : n_(),
      sub_imu_(n_.subscribe("/zed2/zed_node/imu/data", 100, &ZedSyncer::HandleIMU, this)),
      sub_img_left_(n_, "/zed2/zed_node/left/image_rect_color", 10),
      sub_img_right_(n_, "/zed2/zed_node/right/image_rect_color", 10),
      sync_stereo_(sub_img_left_, sub_img_right_, 10),
      spinner_(1) {
  sync_stereo_.registerCallback(&ZedSyncer::HandleStereo, this);
  spinner_.start();
}

ZedSyncer::~ZedSyncer() {
  spinner_.stop();
}

void ZedSyncer::RegisterCallback(const CallbackType& callback) { callback_ = callback; }

void ZedSyncer::HandleIMU(const sensor_msgs::ImuConstPtr& msg) {
  imu_q_.push_back(msg);
  TryInvokeSync();
}

void ZedSyncer::HandleStereo(const sensor_msgs::ImageConstPtr& left_msg,
                             const sensor_msgs::ImageConstPtr& right_msg) {
  if ((img_cnt++) % 3 == 0) {
    stereo_q_.push(std::make_pair(left_msg, right_msg));
    TryInvokeSync();
  }
}

void ZedSyncer::TryInvokeSync() {
  if (imu_q_.empty() || stereo_q_.empty()) { return; }

  if (stereo_q_.front().first->header.stamp < imu_q_.back()->header.stamp) {
    const auto stereo_msg = stereo_q_.front();
    stereo_q_.pop();

    std::vector<sensor_msgs::ImuConstPtr> imu_msgs;
    imu_msgs.reserve(10);

    const auto it = std::upper_bound(imu_q_.begin(), imu_q_.end(), stereo_msg.first,
        [](const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImuConstPtr& imu_msg) {
      return img_msg->header.stamp < imu_msg->header.stamp;
    });

    std::move(imu_q_.begin(), it, std::back_inserter(imu_msgs));
    imu_q_.erase(imu_q_.begin(), it);

    HandleSyncedSequence(stereo_msg.first, stereo_msg.second, imu_msgs);
  }
}

void ZedSyncer::HandleSyncedSequence(const sensor_msgs::ImageConstPtr& left_img_msg,
                                     const sensor_msgs::ImageConstPtr& right_img_msg,
                                     const std::vector<sensor_msgs::ImuConstPtr>& imu_msgs) {
  if (callback_) {
    callback_(left_img_msg, right_img_msg, imu_msgs);
  }
}
