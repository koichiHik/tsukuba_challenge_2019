#ifndef __STATUS_MANAGEMENT_CURRENT_POSE_GENERATOR_H__
#define __STATUS_MANAGEMENT_CURRENT_POSE_GENERATOR_H__

// Original
#include <status_management/status_management_helper.h>

// Message
#include <nav_msgs/Odometry.h>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

// Boost
#include <boost/circular_buffer.hpp>

// STL
#include <mutex>

namespace koichi_robotics_lib {

struct CurrentPoseGenerator {
 public:
  CurrentPoseGenerator();
  bool ResetBuffer();
  void UpdateNdtPose(const geometry_msgs::Pose &ndt_pose);
  void UpdateOdomPose(const nav_msgs::Odometry &odom);
  bool IsCurrentPoseAvailable();
  geometry_msgs::Pose GetCurrentPose();

  static bool UpdateTfBuffer(
      const geometry_msgs::Pose &ndt_pose, const geometry_msgs::Pose &cur_odom,
      const tf::Transform &last_tf_odom_to_world,
      boost::circular_buffer<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>
          &tf_buff);

 private:
  unsigned int odom_update_cnt_;
  unsigned int ndt_update_cnt_;
  tf::Transform tf_odom_to_world_;
  geometry_msgs::Pose cur_odom_;
  boost::circular_buffer<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> tf_buff_;

  std::mutex mtx_;

 private:
  static constexpr int TF_BUFFER_SIZE = 30;
  static constexpr int DUMP_FIRST_MSG = 5;
  static constexpr int MINIMUM_BUFF_COUNT = 5;
};

}  // namespace koichi_robotics_lib

#endif