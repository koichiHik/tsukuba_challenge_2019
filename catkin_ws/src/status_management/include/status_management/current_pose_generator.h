#ifndef __STATUS_MANAGEMENT_CURRENT_POSE_GENERATOR_H__
#define __STATUS_MANAGEMENT_CURRENT_POSE_GENERATOR_H__

// Original
#include <status_management/status_management_util.h>

// Message
#include <nav_msgs/Odometry.h>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

// Eigen
#include <Eigen/Core>

// Boost
#include <boost/circular_buffer.hpp>

// STL
#include <mutex>

namespace koichi_robotics_lib {

struct CurrentPoseGenerator {
 public:
  struct Params {
    int tf_buff_size_;
    int first_dump_msg_count_;
    int minimum_valid_buff_count_;
    double max_translation_diff_;
    double max_rotation_diff_in_deg_;
  };

 public:
  CurrentPoseGenerator(const Params &params, ros::Publisher &ndt_config_pub,
                       ros::Publisher &ndt_init_pose_pub);
  void InitializePose(XYZRPY &init_pose);
  bool ResetBuffer();
  void UpdateNdtPose(const geometry_msgs::Pose &ndt_pose);
  void UpdateOdomPose(const nav_msgs::Odometry &odom);
  bool GetCurrentPoseAndTransform(geometry_msgs::Pose &pose,
                                  tf::Transform &transform);

  static bool UpdateTfBuffer(
      const geometry_msgs::Pose &ndt_pose, const geometry_msgs::Pose &cur_odom,
      const tf::Transform &last_tf_odom_to_world,
      const double max_allowed_translation_diff,
      const double max_allowed_rotation_diff_in_deg,
      boost::circular_buffer<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>
          &tf_buff,
      ros::Publisher &ndt_init_pub);

 private:
  Params params_;
  bool update_enable_;
  unsigned int odom_update_cnt_;
  unsigned int ndt_update_cnt_;
  tf::Transform tf_odom_to_world_;
  geometry_msgs::Pose cur_odom_;
  boost::circular_buffer<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> tf_buff_;
  ros::Publisher &ndt_config_pub_, &ndt_init_pose_pub_;

  std::mutex mtx_;
};

}  // namespace koichi_robotics_lib

#endif