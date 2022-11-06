
#ifndef __STATUS_MANAGEMENT_UTIL_H__
#define __STATUS_MANAGEMENT_UTIL_H__

// Autoware
#include <autoware_config_msgs/ConfigNDT.h>
#include <autoware_config_msgs/ConfigVoxelGridFilter.h>
#include <autoware_msgs/Lane.h>

// Eigen
#include <Eigen/Core>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// Original
#include <messages/initialize_pose.h>

namespace koichi_robotics_lib {

struct XYZRPY {
 public:
  XYZRPY() : x_(0.0), y_(0.0), z_(0.0), roll_(0.0), pitch_(0.0), yaw_(0.0) {}
  XYZRPY(double x, double y, double z, double roll, double pitch, double yaw)
      : x_(x), y_(y), z_(z), roll_(roll), pitch_(pitch), yaw_(yaw) {}

  double x_, y_, z_;
  double roll_, pitch_, yaw_;
};

XYZRPY CreateXYZRPYFromPose(const geometry_msgs::Pose &pose);

geometry_msgs::Pose CreatePoseFromXYZRPY(const XYZRPY &xyzrpy);

geometry_msgs::PoseWithCovarianceStamped CreatePoseWithCovarianceStamped(
    const XYZRPY &xyzrpy, const ros::Time &stamp);

tf::Transform CreateTransformFromPose(const geometry_msgs::Pose &to_pose,
                                      const geometry_msgs::Pose &from_pose);

inline autoware_config_msgs::ConfigVoxelGridFilter
CONFIG_VOXELFILT_POSE_SEARCH() {
  autoware_config_msgs::ConfigVoxelGridFilter config;
  config.measurement_range = 30.0;
  config.voxel_leaf_size = 2.0;
  return config;
}

inline autoware_config_msgs::ConfigVoxelGridFilter CONFIG_VOXELFILT_POSE_RUN() {
  autoware_config_msgs::ConfigVoxelGridFilter config;
  config.measurement_range = 50.0;
  config.voxel_leaf_size = 0.5;
  return config;
}

inline autoware_config_msgs::ConfigNDT CONFIG_DEFAULT_NDT(
    const XYZRPY &init_pose) {
  autoware_config_msgs::ConfigNDT config;

  // X. Pose
  config.x = init_pose.x_;
  config.y = init_pose.y_;
  config.z = init_pose.z_;
  config.roll = init_pose.roll_;
  config.pitch = init_pose.pitch_;
  config.yaw = init_pose.yaw_;

  // X.
  config.init_pos_gnss = 0;
  config.use_predict_pose = 0;
  config.error_threshold = 0.0;
  // config.resolution = 1.0;
  config.resolution = 0.1;
  config.step_size = 0.1;
  config.trans_epsilon = 0.01;
  config.max_iterations = 30;

  return config;
}

inline messages::initialize_pose POSE_INIT_REQUEST_FULL(const XYZRPY &xyzrpy) {
  messages::initialize_pose req;

  // xyzrpy
  req.request.x = xyzrpy.x_;
  req.request.y = xyzrpy.y_;
  req.request.z = xyzrpy.z_;
  req.request.roll = xyzrpy.roll_;
  req.request.pitch = xyzrpy.pitch_;
  req.request.yaw = xyzrpy.yaw_;

  // config.
  // req.request.resolution = 1.0;
  req.request.resolution = 0.1;
  req.request.step_size = 0.1;
  req.request.outlier_ratio = 0.55;
  req.request.trans_eps = 0.01;
  req.request.max_itr = 100;

  // Range.
  req.request.x_range = 10;
  req.request.y_range = 10;
  req.request.yaw_range = 2.0 * M_PI;

  // Step
  req.request.x_step = 1.0;
  req.request.y_step = 1.0;
  req.request.yaw_step = 20.0 / 180.0 * M_PI;

  return req;
}

inline geometry_msgs::Twist CREATE_ZERO_TWIST() {
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  return twist;
}

inline bool TwistIsBelowThreshold(const geometry_msgs::Twist &twist,
                                  const double vx_thr, const double wx_thr) {
  Eigen::Vector3d vx;
  vx << twist.linear.x, twist.linear.y, twist.linear.z;

  Eigen::Vector3d wx;
  wx << twist.angular.x, twist.angular.y, twist.angular.z;

  return vx.norm() <= vx_thr && wx.norm() <= wx_thr;
}

inline double NormalizeAnglePi2Pi(const double angle) {
  double tmp_angle = angle;
  while (true) {
    if (tmp_angle < -M_PI) {
      tmp_angle = tmp_angle + M_PI;
    } else if (M_PI <= tmp_angle) {
      tmp_angle = tmp_angle - M_PI;
    } else {
      break;
    }
  }
  return tmp_angle;
}

inline double ComputeAngleBetweenQuaternion(
    const geometry_msgs::Quaternion &q1, const geometry_msgs::Quaternion &q2) {
  tf::Quaternion tf_q1, tf_q2, tf_q1_to_q2;
  tf::quaternionMsgToTF(q1, tf_q1);
  tf::quaternionMsgToTF(q2, tf_q2);

  tf_q1_to_q2 = tf_q2 * tf_q1.inverse();

  return std::abs(tf_q1_to_q2.getAngleShortestPath());
}

inline void ComputePoseDiff(const geometry_msgs::Pose &pose1,
                            const geometry_msgs::Pose &pose2,
                            double &trans_diff_norm, double &angle_diff_norm) {
  Eigen::Vector3d translation_diff;
  translation_diff << pose1.position.x - pose2.position.x,
      pose1.position.y - pose2.position.y, pose1.position.z - pose2.position.z;

  tf::Quaternion q1, q2;
  tf::quaternionMsgToTF(pose1.orientation, q1);
  tf::quaternionMsgToTF(pose2.orientation, q2);

  trans_diff_norm = translation_diff.norm();
  angle_diff_norm = std::abs(
      ComputeAngleBetweenQuaternion(pose1.orientation, pose2.orientation));
}

inline bool PoseDiffIsBelowThreshold(const geometry_msgs::Pose &pose1,
                                     const geometry_msgs::Pose &pose2,
                                     const double trans_thr,
                                     const double orientation_thr) {
  double trans_diff_norm, angle_diff_norm;
  ComputePoseDiff(pose1, pose2, trans_diff_norm, angle_diff_norm);
  return trans_diff_norm <= trans_thr && angle_diff_norm <= orientation_thr;
}

double ComputeDistanceToObstacleOnWaypoint(const int32_t obx_wp_idx,
                                           const autoware_msgs::Lane &lane,
                                           const geometry_msgs::Pose &pose);

bool IsAvoidanceOkWaypoint(const int32_t wp_idx,
                           const autoware_msgs::Lane &lane);

bool IsShortWaitAvoidanceWaypoint(const int32_t wp_idx,
                                  const autoware_msgs::Lane &lane);

bool IsLongWaitAvoidanceWaypoint(const int32_t wp_idx,
                                 const autoware_msgs::Lane &lane);

geometry_msgs::Pose TransformPose(const tf::Transform &trans,
                                  const geometry_msgs::Pose &pose);

}  // namespace koichi_robotics_lib

#endif