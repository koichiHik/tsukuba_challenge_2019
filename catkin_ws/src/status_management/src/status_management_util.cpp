
#include "status_management_util.h"

// ROS
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

// Autoware
#include <autoware_msgs/Waypoint.h>

namespace koichi_robotics_lib {

XYZRPY CreateXYZRPYFromPose(const geometry_msgs::Pose &pose) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose.orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  XYZRPY xyzrpy;
  xyzrpy.x_ = pose.position.x;
  xyzrpy.y_ = pose.position.y;
  xyzrpy.z_ = pose.position.z;
  xyzrpy.roll_ = roll;
  xyzrpy.pitch_ = pitch;
  xyzrpy.yaw_ = yaw;
  return xyzrpy;
}

geometry_msgs::Pose CreatePoseFromXYZRPY(const XYZRPY &xyzrpy) {
  geometry_msgs::Pose pose;
  pose.position.x = xyzrpy.x_;
  pose.position.y = xyzrpy.y_;
  pose.position.z = xyzrpy.z_;

  tf::Quaternion quat;
  quat.setEulerZYX(xyzrpy.yaw_, xyzrpy.pitch_, xyzrpy.roll_);
  tf::quaternionTFToMsg(quat, pose.orientation);

  return pose;
}

geometry_msgs::PoseWithCovarianceStamped CreatePoseWithCovarianceStamped(
    const XYZRPY &xyzrpy, const ros::Time &stamp) {
  geometry_msgs::PoseWithCovarianceStamped p_cov;

  //
  p_cov.header.frame_id = "map";
  p_cov.header.stamp = stamp;
  p_cov.pose.pose.position.x = xyzrpy.x_;
  p_cov.pose.pose.position.y = xyzrpy.y_;
  p_cov.pose.pose.position.z = xyzrpy.z_;

  // tf::Quaternion quat(xyzrpy.yaw_, xyzrpy.pitch_, xyzrpy.roll_);
  tf::Quaternion quat;
  quat.setEulerZYX(xyzrpy.yaw_, xyzrpy.pitch_, xyzrpy.roll_);
  tf::quaternionTFToMsg(quat, p_cov.pose.pose.orientation);

  return p_cov;
}

tf::Transform CreateTransformFromPose(const geometry_msgs::Pose &to_pose,
                                      const geometry_msgs::Pose &from_pose) {
  tf::Quaternion to_q, from_q;
  tf::quaternionMsgToTF(to_pose.orientation, to_q);
  tf::quaternionMsgToTF(from_pose.orientation, from_q);

  tf::Transform to_trans, from_trans;
  to_trans.setOrigin(
      tf::Vector3(to_pose.position.x, to_pose.position.y, to_pose.position.z));
  to_trans.setRotation(to_q);
  from_trans.setOrigin(tf::Vector3(from_pose.position.x, from_pose.position.y,
                                   from_pose.position.z));
  from_trans.setRotation(from_q);

  return to_trans * from_trans.inverse();
}

double ComputeDistanceToObstacleOnWaypoint(const int32_t obx_wp_idx,
                                           const autoware_msgs::Lane &lane,
                                           const geometry_msgs::Pose &pose) {
  autoware_msgs::Waypoint wp = lane.waypoints[obx_wp_idx];

  Eigen::Vector3d diff;
  diff << pose.position.x - wp.pose.pose.position.x,
      pose.position.y - wp.pose.pose.position.y,
      pose.position.z - wp.pose.pose.position.z;
  return diff.norm();
}

bool IsAvoidanceOkWaypoint(const int32_t wp_idx,
                           const autoware_msgs::Lane &lane) {
  return lane.waypoints[wp_idx].wpstate.event_state != 0;
}

bool IsShortWaitAvoidanceWaypoint(const int32_t wp_idx,
                           const autoware_msgs::Lane &lane) {
  return lane.waypoints[wp_idx].wpstate.event_state == 1;
}

bool IsLongWaitAvoidanceWaypoint(const int32_t wp_idx,
                           const autoware_msgs::Lane &lane) {
  return lane.waypoints[wp_idx].wpstate.event_state == 2;
}

geometry_msgs::Pose TransformPose(const tf::Transform &trans,
                                  const geometry_msgs::Pose &pose) {
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);
  tf::Pose tf_tmp = trans * tf_pose;

  geometry_msgs::Pose tmp_pose;
  tf::poseTFToMsg(tf_tmp, tmp_pose);
  return tmp_pose;
}

}  // namespace koichi_robotics_lib