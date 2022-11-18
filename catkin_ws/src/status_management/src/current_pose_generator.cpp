
#include <status_management/current_pose_generator.h>

// Original
#include <status_management/status_management_helper.h>
#include <status_management/status_management_util.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

// Google
#include <glog/logging.h>

namespace koichi_robotics_lib {

template <typename T>
using eigen_alloc_vec = std::vector<T, Eigen::aligned_allocator<T>>;

double ComputeAngleBetweenQuaternion(const tf::Quaternion &tf_q1,
                                     const tf::Quaternion &tf_q2) {
  tf::Quaternion tf_q1_to_q2 = tf_q2 * tf_q1.inverse();
  return std::abs(tf_q1_to_q2.getAngleShortestPath());
}

// double ComputeAngleBetweenQuaternion(const geometry_msgs::Quaternion &q1,
//                                      const geometry_msgs::Quaternion &q2) {
//   tf::Quaternion tf_q1(q1.x, q1.y, q1.z, q1.w);
//   tf::Quaternion tf_q2(q2.x, q2.y, q2.z, q2.w);
//   return ComputeAngleBetweenQuaternion(tf_q1, tf_q2);
// }

double ComputeNorm(const tf::Vector3 &original, const tf::Vector3 &updated) {
  double diff = std::pow(updated.x() - original.x(), 2.0) +
                std::pow(updated.y() - original.y(), 2.0) +
                std::pow(updated.z() - original.z(), 2.0);
  return std::sqrt(diff);
}

double ComputeNorm(const geometry_msgs::Point &original,
                   const geometry_msgs::Point &updated) {
  double diff = std::pow(updated.x - original.x, 2.0) +
                std::pow(updated.y - original.y, 2.0) +
                std::pow(updated.z - original.z, 2.0);
  return std::sqrt(diff);
}

void ComputeTransformDifference(const geometry_msgs::Pose &cur_pose,
                                const tf::Transform &tf1,
                                const tf::Transform &tf2,
                                double &translation_diff,
                                double &rotation_diff) {
  geometry_msgs::Pose pose1 = TransformPose(tf1, cur_pose);
  geometry_msgs::Pose pose2 = TransformPose(tf2, cur_pose);

  translation_diff = ComputeNorm(pose1.position, pose2.position);
  rotation_diff =
      ComputeAngleBetweenQuaternion(pose1.orientation, pose2.orientation);
}

Eigen::Vector3d AverageTranslations(
    const eigen_alloc_vec<Eigen::Vector3d> &translations) {
  CHECK(0 < translations.size());
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (int idx = 0; idx < translations.size(); idx++) {
    sum = sum + translations.at(idx);
  }
  return sum / translations.size();
}

Eigen::Matrix3d AverageRotations(const eigen_alloc_vec<Eigen::Matrix3d> &Rs) {
  Eigen::Matrix3d R_sum = Eigen::Matrix3d::Zero();
  for (int idx = 0; idx < Rs.size(); idx++) {
    R_sum = R_sum + Rs.at(idx);
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> jsvd(
      R_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R_filt = jsvd.matrixU() * jsvd.matrixV().transpose();

  return R_filt;
}

tf::Transform ComputeAveragedTransform(
    const boost::circular_buffer<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>
        &tf_buff) {
  eigen_alloc_vec<Eigen::Vector3d> Ts;
  eigen_alloc_vec<Eigen::Matrix3d> Rs;
  for (int idx = 0; idx < tf_buff.size(); idx++) {
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &pair = tf_buff.at(idx);
    Ts.push_back(pair.first);
    Rs.push_back(pair.second);
  }
  Eigen::Vector3d avg_trans = AverageTranslations(Ts);
  Eigen::Quaterniond avg_qd(AverageRotations(Rs));

  tf::Quaternion tf_q(avg_qd.x(), avg_qd.y(), avg_qd.z(), avg_qd.w());
  tf::Vector3 tf_v(avg_trans(0), avg_trans(1), avg_trans(2));

  return tf::Transform(tf_q, tf_v);
}

/**
 * Implementation of CurrentPoseGenerator
 */
CurrentPoseGenerator::CurrentPoseGenerator(const Params &params,
                                           ros::Publisher &ndt_config_pub,
                                           ros::Publisher &ndt_init_pose_pub)
    : params_(params),
      update_enable_(false),
      odom_update_cnt_(0),
      ndt_update_cnt_(0),
      tf_odom_to_world_{},
      cur_odom_{},
      tf_buff_(params_.tf_buff_size_),
      ndt_config_pub_(ndt_config_pub),
      ndt_init_pose_pub_(ndt_init_pose_pub),
      mtx_{} {}

void CurrentPoseGenerator::InitializePose(XYZRPY &init_pose) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  ndt_config_pub_.publish(CONFIG_DEFAULT_NDT(init_pose));
  update_enable_ = true;
}

bool CurrentPoseGenerator::ResetBuffer() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  update_enable_ = false;
  odom_update_cnt_ = 0;
  ndt_update_cnt_ = 0;
  tf_odom_to_world_ = tf::Transform();
  cur_odom_ = geometry_msgs::Pose();
  tf_buff_.clear();
  return true;
}

void CurrentPoseGenerator::UpdateNdtPose(const geometry_msgs::Pose &ndt_pose) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  // X.
  if (!update_enable_) {
    return;
  }

  // X. Increment counter.
  ndt_update_cnt_ = ndt_update_cnt_ + 1;

  // X. Dump first n count of ndt.
  if (odom_update_cnt_ <= params_.first_dump_msg_count_ ||
      ndt_update_cnt_ < params_.first_dump_msg_count_) {
    return;
  }

  // X. Update tf buffer.
  UpdateTfBuffer(
      ndt_pose, cur_odom_, tf_odom_to_world_, params_.minimum_valid_buff_count_,
      params_.max_translation_diff_, params_.max_rotation_diff_in_deg_,
      tf_buff_, ndt_init_pose_pub_);

  // X. Compute averaged transform.
  tf_odom_to_world_ = ComputeAveragedTransform(tf_buff_);
}

void CurrentPoseGenerator::UpdateOdomPose(const nav_msgs::Odometry &odom) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  // X.
  if (!update_enable_) {
    return;
  }

  // X. Increment counter.
  odom_update_cnt_ = odom_update_cnt_ + 1;
  cur_odom_ = odom.pose.pose;
}

bool CurrentPoseGenerator::GetCurrentPoseAndTransform(
    geometry_msgs::Pose &pose, tf::Transform &transform) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  pose = TransformPose(tf_odom_to_world_, cur_odom_);
  transform = tf_odom_to_world_;
  return (update_enable_ &&
          params_.minimum_valid_buff_count_ <= tf_buff_.size());
}

bool CurrentPoseGenerator::UpdateTfBuffer(
    const geometry_msgs::Pose &ndt_pose, const geometry_msgs::Pose &cur_odom,
    const tf::Transform &last_tf_odom_to_world,
    const int minmum_valid_buff_count,
    const double max_allowed_translation_diff,
    const double max_allowed_rotation_diff_in_deg,
    boost::circular_buffer<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>
        &tf_buff,
    ros::Publisher &ndt_init_pub) {
  tf::Transform tf_odom_to_ndt = CreateTransformFromPose(ndt_pose, cur_odom);

  // X. If transform differs too much, use last one.
  {
    double translation_diff, rotation_diff;
    ComputeTransformDifference(cur_odom, last_tf_odom_to_world, tf_odom_to_ndt,
                               translation_diff, rotation_diff);

    if (minmum_valid_buff_count <= tf_buff.size() &&
        !(translation_diff < max_allowed_translation_diff &&
          rotation_diff < max_allowed_rotation_diff_in_deg / 180.0 * M_PI)) {
      LOG(INFO) << "Transform invalid. Norm : " << translation_diff
                << ", angle : " << rotation_diff / M_PI * 180.0;
      tf_odom_to_ndt = last_tf_odom_to_world;

      geometry_msgs::PoseWithCovarianceStamped init_pose;
      init_pose.header.frame_id = "map";
      init_pose.header.stamp = ros::Time::now();
      init_pose.pose.pose = TransformPose(tf_odom_to_ndt, cur_odom);
      ndt_init_pub.publish(init_pose);
    }
  }

  // X. Create translation.
  const tf::Vector3 &origin = tf_odom_to_ndt.getOrigin();
  Eigen::Vector3d trans(origin.x(), origin.y(), origin.z());

  // X. Create rotation.
  tf::Quaternion q;
  tf_odom_to_ndt.getBasis().getRotation(q);
  Eigen::Quaterniond qd(q.w(), q.x(), q.y(), q.z());

  tf_buff.push_back(std::make_pair(trans, qd.toRotationMatrix()));

  return true;
}

}  // namespace koichi_robotics_lib