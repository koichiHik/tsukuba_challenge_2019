
#include <status_management/current_pose_generator.h>

// Original
#include <status_management/status_management_util.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

// Google
#include <glog/logging.h>

namespace koichi_robotics_lib {

template <typename T>
using eigen_alloc_vec = std::vector<T, Eigen::aligned_allocator<T>>;

CurrentPoseGenerator::CurrentPoseGenerator() : tf_buff_(TF_BUFFER_SIZE) {}

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
 * Implementation of SyncState
 */
bool CurrentPoseGenerator::ResetBuffer() {
  std::lock_guard<std::mutex> lock(mtx_);
  tf_buff_.clear();
  odom_update_cnt_ = 0;
  ndt_update_cnt_ = 0;
  return true;
}

void CurrentPoseGenerator::UpdateNdtPose(const geometry_msgs::Pose &ndt_pose) {
  std::lock_guard<std::mutex> lock(mtx_);

  // X. Increment counter.
  ndt_update_cnt_ = ndt_update_cnt_ + 1;

  // X. Dump first n count of ndt.
  if (odom_update_cnt_ <= DUMP_FIRST_MSG || ndt_update_cnt_ < DUMP_FIRST_MSG) {
    return;
  }

  // X. Update tf buffer.
  UpdateTfBuffer(ndt_pose, cur_odom_, tf_odom_to_world_, tf_buff_);

  // X. Compute averaged transform.
  tf_odom_to_world_ = ComputeAveragedTransform(tf_buff_);
}

void CurrentPoseGenerator::UpdateOdomPose(const nav_msgs::Odometry &odom) {
  std::lock_guard<std::mutex> lock(mtx_);

  // X. Increment counter.
  odom_update_cnt_ = odom_update_cnt_ + 1;
  if (odom_update_cnt_ < DUMP_FIRST_MSG) {
    return;
  }

  cur_odom_ = odom.pose.pose;
}

bool CurrentPoseGenerator::IsCurrentPoseAvailable() {
  std::lock_guard<std::mutex> lock(mtx_);
  return MINIMUM_BUFF_COUNT <= tf_buff_.size();
}

geometry_msgs::Pose CurrentPoseGenerator::GetCurrentPose() {
  std::lock_guard<std::mutex> lock(mtx_);
  return TransformPose(tf_odom_to_world_, cur_odom_);
}

bool CurrentPoseGenerator::UpdateTfBuffer(
    const geometry_msgs::Pose &ndt_pose, const geometry_msgs::Pose &cur_odom,
    const tf::Transform &last_tf_odom_to_world,
    boost::circular_buffer<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>
        &tf_buff) {
  tf::Transform tf_odom_to_ndt = CreateTransformFromPose(ndt_pose, cur_odom);

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