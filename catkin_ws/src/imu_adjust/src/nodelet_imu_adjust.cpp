
#include <imu_adjust/nodelet_imu_adjust.h>

// ROS
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <thread>

// Glog
#include <glog/logging.h>

namespace imu_adjust {

struct ImuAdjustParams {
 public:
  double offset_x, offset_y, offset_z;
  double offset_rx, offset_ry, offset_rz;
};

void ReadParams(ros::NodeHandle &pnh, ImuAdjustParams &params) {
  CHECK(pnh.param("offset_x", params.offset_x, 0.0))
      << "Paramter offset_x cannot be read.";
  CHECK(pnh.param("offset_y", params.offset_y, 0.0))
      << "Paramter offset_y cannot be read.";
  CHECK(pnh.param("offset_z", params.offset_z, 0.0))
      << "Paramter offset_z cannot be read.";

  CHECK(pnh.param("offset_rx", params.offset_rx, 0.0))
      << "Paramter offset_rx cannot be read.";
  CHECK(pnh.param("offset_ry", params.offset_ry, 0.0))
      << "Paramter offset_ry cannot be read.";
  CHECK(pnh.param("offset_rz", params.offset_rz, 0.0))
      << "Paramter offset_rz cannot be read.";
};

struct NodeletImuAdjustImpl {
 public:
  NodeletImuAdjustImpl(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~NodeletImuAdjustImpl();

  void Initialize();

 private:
  void PreparePubSub();

  void ImuCallback(const sensor_msgs::ImuConstPtr &msg);

 private:
  // X. Ros
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_imu_;
  ros::Subscriber sub_imu_raw_;

  // X. Params
  ImuAdjustParams params_;

  // X. Pose status.
  ros::Time last_time_stamp;
  Eigen::Matrix3d cur_R;

  std::thread st_thread_;

  // X. Const
  static const uint32_t DEFAULT_QUEUE_SIZE = 100;
};

/**
 * Implementation of NodeletImuAdjustImpl
 */
NodeletImuAdjustImpl::NodeletImuAdjustImpl(ros::NodeHandle &nh,
                                           ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      pub_imu_(),
      sub_imu_raw_(),
      last_time_stamp(),
      cur_R(),
      st_thread_() {
  cur_R = Eigen::Matrix3d::Identity();
  st_thread_ = std::thread(&NodeletImuAdjustImpl::Initialize, this);
}

NodeletImuAdjustImpl::~NodeletImuAdjustImpl() {
  // X. Join thread.
  st_thread_.join();
}

void NodeletImuAdjustImpl::Initialize() {
  // X. Read parameters.
  ReadParams(pnh_, params_);

  // X. Preapare pubsub.
  PreparePubSub();
}

void NodeletImuAdjustImpl::PreparePubSub() {
  sub_imu_raw_ = pnh_.subscribe("imu_raw", DEFAULT_QUEUE_SIZE,
                                &NodeletImuAdjustImpl::ImuCallback, this);

  pub_imu_ = pnh_.advertise<sensor_msgs::Imu>("imu", DEFAULT_QUEUE_SIZE);
}

void NodeletImuAdjustImpl::ImuCallback(const sensor_msgs::ImuConstPtr &msg) {
  sensor_msgs::Imu imu;

  // X. Header
  imu.header = msg->header;

  // X. Linear acceleration.
  imu.linear_acceleration.x = -(msg->linear_acceleration.x - params_.offset_x);
  imu.linear_acceleration.y = -(msg->linear_acceleration.y - params_.offset_y);
  imu.linear_acceleration.z = -(msg->linear_acceleration.z - params_.offset_z);
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  // X. Angular velocity.
  imu.angular_velocity.x = msg->angular_velocity.x - params_.offset_rx;
  imu.angular_velocity.y = msg->angular_velocity.y - params_.offset_ry;
  imu.angular_velocity.z = msg->angular_velocity.z - params_.offset_rz;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;

  // X. Compute orientation.
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  if (!last_time_stamp.isZero()) {
    double dt = (msg->header.stamp - last_time_stamp).toSec();
    double kx = imu.angular_velocity.x * dt;
    double ky = imu.angular_velocity.y * dt;
    double kz = imu.angular_velocity.z * dt;

    Eigen::Matrix3d K;
    K << 0.0, -kz, ky, kz, 0.0, -kx, -ky, kx, 0.0;

    Eigen::Vector3d rot_vec;
    rot_vec << kx, ky, kz;
    double norm_rot = rot_vec.norm();

    if (std::numeric_limits<double>::epsilon() < std::pow(norm_rot, 2.0)) {
      dR = Eigen::Matrix3d::Identity() + std::sin(norm_rot) * K / norm_rot +
           (1 - std::cos(norm_rot)) * K * K / std::pow(norm_rot, 2);
    } else {
      dR = Eigen::Matrix3d::Identity() + K;
    }
  }

  Eigen::Matrix3d dR_wld = cur_R * dR * cur_R.transpose();
  Eigen::Quaterniond q(dR_wld * cur_R);
  cur_R = q.normalized().toRotationMatrix();

  // X. Orientation
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();
  imu.orientation_covariance = msg->orientation_covariance;

  pub_imu_.publish(imu);

  last_time_stamp = msg->header.stamp;
}

/**
 * Implementation of NodeletImuAdjust
 */
NodeletImuAdjust::NodeletImuAdjust() : p_impl_(nullptr) {}

NodeletImuAdjust::~NodeletImuAdjust() {}

void NodeletImuAdjust::onInit() {
  // X. Initialize impl
  p_impl_.reset(
      new NodeletImuAdjustImpl(getNodeHandle(), getPrivateNodeHandle()));
}

}  // namespace imu_adjust
PLUGINLIB_EXPORT_CLASS(imu_adjust::NodeletImuAdjust, nodelet::Nodelet);