
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <sys/time.h>
#include <cmath>
#include <cstddef>
#include <mutex>

namespace {

struct JointInfo {
 public:
  JointInfo()
      : timestamp_(),
        r_rad_(0.0),
        l_rad_(0.0),
        r_vel_(0.0),
        l_vel_(0.0),
        T_3d_(Eigen::Vector3d::Zero()),
        v_3d_(Eigen::Vector3d::Zero()),
        T_2d_(Eigen::Vector3d::Zero()),
        v_2d_(Eigen::Vector3d::Zero()) {}

  ros::Time timestamp_;
  double r_rad_, l_rad_;
  double r_vel_, l_vel_;
  Eigen::Vector3d T_3d_;
  Eigen::Vector3d v_3d_;
  Eigen::Vector3d T_2d_;
  Eigen::Vector3d v_2d_;
};

struct ImuInfo {
 public:
  ImuInfo()
      : timestamp_(),
        R_3d_(Eigen::Matrix3d::Identity()),
        w_3d_(Eigen::Vector3d::Zero()),
        R_2d_(Eigen::Matrix3d::Identity()),
        w_2d_(Eigen::Vector3d::Zero()) {}

  ros::Time timestamp_;
  Eigen::Matrix3d R_3d_;
  Eigen::Vector3d w_3d_;
  Eigen::Matrix3d R_2d_;
  Eigen::Vector3d w_2d_;
};

struct LockedBuffer {
 public:
  void SetJointInfo(const JointInfo &joint_info) {
    std::lock_guard<std::mutex> lock(joint_mtx_);
    joint_info_ = joint_info;
  }

  JointInfo GetJointInfo() {
    std::lock_guard<std::mutex> lock(joint_mtx_);
    return joint_info_;
  }

  void SetImuInfo(const ImuInfo &imu_info) {
    std::lock_guard<std::mutex> lock(imu_mtx_);
    imu_info_ = imu_info;
  }

  ImuInfo GetImuInfo() {
    std::lock_guard<std::mutex> lock(imu_mtx_);
    return imu_info_;
  }

 private:
  JointInfo joint_info_;
  ImuInfo imu_info_;

  std::mutex joint_mtx_;
  std::mutex imu_mtx_;
};

void ParseJointStateMsg(const sensor_msgs::JointStateConstPtr &jointPos,
                        double &r_rad, double &l_rad, double &r_vel,
                        double &l_vel) {
  for (int i = 0; i < jointPos->name.size(); i++) {
    std::string name = jointPos->name[i];
    double jntPos = jointPos->position[i];
    double jntVel = jointPos->velocity[i];

    if (name == "right") {
      r_rad = -jntPos;
      r_vel = -jntVel;
    } else if (name == "left") {
      l_rad = jntPos;
      l_vel = jntVel;
    } else {
      ROS_ERROR("Unexpected Joint Name : %s", name.c_str());
    }
  }
}

void ComputeIncrementalDistance(const double distance_per_radian_right,
                                const double distance_per_radian_left,
                                const JointInfo &last_joint_info,
                                const JointInfo &joint_info,
                                double &increment_avg, double &increment_right,
                                double &increment_left) {
  increment_right =
      (joint_info.r_rad_ - last_joint_info.r_rad_) * distance_per_radian_right;
  increment_left =
      (joint_info.l_rad_ - last_joint_info.l_rad_) * distance_per_radian_left;
  increment_avg = (increment_right + increment_left) / 2.0;
}

Eigen::Matrix3d UpdateOrientation(const Eigen::Matrix3d &cur_R,
                                  const sensor_msgs::Imu &imu,
                                  const double dt) {
  double kx = imu.angular_velocity.x * dt;
  double ky = imu.angular_velocity.y * dt;
  double kz = imu.angular_velocity.z * dt;

  Eigen::Matrix3d K;
  K << 0.0, -kz, ky, kz, 0.0, -kx, -ky, kx, 0.0;

  Eigen::Vector3d rot_vec;
  rot_vec << kx, ky, kz;
  double norm_rot = rot_vec.norm();

  // X. Compute infinitesimal rotation.
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  if (std::numeric_limits<double>::epsilon() < std::pow(norm_rot, 2.0)) {
    dR = Eigen::Matrix3d::Identity() + std::sin(norm_rot) * K / norm_rot +
         (1 - std::cos(norm_rot)) * K * K / std::pow(norm_rot, 2);
  } else {
    dR = Eigen::Matrix3d::Identity() + K;
  }

  // X. Compute orientation relative to odometry frame.
  Eigen::Matrix3d dR_wld = cur_R * dR * cur_R.transpose();
  Eigen::Quaterniond q(dR_wld * cur_R);
  return q.normalized().toRotationMatrix();
}

nav_msgs::Odometry CreateOdomMessage(const ros::Time &stamp,
                                     const std::string &odom_frame_name,
                                     const std::string &base_frame_name,
                                     const Eigen::Vector3d &T,
                                     const Eigen::Matrix3d &R,
                                     const Eigen::Vector3d &v,
                                     const Eigen::Vector3d &w) {
  nav_msgs::Odometry odom;

  // X. Header
  odom.header.frame_id = odom_frame_name;
  odom.child_frame_id = base_frame_name;
  odom.header.stamp = stamp;

  // X. Pose Translation.
  odom.pose.pose.position.x = T(0);
  odom.pose.pose.position.y = T(1);
  odom.pose.pose.position.z = T(2);

  // X. Pose Rotation.
  Eigen::Quaterniond q(R);
  Eigen::Quaterniond qn = q.normalized();
  odom.pose.pose.orientation.x = qn.x();
  odom.pose.pose.orientation.y = qn.y();
  odom.pose.pose.orientation.z = qn.z();
  odom.pose.pose.orientation.w = qn.w();

  // X. Twist Linear
  odom.twist.twist.linear.x = v(0);
  odom.twist.twist.linear.y = v(1);
  odom.twist.twist.linear.z = v(2);

  // X. Twist Angular
  odom.twist.twist.angular.x = w(0);
  odom.twist.twist.angular.y = w(1);
  odom.twist.twist.angular.z = w(2);

  return odom;
}

struct OdometryParams {
 public:
  bool use_odom_2d;
  double right_distance_per_rad;
  double left_distance_per_rad;
  double wheel_base;

  std::string odom_frame_name;
  std::string baselink_frame_name;
};

void ReadParam(const std::string &paramName, bool result) {
  if (!result) {
    ROS_FATAL("Param read failure. : %s", paramName.c_str());
  }
}

void ReadParams(ros::NodeHandle &pnh_, OdometryParams &params) {
  ReadParam("use_odom_2d",
            pnh_.param<bool>("use_odom_2d", params.use_odom_2d, false));

  // Frame Name
  ReadParam("odometry_frame_name",
            pnh_.param<std::string>("odometry_frame_name",
                                    params.odom_frame_name, ""));
  ReadParam("base_link_frame_name",
            pnh_.param<std::string>("base_link_frame_name",
                                    params.baselink_frame_name, ""));

  // Hardware Related
  double r_whl_dia, l_whl_dia;
  ReadParam(
      "l_whl_diameter",
      pnh_.param<double>("l_whl_diameter", params.left_distance_per_rad, 0.0));
  ReadParam(
      "r_whl_diameter",
      pnh_.param<double>("r_whl_diameter", params.right_distance_per_rad, 0.0));
  ReadParam("wheel_base",
            pnh_.param<double>("wheel_base", params.wheel_base, 1.0));
}

}  // namespace

namespace koichi_robotics_lib {

class OdometryNodelet : public nodelet::Nodelet {
 public:
  OdometryNodelet();

  ~OdometryNodelet();

  virtual void onInit();

 private:
  void jointStateCallback(const sensor_msgs::JointStateConstPtr &jointPos);

  void imuCallback(const sensor_msgs::ImuConstPtr &imuMsg);

 private:
  // Parameters
  OdometryParams params_;

  ros::NodeHandle nh_, pnh_;
  nav_msgs::Odometry curOdom;

  // ROS pub sub.
  ros::Subscriber imu_sub_, joint_sub_;
  ros::Publisher odom_pub_;

  // ROS Publisher
  tf::TransformBroadcaster odom_tf_broadcaster;

  // Current state
  LockedBuffer state_;

  static const int DEFAULT_SUBSCRIBE_QUEUE_SIZE = 100;
};

using namespace std;
using namespace koichi_robotics_lib;

OdometryNodelet::OdometryNodelet() {}

OdometryNodelet::~OdometryNodelet() {}

void OdometryNodelet::onInit() {
  NODELET_DEBUG("Initializing Odometry Nodelet....");

  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();

  ReadParams(pnh_, params_);

  // X. Publisher
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);

  // X. Subscriber
  joint_sub_ = nh_.subscribe("joint_states", DEFAULT_SUBSCRIBE_QUEUE_SIZE,
                             &OdometryNodelet::jointStateCallback, this);
  imu_sub_ = nh_.subscribe("imu", DEFAULT_SUBSCRIBE_QUEUE_SIZE,
                           &OdometryNodelet::imuCallback, this);
}

void OdometryNodelet::jointStateCallback(
    const sensor_msgs::JointStateConstPtr &jointPos) {
  // X. Update joint info.
  JointInfo new_joint_info;
  {
    double r_rad, l_rad, r_vel, l_vel;
    ParseJointStateMsg(jointPos, r_rad, l_rad, r_vel, l_vel);
    new_joint_info.timestamp_ = jointPos->header.stamp;
    new_joint_info.r_rad_ = r_rad;
    new_joint_info.l_rad_ = l_rad;
    new_joint_info.r_vel_ = r_vel;
    new_joint_info.l_vel_ = l_vel;
  }

  // X. Compute incremental distance.
  JointInfo last_joint_info = state_.GetJointInfo();
  if (last_joint_info.timestamp_.isZero()) {
    last_joint_info = new_joint_info;
  }
  double increment_right, increment_left, increment_avg;
  ComputeIncrementalDistance(params_.right_distance_per_rad,
                             params_.left_distance_per_rad, last_joint_info,
                             new_joint_info, increment_avg, increment_right,
                             increment_left);
  double increment = increment_avg;

  // X. Rotate incremental vector.
  ImuInfo cur_imu_info = state_.GetImuInfo();
  Eigen::Vector3d dT_3d =
      cur_imu_info.R_3d_ * Eigen::Vector3d(increment, 0.0, 0.0);
  Eigen::Vector3d dT_2d =
      cur_imu_info.R_2d_ * Eigen::Vector3d(increment, 0.0, 0.0);

  // X. Update translation part of pose.
  new_joint_info.T_3d_ = last_joint_info.T_3d_ + dT_3d;
  new_joint_info.T_2d_ = last_joint_info.T_2d_ + dT_2d;

  // X. Compute velocity
  {
    double vx_3d = 0.0;
    double vx_2d = 0.0;
    if (last_joint_info.timestamp_ < new_joint_info.timestamp_) {
      double dt =
          (new_joint_info.timestamp_ - last_joint_info.timestamp_).toSec();
      vx_3d = increment / dt;
      vx_2d = increment / dt;
    } else {
      vx_3d = last_joint_info.v_3d_(0);
      vx_2d = last_joint_info.v_2d_(0);
    }
    new_joint_info.v_3d_(0) = vx_3d;
    new_joint_info.v_3d_(1) = 0.0;
    new_joint_info.v_3d_(2) = 0.0;

    new_joint_info.v_2d_(0) = vx_2d;
    new_joint_info.v_2d_(1) = 0.0;
    new_joint_info.v_2d_(2) = 0.0;
  }

  // X. Set
  state_.SetJointInfo(new_joint_info);
}

void OdometryNodelet::imuCallback(const sensor_msgs::ImuConstPtr &imuMsg) {
  // X. Update IMU Info
  ImuInfo new_imu_info;
  {
    new_imu_info.timestamp_ = imuMsg->header.stamp;
    new_imu_info.w_3d_(0) = imuMsg->angular_velocity.x;
    new_imu_info.w_3d_(1) = imuMsg->angular_velocity.y;
    new_imu_info.w_3d_(2) = imuMsg->angular_velocity.z;
    new_imu_info.w_2d_(0) = 0.0;
    new_imu_info.w_2d_(1) = 0.0;
    new_imu_info.w_2d_(2) = imuMsg->angular_velocity.z;

    // X. Update rotation part of pose.
    sensor_msgs::Imu imu_2d;
    imu_2d.angular_velocity.x = 0.0;
    imu_2d.angular_velocity.y = 0.0;
    imu_2d.angular_velocity.z = imuMsg->angular_velocity.z;

    // X.
    JointInfo tmp_joint_info = state_.GetJointInfo();
    ImuInfo last_imu_info = state_.GetImuInfo();
    if (!last_imu_info.timestamp_.isZero()) {
      double dt = (imuMsg->header.stamp - last_imu_info.timestamp_).toSec();
      if (0.0001 < tmp_joint_info.v_3d_.norm()) {
        new_imu_info.R_3d_ =
            UpdateOrientation(last_imu_info.R_3d_, *imuMsg, dt);
        new_imu_info.R_2d_ = UpdateOrientation(last_imu_info.R_2d_, imu_2d, dt);
      } else {
        new_imu_info.R_3d_ = last_imu_info.R_3d_;
        new_imu_info.R_2d_ = last_imu_info.R_2d_;
      }
    } else {
      new_imu_info.R_3d_ = Eigen::Matrix3d::Identity();
      new_imu_info.R_2d_ = Eigen::Matrix3d::Identity();
    }
    state_.SetImuInfo(new_imu_info);
  }

  // X. Publish
  JointInfo joint_info = state_.GetJointInfo();

  nav_msgs::Odometry odom;
  if (!params_.use_odom_2d) {
    odom = CreateOdomMessage(imuMsg->header.stamp, params_.odom_frame_name,
                             params_.baselink_frame_name, joint_info.T_3d_,
                             new_imu_info.R_3d_, joint_info.v_3d_,
                             new_imu_info.w_3d_);
  } else {
    odom = CreateOdomMessage(imuMsg->header.stamp, params_.odom_frame_name,
                             params_.baselink_frame_name, joint_info.T_2d_,
                             new_imu_info.R_2d_, joint_info.v_2d_,
                             new_imu_info.w_2d_);
  }

  tf::Transform odom_trans;
  tf::poseMsgToTF(odom.pose.pose, odom_trans);
  odom_tf_broadcaster.sendTransform(tf::StampedTransform(
      odom_trans, odom.header.stamp, params_.odom_frame_name,
      params_.baselink_frame_name));
  odom_pub_.publish(odom);
}

}  // namespace koichi_robotics_lib

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(koichi_robotics_lib::OdometryNodelet, nodelet::Nodelet)
