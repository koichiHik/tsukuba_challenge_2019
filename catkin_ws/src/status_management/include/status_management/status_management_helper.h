#ifndef __STATUS_MANAGEMENT_STATUS_MANAGEMENT_HELPER_H__
#define __STATUS_MANAGEMENT_STATUS_MANAGEMENT_HELPER_H__

// Original
#include <status_management/autorun_config.h>
#include <status_management/current_pose_generator.h>
#include <status_management/status_management_util.h>

// Autoware message
#include <autoware_config_msgs/ConfigNDT.h>
#include <autoware_config_msgs/ConfigVoxelGridFilter.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/String.h>

// ROS Message
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <ypspur_ros/ControlMode.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Boost
#include <boost/circular_buffer.hpp>

// STL
#include <mutex>

namespace koichi_robotics_lib {

template <typename T>
class LockedObj {
 public:
  LockedObj() : obj_() {}
  LockedObj(const T &obj) : obj_(obj) {}

  T GetObj() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return obj_;
  };

  void SetObj(const T &obj) {
    std::lock_guard<std::mutex> lock(mtx_);
    obj_ = obj;
  }

 private:
  mutable std::mutex mtx_;
  T obj_;
};

struct StatusManagementNodeletParams {
  std::string course_config_yaml;
  bool localize_only;
  int start_course_idx;

  bool init_from_gnss;
  bool use_pose_search;
  double x, y, z, roll, pitch, yaw;

  bool reinit_via_gnss;
  double init_pose_srv_timeout;
  double status_check_freq;
  double robot_stop_check_freq;
  double standstill_vx_thr;
  double standstill_wx_thr;
  double localization_reinit_minimum_period;
  double localization_odom_trust_period;
  int localization_unreliable_cnt_for_reinit;
  int localization_reliable_count;
  int init_pose_standstill_count;
  double localization_translation_thresh;
  double localization_orientation_thresh;
  double localization_translation_reliable_thresh;
  double localization_orientation_reliable_thresh;

  double obstacle_distance_thresh;
  double obstacle_short_wait_before_avoid;
  double obstacle_long_wait_before_avoid;

  CurrentPoseGenerator::Params cur_pose_gen_params_;
};

class CourseConfigMgmt {
 public:
  CourseConfigMgmt(const std::string &course_yaml_path,
                   const unsigned int start_idx);

  autorun_config GetCurrentConfig();

  bool NextConfig(
      uint32_t min_time_from_last_call = MINIMUM_SEC_TO_UPDATE_CONFIG);

 private:
  std::map<unsigned int, autorun_config> course_configs_;
  std::map<unsigned int, autorun_config>::const_iterator cur_itr_;
  ros::Time last_update_time_;

 private:
  static const uint32_t MINIMUM_SEC_TO_UPDATE_CONFIG = 10;
};

struct SyncState {
 public:
  SyncState(int localization_reliable_count) {
    engage_request_.SetObj(false);
    autorun_request_.SetObj(false);
    {
      std_msgs::Int32 n;
      n.data = -1;
      cls_wp_idx_.SetObj(n);
      obst_wp_idx_.SetObj(n);
    }

    standstill_cnt_.SetObj(0);
    localization_unreliable_cnt_.SetObj(0);
    stopped_due_to_obstacle_cnt_.SetObj(0);
    pose_initializing_.SetObj(false);
    stop_request_.SetObj(false);
    control_cycle_cnt_.SetObj(0);
    checking_cycle_cnt_.SetObj(0);
    last_valid_ndt_pose_queue_.set_capacity(localization_reliable_count);
  }

  // X. Subscribed message.
  LockedObj<bool> engage_request_, autorun_request_;

  // Pose
  LockedObj<geometry_msgs::PoseStamped> gnss_local_;
  LockedObj<geometry_msgs::PoseStamped> ndt_pose_;

  // Odom & Control
  LockedObj<nav_msgs::Odometry> odom_;
  LockedObj<geometry_msgs::TwistStamped> twist_raw_;

  // Waypoint & Lane
  LockedObj<std_msgs::Int32> cls_wp_idx_;
  LockedObj<std_msgs::Int32> obst_wp_idx_;
  LockedObj<autoware_msgs::Lane> base_wps_;
  LockedObj<autoware_msgs::Lane> fin_wps_;

  // Config & Status
  LockedObj<autoware_msgs::NDTStat> ndt_stat_;
  LockedObj<std_msgs::Header> avoidance_done_;

  // Data.
  LockedObj<geometry_msgs::PoseStamped> last_cur_pose_;
  LockedObj<geometry_msgs::PoseStamped> last_valid_ndt_pose_;
  boost::circular_buffer<
      std::pair<geometry_msgs::PoseStamped, tf::StampedTransform>>
      last_valid_ndt_pose_queue_;
  LockedObj<tf::StampedTransform> last_valid_tf_odom_to_map_;
  LockedObj<int32_t> standstill_cnt_;
  LockedObj<int32_t> localization_unreliable_cnt_;
  LockedObj<int32_t> stopped_due_to_obstacle_cnt_;
  LockedObj<bool> pose_initializing_;
  LockedObj<bool> stop_request_;
  LockedObj<ros::Time> last_pose_init_time_;
  LockedObj<uint64_t> control_cycle_cnt_, checking_cycle_cnt_;
};

struct Publishers {
 public:
  Publishers(ros::NodeHandle &nh_) {
    // X. Publisher
    status_mgmt_status_pub_ = nh_.advertise<std_msgs::Bool>(
        "status_management_status", DEFAULT_PUB_QUEUE_SIZE, true);

    // Control related.
    yp_spur_free_pub_ = nh_.advertise<ypspur_ros::ControlMode>(
        "/ypspur/control_mode", DEFAULT_PUB_QUEUE_SIZE, false);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(
        "cmd_vel", DEFAULT_PUB_QUEUE_SIZE, false);
    cur_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "current_pose", DEFAULT_PUB_QUEUE_SIZE, false);
    cur_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "current_velocity", DEFAULT_PUB_QUEUE_SIZE, false);

    // Config
    ndt_config_pub_ = nh_.advertise<autoware_config_msgs::ConfigNDT>(
        "/config/ndt", DEFAULT_PUB_QUEUE_SIZE, true);
    ndt_init_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "/initialpose", DEFAULT_PUB_QUEUE_SIZE, true);

    voxel_filt_pub_ =
        nh_.advertise<autoware_config_msgs::ConfigVoxelGridFilter>(
            "config/voxel_grid_filter", DEFAULT_PUB_QUEUE_SIZE, true);

    // Request
    onetime_avoidance_req_pub_ = nh_.advertise<std_msgs::Header>(
        "onetime_avoidance_request", DEFAULT_PUB_QUEUE_SIZE, true);
    message_pub_ = nh_.advertise<std_msgs::String>(
        "status_message", DEFAULT_PUB_QUEUE_SIZE, true);
  }

 public:
  // Control
  ros::Publisher cmd_vel_pub_, yp_spur_free_pub_, status_mgmt_status_pub_;
  // Pose & Vel
  ros::Publisher cur_pose_pub_, cur_vel_pub_;
  // Config & Status
  ros::Publisher ndt_config_pub_, voxel_filt_pub_, message_pub_,
      onetime_avoidance_req_pub_, ndt_init_pose_pub_;

  tf::TransformBroadcaster tf_broadcaster_;

 private:
  static const int32_t DEFAULT_PUB_QUEUE_SIZE = 100;
};

struct Subscribers {
 public:
  Subscribers(ros::NodeHandle &nh, SyncState &sync_state,
              ros::Publisher &message_pub, CurrentPoseGenerator &pose_gen)
      : sync_state_(sync_state),
        message_pub_(message_pub),
        cur_pose_gen_(pose_gen) {
    // Control window
    engage_request_sub_ =
        nh.subscribe("control_window_engage_request", DEFAULT_SUB_QUEUE_SIZE,
                     &Subscribers::ControlWindowEngageRequestCallback, this);
    autorun_request_sub_ =
        nh.subscribe("control_window_autorun_request", DEFAULT_SUB_QUEUE_SIZE,
                     &Subscribers::ControlWindowAutorunRequestCallback, this);

    // Pose
    gnss_local_pose_sub_ =
        nh.subscribe("gnss_pose_local", DEFAULT_SUB_QUEUE_SIZE,
                     &Subscribers::GNSSLocalPoseCallback, this);
    ndt_pose_sub_ = nh.subscribe("ndt_pose", DEFAULT_SUB_QUEUE_SIZE,
                                 &Subscribers::NdtPoseCallback, this);

    // Odom & Control
    odom_sub_ = nh.subscribe("odom", DEFAULT_SUB_QUEUE_SIZE,
                             &Subscribers::OdomCallback, this);
    twist_raw_sub_ = nh.subscribe("twist_raw", DEFAULT_SUB_QUEUE_SIZE,
                                  &Subscribers::TwistRawCallback, this);

    // Waypoint and Lane
    cls_wp_idx_sub_ = nh.subscribe("/closest_waypoint", DEFAULT_SUB_QUEUE_SIZE,
                                   &Subscribers::ClosestWaypointCallback, this);
    obst_wp_idx_sub_ =
        nh.subscribe("obstacle_waypoint", DEFAULT_SUB_QUEUE_SIZE,
                     &Subscribers::ObstacleWaypointCallback, this);
    base_wps_sub_ = nh.subscribe("/base_waypoints", DEFAULT_SUB_QUEUE_SIZE,
                                 &Subscribers::BaseWaypointCallback, this);
    final_wps_sub_ = nh.subscribe("final_waypoints", DEFAULT_SUB_QUEUE_SIZE,
                                  &Subscribers::FinalWaypointsCallback, this);

    // Config & Status
    ndt_stat_sub_ = nh.subscribe("ndt_stat", DEFAULT_SUB_QUEUE_SIZE,
                                 &Subscribers::NdtStatCallback, this);
    avoidance_done_sub_ =
        nh.subscribe("avoidance_request_done", DEFAULT_SUB_QUEUE_SIZE,
                     &Subscribers::AvoidanceRequestDoneCallback, this);
  }

 private:
  void PublishStatusMessage(const std::string &text) {
    std_msgs::String msg;
    msg.data = text;
    message_pub_.publish(msg);
  }

  // Subscribe callback.
  void ControlWindowEngageRequestCallback(const std_msgs::Bool &msg) {
    sync_state_.engage_request_.SetObj(msg.data);
  }

  void ControlWindowAutorunRequestCallback(const std_msgs::Bool &msg) {
    sync_state_.autorun_request_.SetObj(msg.data);
  }

  void GNSSLocalPoseCallback(const geometry_msgs::PoseStamped &msg) {
    sync_state_.gnss_local_.SetObj(msg);
  }

  void NdtPoseCallback(const geometry_msgs::PoseStamped &msg) {
    cur_pose_gen_.UpdateNdtPose(msg.pose);
    sync_state_.ndt_pose_.SetObj(msg);
  }

  void OdomCallback(const nav_msgs::Odometry &msg) {
    cur_pose_gen_.UpdateOdomPose(msg);
    sync_state_.odom_.SetObj(msg);
  }

  void TwistRawCallback(const geometry_msgs::TwistStamped &msg) {
    sync_state_.twist_raw_.SetObj(msg);
  }

  void ClosestWaypointCallback(const std_msgs::Int32 &msg) {
    sync_state_.cls_wp_idx_.SetObj(msg);
  }

  void ObstacleWaypointCallback(const std_msgs::Int32 &msg) {
    sync_state_.obst_wp_idx_.SetObj(msg);
  }

  void BaseWaypointCallback(const autoware_msgs::Lane &msg) {
    sync_state_.base_wps_.SetObj(msg);
  }

  void FinalWaypointsCallback(const autoware_msgs::Lane &msg) {
    sync_state_.fin_wps_.SetObj(msg);
  }

  void NdtStatCallback(const autoware_msgs::NDTStat &msg) {
    sync_state_.ndt_stat_.SetObj(msg);
  }

  void AvoidanceRequestDoneCallback(const std_msgs::Header &msg) {
    sync_state_.avoidance_done_.SetObj(msg);
    PublishStatusMessage("Avoidance done. Back to waypoint following mode.");
  }

 private:
  SyncState &sync_state_;

  ros::Publisher &message_pub_;

  // Control window.
  ros::Subscriber engage_request_sub_, autorun_request_sub_;
  // Pose
  ros::Subscriber gnss_local_pose_sub_, ndt_pose_sub_;
  // Odom & Control
  ros::Subscriber odom_sub_, twist_raw_sub_;
  // Waypoint & Land
  ros::Subscriber cls_wp_idx_sub_, obst_wp_idx_sub_, base_wps_sub_,
      final_wps_sub_;
  // Config & Status
  ros::Subscriber ndt_stat_sub_, avoidance_done_sub_;

  CurrentPoseGenerator &cur_pose_gen_;

 public:
  tf::TransformListener tf_listener_;

 private:
  static const int32_t DEFAULT_SUB_QUEUE_SIZE = 100;
};

StatusManagementNodeletParams ReadStatusManagementNodeletParams(
    ros::NodeHandle &pnh);

void CallMapLoadService(ros::NodeHandle &nh,
                        const std::string &map_pcd_filepath);

void CallWaypointLoadService(ros::NodeHandle &nh,
                             const std::string &waypoint_csv_filepath);

void CallWorld2MapLoadService(ros::NodeHandle &nh,
                              const std::string &world_to_map_json_filepath);

bool InitializePoseViaGNSS(SyncState &sync_state, XYZRPY &init_pose,
                           const double gnss_check_freq);

bool CallPoseInitializeService(ros::NodeHandle &nh,
                               ros::Publisher &voxel_filt_pub,
                               XYZRPY &init_pose,
                               const double init_pose_srv_timeout);

bool CreateInitPoseFromConfig(const init_config &conf, SyncState &sync_state,
                              XYZRPY &init_pose, const double odom_wait_freq);

XYZRPY TryInitializePose(ros::NodeHandle &nh, ros::Publisher &voxel_filt_pub,
                         SyncState &sync_state, const XYZRPY &base_pose,
                         const bool via_gnss, const bool pose_init,
                         const double gnss_check_freq,
                         const double init_pose_srv_timeout);

}  // namespace koichi_robotics_lib

#endif