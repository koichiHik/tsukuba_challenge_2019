
// Original
#include "status_management_util.h"

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Autoware
#include <autoware_config_msgs/ConfigNDT.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/NDTStat.h>

// Glog
#include <glog/logging.h>

// Boost
#include <boost/circular_buffer.hpp>

// STL
#include <mutex>
#include <thread>

namespace {

static const int32_t DEFAULT_PUB_QUEUE_SIZE = 100;
static const int32_t DEFAULT_SUB_QUEUE_SIZE = 100;

// Timeout
static constexpr double DEFAULT_SERVICE_TIMEOUT_SEC = 20.0;

// Frequency.
static constexpr double DEFAULT_INIT_ODOM_WAIT_FREQ = 2.0;
static constexpr double DEFAULT_INIT_GNSS_CHECK_FREQ = 2.0;
static constexpr double DEFAULT_CONTROL_CYCLE_FREQ = 50.0;
static constexpr double DEFAULT_CHECKING_CYCLE_FREQ = 1.0;
static constexpr double DEFAULT_ROBOT_STOP_CHECK_FREQ = 1.0;

// Standstill Thresh
static constexpr double DEFAULT_STANDSTILL_VX_THR = 0.01;
static constexpr double DEFAULT_STANDSTILL_WX_THR = 0.01;

// Time thresh
static constexpr double DEFAULT_LOCALIZATION_REINIT_MINIMUM_PERIOD = 20.0;
static constexpr double DEFAULT_LOCALIZATION_ODOM_TRUST_PERIOD = 60.0;

// Count thresh
static const int32_t DEFAULT_LOCALIZAITON_UNRELIABLE_CNT_FOR_REINIT =
    DEFAULT_CONTROL_CYCLE_FREQ * 2;
static const int32_t DEFAULT_INIT_POSE_STANDSTILL_COUNT = 10;
static const int32_t DEFAULT_LOCALIZATION_RELIABLE_CNT = 60;

// Pose diff thresh.
static constexpr double DEFAULT_LOCALIZATION_TRANS_THRESH = 2.0;
static constexpr double DEFAULT_LOCALIZATION_ORIEN_THRESH = 45 / 180.0 * M_PI;
static constexpr double DEFAULT_LOCALIZATION_TRANS_RELIABLE_THRESH = 0.75;
static constexpr double DEFAULT_LOCALIZATION_ORIEN_RELIABLE_THRESH =
    15 / 180.0 * M_PI;

// Obstacle distance.
static constexpr double DEFAULT_OBSTACLE_DISTANCE_THRESH = 2.0;
static constexpr double DEFAULT_OBSTACLE_WAIT_BEFORE_AVOID = 5.0;

}  // namespace

namespace {

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

struct StatusManagementParams {
  bool init_from_gnss;
  bool use_pose_search;
  double x, y, z, roll, pitch, yaw;
};

}  // namespace

namespace koichi_robotics_lib {

class StatusManagementNodelet : public nodelet::Nodelet {
 public:
  StatusManagementNodelet();

  ~StatusManagementNodelet();

  virtual void onInit();

 private:
  void Initialize();

  void ReadParams(StatusManagementParams &params);

  void PreparePublisher();

  void PrepareSubscriber();

  XYZRPY InitializePose(const bool via_gnss, const bool pose_init,
                        const XYZRPY &pose);

  void PublishStatusMessage(const std::string &msg);

  void RunControlCycle();

  void RunCheckingCycle();

  void CheckPoseValidity();

  void CreateAndPublishCurrentPose(const ros::Time &time);

  void CheckRobotStopReason();

  void RunPoseInitializerIfNecessary();

  void CheckIfEndIsReached();

 private:
  // Subscribers.
  // Pose
  void GNSSLocalPoseCallback(const geometry_msgs::PoseStamped &msg);

  void NdtPoseCallback(const geometry_msgs::PoseStamped &msg);

  void AmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

  // Odom & Control
  void OdomCallback(const nav_msgs::Odometry &msg);

  void TwistRawCallback(const geometry_msgs::TwistStamped &msg);

  // Waypoint & Lane
  void ClosestWaypointCallback(const std_msgs::Int32 &msg);

  void ObstacleWaypointCallback(const std_msgs::Int32 &msg);

  void BaseWaypointCallback(const autoware_msgs::Lane &msg);

  void FinalWaypointsCallback(const autoware_msgs::Lane &msg);

  // Config & Status
  void NdtStatCallback(const autoware_msgs::NDTStat &msg);

  void AvoidanceRequestDoneCallback(const std_msgs::Header &msg);

 private:
  ros::NodeHandle nh_, pnh_;

  // X. Tf
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  // X. Parameter
  StatusManagementParams params_;

  // X. Subscriber
  // Pose
  ros::Subscriber gnss_local_pose_sub_, ndt_pose_sub_, amcl_pose_sub_;
  // Odom & Control
  ros::Subscriber odom_sub_, twist_raw_sub_;
  // Waypoint & Land
  ros::Subscriber cls_wp_idx_sub_, obst_wp_idx_sub_, base_wps_sub_,
      final_wps_sub_;
  // Config & Status
  ros::Subscriber ndt_stat_sub_, avoidance_done_sub_;

  // X. Publisher
  // Control
  ros::Publisher cmd_vel_pub_;
  // Pose & Vel
  ros::Publisher cur_pose_pub_, cur_vel_pub_;
  // Config & Status
  ros::Publisher ndt_config_pub_, voxel_filt_pub_, mcl_3dl_init_pub_,
      message_pub_, onetime_avoidance_req_pub_;

  // X. Subscribed message.
  // Pose
  LockedObj<geometry_msgs::PoseStamped> gnss_local_;
  LockedObj<geometry_msgs::PoseStamped> ndt_pose_;
  LockedObj<geometry_msgs::PoseWithCovarianceStamped> amcl_pose_;

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
  LockedObj<ros::Time> last_pose_init_time_;
  uint64_t control_cycle_cnt_, checking_cycle_cnt_;

  std::thread thread_control_, thread_checking_;
};

StatusManagementNodelet::StatusManagementNodelet()
    : nh_(),
      pnh_(),
      tf_broadcaster_(),
      tf_listener_(),
      params_(),
      gnss_local_pose_sub_(),
      ndt_pose_sub_(),
      amcl_pose_sub_(),
      odom_sub_(),
      twist_raw_sub_(),
      cls_wp_idx_sub_(),
      obst_wp_idx_sub_(),
      base_wps_sub_(),
      final_wps_sub_(),
      ndt_stat_sub_(),
      avoidance_done_sub_(),
      cmd_vel_pub_(),
      cur_pose_pub_(),
      cur_vel_pub_(),
      ndt_config_pub_(),
      voxel_filt_pub_(),
      mcl_3dl_init_pub_(),
      message_pub_(),
      onetime_avoidance_req_pub_(),
      gnss_local_(),
      ndt_pose_(),
      amcl_pose_(),
      odom_(),
      twist_raw_(),
      cls_wp_idx_(),
      obst_wp_idx_(),
      base_wps_(),
      fin_wps_(),
      ndt_stat_(),
      avoidance_done_(),
      last_cur_pose_(),
      last_valid_ndt_pose_(),
      last_valid_ndt_pose_queue_(),
      last_valid_tf_odom_to_map_(),
      standstill_cnt_(),
      localization_unreliable_cnt_(),
      stopped_due_to_obstacle_cnt_(),
      pose_initializing_(),
      last_pose_init_time_(),
      control_cycle_cnt_(),
      checking_cycle_cnt_(),
      thread_control_(),
      thread_checking_() {}

StatusManagementNodelet::~StatusManagementNodelet() {
  // X. Terminate.
  thread_control_.join();
  thread_checking_.join();
}

void StatusManagementNodelet::onInit() {
  thread_control_ = std::thread(&StatusManagementNodelet::Initialize, this);
}

void StatusManagementNodelet::Initialize() {
  ROS_WARN("StatusManagementNodelet initialize");

  /*
  ros::Rate r(0.1);
  while (ros::Time::now().is_zero()) {
    r.sleep();
  }
  ROS_WARN("Current time : %lf", ros::Time::now().toSec());
  */

  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();

  // Variable init
  {
    std_msgs::Int32 n;
    n.data = -1;
    cls_wp_idx_.SetObj(n);
    obst_wp_idx_.SetObj(n);
    standstill_cnt_.SetObj(0);
    localization_unreliable_cnt_.SetObj(0);
    stopped_due_to_obstacle_cnt_.SetObj(0);
    pose_initializing_.SetObj(false);
    control_cycle_cnt_ = 0;
    checking_cycle_cnt_ = 0;
    last_valid_ndt_pose_queue_.set_capacity(DEFAULT_LOCALIZATION_RELIABLE_CNT);
  }

  // Read params.
  ReadParams(params_);

  // Publisher
  PreparePublisher();

  // Messaging.
  PublishStatusMessage("Initializing system. Please wait for a moment.");

  // Subscriber
  PrepareSubscriber();

  // If direct initialization, keep parameter value as valid.
  if (!params_.use_pose_search) {
    ros::Rate r(DEFAULT_INIT_ODOM_WAIT_FREQ);
    while (ros::ok() && odom_.GetObj().header.stamp.is_zero()) {
      r.sleep();
    }
    nav_msgs::Odometry odom = odom_.GetObj();
    geometry_msgs::Pose init_pose =
        CreatePoseFromXYZRPY(XYZRPY(params_.x, params_.y, params_.z,
                                    params_.roll, params_.pitch, params_.yaw));
    geometry_msgs::PoseStamped init_pose_stamped;
    ros::Time cur_time = ros::Time::now();
    init_pose_stamped.header.stamp = cur_time;
    init_pose_stamped.header.frame_id = "map";
    init_pose_stamped.pose = init_pose;
    last_valid_ndt_pose_.SetObj(init_pose_stamped);

    tf::Transform trans_odom_to_map =
        CreateTransformFromPose(init_pose, odom.pose.pose);

    last_valid_tf_odom_to_map_.SetObj(
        tf::StampedTransform(trans_odom_to_map, cur_time, "map", "odom"));
  }

  // Initialize pose.
  XYZRPY init_pose(params_.x, params_.y, params_.z, params_.roll, params_.pitch,
                   params_.yaw);
  XYZRPY init_searched_pose = InitializePose(
      params_.init_from_gnss, params_.use_pose_search, init_pose);

  // Dispatch thread.
  thread_checking_ =
      std::thread(&StatusManagementNodelet::RunCheckingCycle, this);
  RunControlCycle();
}

void StatusManagementNodelet::ReadParams(StatusManagementParams &params) {
  CHECK(pnh_.getParam("init_from_gnss", params.init_from_gnss))
      << "[StatusManagementNodelet] Parameter init_from_gnss cannot be read.";
  CHECK(pnh_.getParam("use_pose_search", params.use_pose_search))
      << "[StatusManagementNodelet] Parameter use_pose_search cannot be read.";
  CHECK(pnh_.getParam("x", params.x))
      << "[SattusManagementNodelet] Parameter x cannot be read.";
  CHECK(pnh_.getParam("y", params.y))
      << "[StatusManagementNodelet] Parameter y cannot be read.";
  CHECK(pnh_.getParam("z", params.z))
      << "[StatusManagementNodelet] Parameter z cannot be read.";
  CHECK(pnh_.getParam("roll", params.roll))
      << "[StatusManagementNodelet] Parameter roll cannot be read.";
  CHECK(pnh_.getParam("pitch", params.pitch))
      << "[StatusManagementNodelet] Parameter pitch cannot be read.";
  CHECK(pnh_.getParam("yaw", params.yaw))
      << "[StatusManagementNodelet] Parameter yaw cannot be read.";
}

void StatusManagementNodelet::PreparePublisher() {
  // X. Publisher

  // Control related.
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(
      "cmd_vel", DEFAULT_PUB_QUEUE_SIZE, false);
  cur_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "current_pose", DEFAULT_PUB_QUEUE_SIZE, false);
  cur_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "current_velocity", DEFAULT_PUB_QUEUE_SIZE, false);

  // Config
  ndt_config_pub_ =
      nh_.advertise<autoware_config_msgs::ConfigNDT>("/config/ndt", true);
  voxel_filt_pub_ = nh_.advertise<autoware_config_msgs::ConfigVoxelGridFilter>(
      "config/voxel_grid_filter", true);
  mcl_3dl_init_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/mcl_3dl/initialpose", true);

  // Request
  onetime_avoidance_req_pub_ =
      nh_.advertise<std_msgs::Header>("onetime_avoidance_request", true);
  message_pub_ = nh_.advertise<std_msgs::String>("status_message",
                                                 DEFAULT_PUB_QUEUE_SIZE, true);
}

void StatusManagementNodelet::PrepareSubscriber() {
  // Pose
  gnss_local_pose_sub_ =
      nh_.subscribe("gnss_pose_local", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::GNSSLocalPoseCallback, this);
  ndt_pose_sub_ =
      nh_.subscribe("ndt_pose", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::NdtPoseCallback, this);
  amcl_pose_sub_ =
      nh_.subscribe("amcl_pose", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::AmclPoseCallback, this);

  // Odom & Control
  odom_sub_ = nh_.subscribe("odom", DEFAULT_SUB_QUEUE_SIZE,
                            &StatusManagementNodelet::OdomCallback, this);
  twist_raw_sub_ =
      nh_.subscribe("twist_raw", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::TwistRawCallback, this);

  // Waypoint and Lane
  cls_wp_idx_sub_ =
      nh_.subscribe("/closest_waypoint", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::ClosestWaypointCallback, this);
  obst_wp_idx_sub_ =
      nh_.subscribe("obstacle_waypoint", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::ObstacleWaypointCallback, this);
  base_wps_sub_ =
      nh_.subscribe("/base_waypoints", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::BaseWaypointCallback, this);
  final_wps_sub_ =
      nh_.subscribe("final_waypoints", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::FinalWaypointsCallback, this);

  // Config & Status
  ndt_stat_sub_ =
      nh_.subscribe("ndt_stat", DEFAULT_SUB_QUEUE_SIZE,
                    &StatusManagementNodelet::NdtStatCallback, this);
  avoidance_done_sub_ = nh_.subscribe(
      "avoidance_request_done", DEFAULT_SUB_QUEUE_SIZE,
      &StatusManagementNodelet::AvoidanceRequestDoneCallback, this);
}

XYZRPY StatusManagementNodelet::InitializePose(const bool via_gnss,
                                               const bool pose_init,
                                               const XYZRPY &pose) {
  XYZRPY init_pose = pose;

  // X. Update from GNSS is ON.
  if (via_gnss) {
    // X. Wait update from GNSS.
    ros::Rate r(DEFAULT_INIT_GNSS_CHECK_FREQ);
    while (gnss_local_.GetObj().header.stamp.is_zero() && ros::ok()) {
      ROS_WARN("Waiting for GNSS data to be received.");
      r.sleep();
    }

    // X. Update init pose.
    init_pose = CreateXYZRPYFromPose(gnss_local_.GetObj().pose);
    init_pose.roll_ = 0.0;
    init_pose.pitch_ = 0.0;
    init_pose.yaw_ = 0.0;
  }

  // X. Use pose initializer.
  if (pose_init) {
    PublishStatusMessage(
        "Start pose initializer. This will take about twenty seconds.");
    voxel_filt_pub_.publish(CONFIG_VOXELFILT_POSE_SEARCH());
    messages::initialize_pose srv = POSE_INIT_REQUEST_FULL(init_pose);
    ros::ServiceClient client =
        nh_.serviceClient<messages::initialize_pose>("/initialize_pose");
    if (!client.waitForExistence(ros::Duration(DEFAULT_SERVICE_TIMEOUT_SEC))) {
      LOG(FATAL) << "Failed to wait for service. : /initialize_pose";
    }
    if (!client.call(srv)) {
      LOG(FATAL) << "Failed to call service. : /initialize_pose";
    }

    // X. Update pose.
    init_pose.x_ = srv.response.x;
    init_pose.y_ = srv.response.y;
    init_pose.z_ = srv.response.z;
    init_pose.roll_ = srv.response.roll;
    init_pose.pitch_ = srv.response.pitch;
    init_pose.yaw_ = srv.response.yaw;
  }

  // X. Set finer resoluiton to voxelfilter.
  voxel_filt_pub_.publish(CONFIG_VOXELFILT_POSE_RUN());

  // X. Initialize ndt localizer.
  ndt_config_pub_.publish(CONFIG_DEFAULT_NDT(init_pose));

  // X. Initialize amcl localizer.
  mcl_3dl_init_pub_.publish(
      CreatePoseWithCovarianceStamped(init_pose, ros::Time::now()));

  // X. Status Announce.
  PublishStatusMessage("Pose initialization is done. Start control.");

  return init_pose;
}

void StatusManagementNodelet::PublishStatusMessage(const std::string &text) {
  std_msgs::String msg;
  msg.data = text;
  message_pub_.publish(msg);
}

void StatusManagementNodelet::RunControlCycle() {
  control_cycle_cnt_ = 0;
  ros::Rate r(DEFAULT_CONTROL_CYCLE_FREQ);
  while (ros::ok()) {
    control_cycle_cnt_++;
    // X. Ros time.
    ros::Time time = ros::Time::now();

    // X. Control command routing.
    {
      geometry_msgs::TwistStamped twist_stamped = twist_raw_.GetObj();
      if (!twist_stamped.header.stamp.is_zero() &&
          !pose_initializing_.GetObj()) {
        cmd_vel_pub_.publish(twist_stamped.twist);
      } else {
        cmd_vel_pub_.publish(CREATE_ZERO_TWIST());
      }
    }

    // X. Velocity routing.
    {
      nav_msgs::Odometry odom = odom_.GetObj();
      if (!odom.header.stamp.is_zero()) {
        geometry_msgs::TwistStamped twist_stamped;
        twist_stamped.header.stamp = time;
        twist_stamped.twist = odom.twist.twist;
        cur_vel_pub_.publish(twist_stamped);

        if (TwistIsBelowThreshold(twist_stamped.twist,
                                  DEFAULT_STANDSTILL_VX_THR,
                                  DEFAULT_STANDSTILL_WX_THR)) {
          standstill_cnt_.SetObj(standstill_cnt_.GetObj() + 1);
        } else {
          standstill_cnt_.SetObj(0);
        }
      }
    }

    // X. Check pose validity.
    if (!pose_initializing_.GetObj()) {
      CheckPoseValidity();
    }

    // X. Current pose.
    CreateAndPublishCurrentPose(time);

    r.sleep();
  }
}

void StatusManagementNodelet::RunCheckingCycle() {
  ros::Rate r(DEFAULT_CHECKING_CYCLE_FREQ);
  checking_cycle_cnt_ = 0;
  while (ros::ok()) {
    checking_cycle_cnt_++;
    // X. Robot stuck reason.
    CheckRobotStopReason();

    // X. Location status.
    RunPoseInitializerIfNecessary();

    // X. Check if reached goal.
    CheckIfEndIsReached();

    r.sleep();
  }
}

void StatusManagementNodelet::CheckIfEndIsReached() {
  autoware_msgs::Lane base_wps = base_wps_.GetObj();
  std_msgs::Int32 cur_idx = cls_wp_idx_.GetObj();
  nav_msgs::Odometry odom = odom_.GetObj();

  static int count = 0;
  if (0 < base_wps.waypoints.size() &&
      base_wps.waypoints.size() - cur_idx.data < 3 &&
      TwistIsBelowThreshold(odom.twist.twist, DEFAULT_STANDSTILL_VX_THR,
                            DEFAULT_STANDSTILL_WX_THR)) {
    if (count % (5 * static_cast<int>(DEFAULT_CHECKING_CYCLE_FREQ)) == 0) {
      PublishStatusMessage(
          "Reached stop waypoint. Please provide next waypoints to follow.");
    }
    count++;
  } else {
    count = 0;
  }
}

void StatusManagementNodelet::CreateAndPublishCurrentPose(
    const ros::Time &time) {
  nav_msgs::Odometry odom = odom_.GetObj();
  if (!odom.header.stamp.is_zero()) {
    geometry_msgs::PoseStamped cur_pose;
    cur_pose.header.stamp = time;
    cur_pose.header.frame_id = "map";

    static int ndt_unreliable_cnt = 0;
    geometry_msgs::PoseStamped ndt_pose = ndt_pose_.GetObj();

    //
    // ROS_WARN("Localization Unreliable Cnt : %d",
    //         localization_unreliable_cnt_.GetObj());
    if (!ndt_pose.header.stamp.is_zero() &&
        localization_unreliable_cnt_.GetObj() == 0 &&
        !pose_initializing_.GetObj()) {
      // X. Use ndt pose as cur pose.
      cur_pose.pose = ndt_pose.pose;
      tf::Transform trans_odom_to_map =
          CreateTransformFromPose(cur_pose.pose, odom.pose.pose);

      // X. Send tf & cur pose
      // ROS_WARN("Publish transform from ndt.");
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(trans_odom_to_map, time, "map", "odom"));

      // ROS_WARN("Before cur pose pub");
      cur_pose_pub_.publish(cur_pose);
      last_cur_pose_.SetObj(cur_pose);

      ndt_unreliable_cnt = 0;

    } else {
      // X. Use odometry as cur pose.
      tf::StampedTransform transform_st = last_valid_tf_odom_to_map_.GetObj();
      if (!transform_st.stamp_.is_zero()) {
        // X. Send transform.
        // ROS_WARN("Publish transform from odom.");
        transform_st.stamp_ = time;
        tf_broadcaster_.sendTransform(transform_st);

        // X. Transform.
        cur_pose.pose = TransformPose(transform_st, odom.pose.pose);

        // X.
        // ROS_WARN("Before cur pose pub");
        cur_pose_pub_.publish(cur_pose);
        last_cur_pose_.SetObj(cur_pose);

        ndt_unreliable_cnt = ndt_unreliable_cnt + 1;
        if (ndt_unreliable_cnt %
                (5 * static_cast<int>(DEFAULT_CONTROL_CYCLE_FREQ)) ==
            0) {
          PublishStatusMessage("Ndt pose is unreliable");
        }
      }
    }
  }
}

void StatusManagementNodelet::CheckRobotStopReason() {
  int32_t obs_idx = obst_wp_idx_.GetObj().data;

  // X. Robot is not stopped.
  // ROS_WARN("Obstacle Idx : %d", obs_idx);
  if (obs_idx == -1) {
    stopped_due_to_obstacle_cnt_.SetObj(0);
    return;
  }

  // X.
  // ROS_WARN("Twist is almost zero");
  geometry_msgs::TwistStamped twist_raw = twist_raw_.GetObj();
  if (twist_raw.header.stamp.is_zero() ||
      !TwistIsBelowThreshold(twist_raw.twist, DEFAULT_STANDSTILL_VX_THR,
                             DEFAULT_STANDSTILL_WX_THR)) {
    stopped_due_to_obstacle_cnt_.SetObj(0);
    return;
  }

  // X.
  autoware_msgs::Lane fin_wps = fin_wps_.GetObj();
  geometry_msgs::PoseStamped last_cur_pose = last_cur_pose_.GetObj();

  // ROS_WARN("Waypoint time : %lf", fin_wps.header.stamp.toSec());
  // ROS_WARN("Last cur pose : %lf", last_cur_pose.header.stamp.toSec());
  if (fin_wps.waypoints.size() == 0 || last_cur_pose.header.stamp.is_zero()) {
    stopped_due_to_obstacle_cnt_.SetObj(0);
    return;
  }

  // X.
  geometry_msgs::PoseStamped cur_pose = last_cur_pose_.GetObj();

  ROS_WARN("Distance : %lf", ComputeDistanceToObstacleOnWaypoint(
                                 obs_idx, fin_wps, cur_pose.pose));
  if (ComputeDistanceToObstacleOnWaypoint(obs_idx, fin_wps, cur_pose.pose) <
      DEFAULT_OBSTACLE_DISTANCE_THRESH) {
    // ROS_WARN("Robot stop due to obstacle detected.");

    // X. Count up.
    stopped_due_to_obstacle_cnt_.SetObj(stopped_due_to_obstacle_cnt_.GetObj() +
                                        1);
    // X. Status announce.
    int message_duration = 3;
    int stopped_cnt = stopped_due_to_obstacle_cnt_.GetObj();
    if (static_cast<int>(stopped_cnt / DEFAULT_CHECKING_CYCLE_FREQ) %
            message_duration ==
        0) {
      PublishStatusMessage("Robot stops due to obstacle in front.");
    }

    // X. Wait for avoidance.
    if (DEFAULT_OBSTACLE_WAIT_BEFORE_AVOID <
        stopped_cnt / DEFAULT_CHECKING_CYCLE_FREQ) {
      if (IsAvoidanceOkWaypoint(obs_idx, fin_wps)) {
        PublishStatusMessage("Rerouting for obstacle avoidance.");
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        onetime_avoidance_req_pub_.publish(header);
        stopped_due_to_obstacle_cnt_.SetObj(0);
      } else {
        PublishStatusMessage("Rerouting is not allowed here. Keep waiting.");
        stopped_due_to_obstacle_cnt_.SetObj(0);
      }
    }
  }
}

void StatusManagementNodelet::CheckPoseValidity() {
  geometry_msgs::PoseStamped ndt_pose = ndt_pose_.GetObj();
  geometry_msgs::PoseWithCovarianceStamped amcl_pose = amcl_pose_.GetObj();

  // X. During pose initialization, skip.
  if (pose_initializing_.GetObj() || ndt_pose.header.stamp.is_zero() ||
      amcl_pose.header.stamp.is_zero()) {
    return;
  }

  // X. Map pose is not stable.
  if (!PoseDiffIsBelowThreshold(ndt_pose.pose, amcl_pose.pose.pose,
                                DEFAULT_LOCALIZATION_TRANS_THRESH,
                                DEFAULT_LOCALIZATION_ORIEN_THRESH)) {
    localization_unreliable_cnt_.SetObj(
        std::min(localization_unreliable_cnt_.GetObj() + 1,
                 DEFAULT_LOCALIZAITON_UNRELIABLE_CNT_FOR_REINIT));

  } else {
    // X. When map pose is stable.
    if (PoseDiffIsBelowThreshold(ndt_pose.pose, amcl_pose.pose.pose,
                                 DEFAULT_LOCALIZATION_TRANS_RELIABLE_THRESH,
                                 DEFAULT_LOCALIZATION_ORIEN_RELIABLE_THRESH)) {
      try {
        tf::StampedTransform tf_odom_to_map;
        tf_listener_.lookupTransform("map", "odom", ndt_pose.header.stamp,
                                     tf_odom_to_map);
        last_valid_ndt_pose_queue_.push_back(
            std::make_pair(ndt_pose, tf_odom_to_map));
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }

      // X. If reaches stable count, treat the first one as valid pose.
      if (last_valid_ndt_pose_queue_.size() ==
          DEFAULT_LOCALIZATION_RELIABLE_CNT) {
        std::pair<geometry_msgs::PoseStamped, tf::StampedTransform> e =
            last_valid_ndt_pose_queue_.front();
        last_valid_ndt_pose_queue_.pop_front();
        last_valid_ndt_pose_.SetObj(e.first);
        last_valid_tf_odom_to_map_.SetObj(e.second);
      }

    } else {
      last_valid_ndt_pose_queue_.clear();
    }

    // X. Decrement unreliable count.
    localization_unreliable_cnt_.SetObj(
        std::max(0, localization_unreliable_cnt_.GetObj() - 1));
  }
}

void StatusManagementNodelet::RunPoseInitializerIfNecessary() {
  ros::Time last_pose_init_time = last_pose_init_time_.GetObj();
  if (last_pose_init_time.is_zero() ||
      ros::Time::now() - last_pose_init_time >
          ros::Duration(DEFAULT_LOCALIZATION_REINIT_MINIMUM_PERIOD)) {
    if (DEFAULT_LOCALIZAITON_UNRELIABLE_CNT_FOR_REINIT <=
        localization_unreliable_cnt_.GetObj()) {
      pose_initializing_.SetObj(true);

      // X. Make sure that robot stops.
      ros::Rate r(DEFAULT_ROBOT_STOP_CHECK_FREQ);
      int wait_cnt = 0;
      while (standstill_cnt_.GetObj() < DEFAULT_INIT_POSE_STANDSTILL_COUNT) {
        wait_cnt += 1;
        if (wait_cnt % 5) {
          PublishStatusMessage("Robot will stop for pose initialization.");
        }
        r.sleep();
      }

      // X. Choose initial position for search.
      if (!last_valid_ndt_pose_.GetObj().header.stamp.is_zero() &&
          ros::Time::now() - last_valid_ndt_pose_.GetObj().header.stamp <
              ros::Duration(DEFAULT_LOCALIZATION_ODOM_TRUST_PERIOD)) {
        PublishStatusMessage("Reinitialize localization based on odometry.");
        InitializePose(false, true,
                       CreateXYZRPYFromPose(last_cur_pose_.GetObj().pose));
      } else {
        PublishStatusMessage("Reinitialize localization based on GNSS.");
        InitializePose(true, true, XYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      }

      // X. Memorize last update.
      last_pose_init_time_.SetObj(ros::Time::now());
      pose_initializing_.SetObj(false);
      localization_unreliable_cnt_.SetObj(
          std::max(0, localization_unreliable_cnt_.GetObj() -
                          DEFAULT_LOCALIZAITON_UNRELIABLE_CNT_FOR_REINIT / 2));
    }
  }
}

// Subscribe callback.
void StatusManagementNodelet::GNSSLocalPoseCallback(
    const geometry_msgs::PoseStamped &msg) {
  gnss_local_.SetObj(msg);
}

void StatusManagementNodelet::NdtPoseCallback(
    const geometry_msgs::PoseStamped &msg) {
  ndt_pose_.SetObj(msg);
}

void StatusManagementNodelet::AmclPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped &msg) {
  amcl_pose_.SetObj(msg);
}

void StatusManagementNodelet::OdomCallback(const nav_msgs::Odometry &msg) {
  odom_.SetObj(msg);
}

void StatusManagementNodelet::TwistRawCallback(
    const geometry_msgs::TwistStamped &msg) {
  twist_raw_.SetObj(msg);
}

void StatusManagementNodelet::ClosestWaypointCallback(
    const std_msgs::Int32 &msg) {
  cls_wp_idx_.SetObj(msg);
}

void StatusManagementNodelet::ObstacleWaypointCallback(
    const std_msgs::Int32 &msg) {
  obst_wp_idx_.SetObj(msg);
}

void StatusManagementNodelet::BaseWaypointCallback(
    const autoware_msgs::Lane &msg) {
  base_wps_.SetObj(msg);
}

void StatusManagementNodelet::FinalWaypointsCallback(
    const autoware_msgs::Lane &msg) {
  fin_wps_.SetObj(msg);
}

void StatusManagementNodelet::NdtStatCallback(
    const autoware_msgs::NDTStat &msg) {
  ndt_stat_.SetObj(msg);
}

void StatusManagementNodelet::AvoidanceRequestDoneCallback(
    const std_msgs::Header &msg) {
  avoidance_done_.SetObj(msg);
  PublishStatusMessage("Avoidance done. Back to waypoint following mode.");
}

}  // namespace koichi_robotics_lib

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(koichi_robotics_lib::StatusManagementNodelet,
                       nodelet::Nodelet)
