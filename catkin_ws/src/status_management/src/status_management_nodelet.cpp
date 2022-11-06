
// Original
#include <status_management/autorun_config.h>
#include <status_management/status_management_helper.h>
#include <status_management/status_management_util.h>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <ypspur_ros/ControlMode.h>

// Autoware
#include <autoware_config_msgs/ConfigNDT.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/String.h>

// Glog
#include <glog/logging.h>

// Boost
#include <boost/circular_buffer.hpp>

// STL
#include <mutex>
#include <thread>

namespace {

// Frequency.
static constexpr double DEFAULT_INIT_ODOM_WAIT_FREQ = 2.0;
static constexpr double DEFAULT_INIT_GNSS_CHECK_FREQ = 2.0;
static constexpr double DEFAULT_CONTROL_CYCLE_FREQ = 50.0;
static constexpr double DEFAULT_MESSAGE_CYCLE = 5.0;

}  // namespace

namespace koichi_robotics_lib {

class StatusManagementNodelet : public nodelet::Nodelet {
 public:
  StatusManagementNodelet();

  ~StatusManagementNodelet();

  virtual void onInit();

 private:
  void Initialize();

  XYZRPY InitializePose(const init_config &conf);

  void PublishStatusMessage(const std::string &msg, const double sec);

  void RunControlCycle();

  void RunCheckingCycle();

  void CheckPoseValidity();

  void CreateAndPublishCurrentPose(const ros::Time &time);

  void CheckRobotStopReason();

  void RunPoseInitializerIfNecessary();

  bool CheckIfEndIsReached();

 private:
  ros::NodeHandle nh_, pnh_;

  // X. Parameter
  StatusManagementNodeletParams params_;

  std::unique_ptr<SyncState> p_sync_state_;
  std::unique_ptr<CourseConfigMgmt> p_course_conf_mgmt_;
  std::unique_ptr<Publishers> p_pubs_;
  std::unique_ptr<Subscribers> p_subs_;

  std::map<std::string, ros::Time> message_time_map_;

  std::thread thread_control_, thread_checking_;
};

StatusManagementNodelet::StatusManagementNodelet()
    : nh_(),
      pnh_(),
      params_(),
      p_sync_state_(),
      p_course_conf_mgmt_(),
      p_pubs_(),
      p_subs_(),
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

  // X. Initialize
  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();
  params_ = ReadStatusManagementNodeletParams(pnh_);
  p_sync_state_.reset(new SyncState(params_.localization_reliable_count));
  p_course_conf_mgmt_.reset(new CourseConfigMgmt(params_.course_config_yaml,
                                                 params_.start_course_idx));
  p_pubs_.reset(new Publishers(nh_));
  p_subs_.reset(new Subscribers(nh_, *p_sync_state_, p_pubs_->message_pub_));

  // Messaging.
  PublishStatusMessage("Initializing system. Please wait for a moment.",
                       DEFAULT_MESSAGE_CYCLE);

  // X. Initialize voxel filter.
  p_pubs_->voxel_filt_pub_.publish(CONFIG_VOXELFILT_POSE_RUN());

  // X. Load map and waypoint.
  {
    autorun_config conf = p_course_conf_mgmt_->GetCurrentConfig();
    CallMapLoadService(nh_, conf.file_conf_.map_pcd_file_);
    if (!params_.localize_only) {
      CallWaypointLoadService(nh_, conf.file_conf_.lane_csv_file_);
      CallWorld2MapLoadService(nh_, conf.file_conf_.world_to_map_json_file_);
    }
    InitializePose(conf.init_conf_);
  }

  // Dispatch thread.
  thread_checking_ =
      std::thread(&StatusManagementNodelet::RunCheckingCycle, this);
  RunControlCycle();
}

XYZRPY StatusManagementNodelet::InitializePose(const init_config &conf) {
  XYZRPY init_pose;
  CreateInitPoseFromConfig(conf, *p_sync_state_, init_pose,
                           DEFAULT_INIT_ODOM_WAIT_FREQ);

  // X. Update from GNSS is ON.
  if (conf.init_via_gnss_) {
    InitializePoseViaGNSS(*p_sync_state_, init_pose,
                          DEFAULT_INIT_GNSS_CHECK_FREQ);
  }

  // X. Use pose initializer.
  if (conf.pose_initializer_) {
    PublishStatusMessage(
        "Start pose initializer. This will take about twenty seconds.",
        DEFAULT_MESSAGE_CYCLE);
    CallPoseInitializeService(nh_, p_pubs_->voxel_filt_pub_, init_pose,
                              params_.init_pose_srv_timeout);
  }

  InitializeLocalizer(p_pubs_->ndt_config_pub_, p_pubs_->mcl_3dl_init_pub_,
                      init_pose);

  // X. Status Announce.
  PublishStatusMessage("Pose initialization is done. Start control.",
                       DEFAULT_MESSAGE_CYCLE);

  return init_pose;
}

void StatusManagementNodelet::PublishStatusMessage(const std::string &text,
                                                   const double sec) {
  std_msgs::String msg;
  msg.data = text;
  ros::Time cur_time = ros::Time::now();
  if (message_time_map_.find(text) == message_time_map_.end()) {
    message_time_map_.insert(std::make_pair(text, cur_time));
    p_pubs_->message_pub_.publish(msg);
  } else if (ros::Duration(sec) < cur_time - message_time_map_.at(text)) {
    message_time_map_.at(text) = cur_time;
    p_pubs_->message_pub_.publish(msg);
  }
}

void StatusManagementNodelet::RunControlCycle() {
  ros::Rate r(DEFAULT_CONTROL_CYCLE_FREQ);
  while (ros::ok()) {
    p_sync_state_->control_cycle_cnt_.SetObj(
        p_sync_state_->control_cycle_cnt_.GetObj() + 1);
    // X. Ros time.
    ros::Time time = ros::Time::now();

    // X. Control command routing.
    {
      if (!p_sync_state_->engage_request_.GetObj()) {
        // ypspur_ros::ControlMode msg;
        // msg.vehicle_control_mode = 0;
        // p_pubs_->yp_spur_free_pub_.publish(msg);
      } else {
        ypspur_ros::ControlMode msg;
        msg.vehicle_control_mode = 2;
        p_pubs_->yp_spur_free_pub_.publish(msg);
        geometry_msgs::TwistStamped twist_stamped =
            p_sync_state_->twist_raw_.GetObj();
        if (p_sync_state_->autorun_request_.GetObj() &&
            !twist_stamped.header.stamp.is_zero() &&
            !p_sync_state_->stop_request_.GetObj()) {
          p_pubs_->cmd_vel_pub_.publish(twist_stamped.twist);
        } else {
          p_pubs_->cmd_vel_pub_.publish(CREATE_ZERO_TWIST());
        }
      }
    }

    // X. Velocity routing.
    {
      nav_msgs::Odometry odom = p_sync_state_->odom_.GetObj();
      if (!odom.header.stamp.is_zero()) {
        geometry_msgs::TwistStamped twist_stamped;
        twist_stamped.header.stamp = time;
        twist_stamped.twist = odom.twist.twist;
        p_pubs_->cur_vel_pub_.publish(twist_stamped);

        if (TwistIsBelowThreshold(twist_stamped.twist,
                                  params_.standstill_vx_thr,
                                  params_.standstill_wx_thr)) {
          p_sync_state_->standstill_cnt_.SetObj(
              p_sync_state_->standstill_cnt_.GetObj() + 1);
        } else {
          p_sync_state_->standstill_cnt_.SetObj(0);
        }
      }
    }

    // X. Check pose validity.
    if (!p_sync_state_->pose_initializing_.GetObj()) {
      CheckPoseValidity();
    }

    // X. Current pose.
    CreateAndPublishCurrentPose(time);

    r.sleep();
  }
}

void StatusManagementNodelet::RunCheckingCycle() {
  ros::Rate r(params_.status_check_freq);
  while (ros::ok()) {
    p_sync_state_->checking_cycle_cnt_.SetObj(
        p_sync_state_->checking_cycle_cnt_.GetObj() + 1);
    // X. Robot stuck reason.
    CheckRobotStopReason();

    // X. Location status.
    RunPoseInitializerIfNecessary();

    // X. Check if reached goal.
    if (CheckIfEndIsReached() && p_course_conf_mgmt_->NextConfig()) {
      std_msgs::Bool msg;
      msg.data = false;
      p_pubs_->status_mgmt_status_pub_.publish(msg);
      autorun_config conf = p_course_conf_mgmt_->GetCurrentConfig();
      CallMapLoadService(nh_, conf.file_conf_.map_pcd_file_);
      if (!params_.localize_only) {
        CallWaypointLoadService(nh_, conf.file_conf_.lane_csv_file_);
        CallWorld2MapLoadService(nh_, conf.file_conf_.world_to_map_json_file_);
      }
      InitializePose(conf.init_conf_);
    }

    if (!p_sync_state_->engage_request_.GetObj() ||
        (p_sync_state_->engage_request_.GetObj() &&
         !p_sync_state_->autorun_request_.GetObj())) {
      PublishStatusMessage("System is ready. Wait user request.",
                           DEFAULT_MESSAGE_CYCLE);
    }

    r.sleep();
  }
}

bool StatusManagementNodelet::CheckIfEndIsReached() {
  autoware_msgs::Lane base_wps = p_sync_state_->base_wps_.GetObj();
  std_msgs::Int32 cur_idx = p_sync_state_->cls_wp_idx_.GetObj();
  nav_msgs::Odometry odom = p_sync_state_->odom_.GetObj();

  bool end_is_reached = false;
  static int count = 0;

  static int current_idx = cur_idx.data;
  if (0 <= cur_idx.data) {
    current_idx = cur_idx.data;
  }

  LOG(INFO) << "Index diff : " << base_wps.waypoints.size() - current_idx;
  const int DIFF_THRESH = 10;
  if (0 < base_wps.waypoints.size() &&
      static_cast<int>(base_wps.waypoints.size()) - current_idx < DIFF_THRESH &&
      TwistIsBelowThreshold(odom.twist.twist, params_.standstill_vx_thr,
                            params_.standstill_wx_thr)) {
    end_is_reached = true;
    PublishStatusMessage(
        "Reached stop waypoint. Please provide next waypoints to follow.",
        DEFAULT_MESSAGE_CYCLE);
    count++;
  } else {
    count = 0;
  }

  return end_is_reached;
}

void StatusManagementNodelet::CreateAndPublishCurrentPose(
    const ros::Time &time) {
  nav_msgs::Odometry odom = p_sync_state_->odom_.GetObj();
  if (!odom.header.stamp.is_zero()) {
    geometry_msgs::PoseStamped cur_pose;
    cur_pose.header.stamp = time;
    cur_pose.header.frame_id = "map";

    static ros::Time last_pose_init;
    static int ndt_unreliable_cnt = 0;
    geometry_msgs::PoseStamped ndt_pose = p_sync_state_->ndt_pose_.GetObj();

    //
    // ROS_WARN("Localization Unreliable Cnt : %d",
    //         p_sync_state_->localization_unreliable_cnt_.GetObj());
    if (!ndt_pose.header.stamp.is_zero() &&
        p_sync_state_->localization_unreliable_cnt_.GetObj() == 0 &&
        !p_sync_state_->pose_initializing_.GetObj()) {
      // X. Use ndt pose as cur pose.
      cur_pose.pose = ndt_pose.pose;
      tf::Transform trans_odom_to_map =
          CreateTransformFromPose(cur_pose.pose, odom.pose.pose);

      // X. Send tf & cur pose
      // ROS_WARN("Publish transform from ndt.");
      p_pubs_->tf_broadcaster_.sendTransform(
          tf::StampedTransform(trans_odom_to_map, time, "map", "odom"));

      // ROS_WARN("Before cur pose pub");
      p_pubs_->cur_pose_pub_.publish(cur_pose);
      p_sync_state_->last_cur_pose_.SetObj(cur_pose);

      ndt_unreliable_cnt = 0;

    } else {
      // X. Use odometry as cur pose.
      tf::StampedTransform transform_st =
          p_sync_state_->last_valid_tf_odom_to_map_.GetObj();
      if (!transform_st.stamp_.is_zero()) {
        // X. Send transform.
        // ROS_WARN("Publish transform from odom.");
        transform_st.stamp_ = time;
        p_pubs_->tf_broadcaster_.sendTransform(transform_st);

        // X. Transform.
        cur_pose.pose = TransformPose(transform_st, odom.pose.pose);

        // X.
        // ROS_WARN("Before cur pose pub");
        p_pubs_->cur_pose_pub_.publish(cur_pose);
        p_sync_state_->last_cur_pose_.SetObj(cur_pose);
        PublishStatusMessage("Ndt pose is unreliable", DEFAULT_MESSAGE_CYCLE);

        double trans_diff, angle_diff;
        geometry_msgs::PoseStamped ndt_pose = p_sync_state_->ndt_pose_.GetObj();
        geometry_msgs::PoseWithCovarianceStamped amcl_pose =
            p_sync_state_->amcl_pose_.GetObj();
        ComputePoseDiff(ndt_pose.pose, amcl_pose.pose.pose, trans_diff,
                        angle_diff);
        // ROS_WARN("Trans Diff : %lf, Angle Diff : %lf", trans_diff,
        // angle_diff);

        if (!p_sync_state_->pose_initializing_.GetObj() &&
            (ros::Time::now() - last_pose_init).toSec() > 4.0 &&
            ndt_unreliable_cnt % static_cast<int>(DEFAULT_CONTROL_CYCLE_FREQ) ==
                0 /*&&
            !PoseDiffIsBelowThreshold(ndt_pose.pose, amcl_pose.pose.pose,
                                      params_.localization_translation_thresh,
                                      params_.localization_orientation_thresh)*/) {
          ROS_WARN("Pose init called.");
          last_pose_init = ros::Time::now();

          // Init ndt matching.
          XYZRPY init_pose = CreateXYZRPYFromPose(cur_pose.pose);
          p_pubs_->ndt_config_pub_.publish(CONFIG_DEFAULT_NDT(init_pose));

          // Init amcl localizer.
          geometry_msgs::PoseWithCovarianceStamped init_pose_amcl;
          init_pose_amcl.header.stamp = cur_pose.header.stamp;
          init_pose_amcl.header.frame_id = "map";
          init_pose_amcl.pose.pose = cur_pose.pose;
          p_pubs_->mcl_3dl_init_pub_.publish(init_pose_amcl);
        }
        ndt_unreliable_cnt = ndt_unreliable_cnt + 1;
      }
    }
  }
}

void StatusManagementNodelet::CheckRobotStopReason() {
  int32_t obs_idx = p_sync_state_->obst_wp_idx_.GetObj().data;

  // X. Robot is not stopped.
  // ROS_WARN("Obstacle Idx : %d", obs_idx);
  if (obs_idx == -1) {
    p_sync_state_->stopped_due_to_obstacle_cnt_.SetObj(0);
    return;
  }

  // X.
  // ROS_WARN("Twist is almost zero");
  geometry_msgs::TwistStamped twist_raw = p_sync_state_->twist_raw_.GetObj();
  if (twist_raw.header.stamp.is_zero() ||
      !TwistIsBelowThreshold(twist_raw.twist, params_.standstill_vx_thr,
                             params_.standstill_wx_thr)) {
    p_sync_state_->stopped_due_to_obstacle_cnt_.SetObj(0);
    return;
  }

  // X.
  autoware_msgs::Lane fin_wps = p_sync_state_->fin_wps_.GetObj();
  geometry_msgs::PoseStamped last_cur_pose =
      p_sync_state_->last_cur_pose_.GetObj();

  // ROS_WARN("Waypoint time : %lf", fin_wps.header.stamp.toSec());
  // ROS_WARN("Last cur pose : %lf", last_cur_pose.header.stamp.toSec());
  if (fin_wps.waypoints.size() == 0 || last_cur_pose.header.stamp.is_zero()) {
    p_sync_state_->stopped_due_to_obstacle_cnt_.SetObj(0);
    return;
  }

  // X.
  geometry_msgs::PoseStamped cur_pose = p_sync_state_->last_cur_pose_.GetObj();

  ROS_WARN("Distance : %lf", ComputeDistanceToObstacleOnWaypoint(
                                 obs_idx, fin_wps, cur_pose.pose));
  if (ComputeDistanceToObstacleOnWaypoint(obs_idx, fin_wps, cur_pose.pose) <
      params_.obstacle_distance_thresh) {
    // ROS_WARN("Robot stop due to obstacle detected.");

    // X. Count up.
    p_sync_state_->stopped_due_to_obstacle_cnt_.SetObj(
        p_sync_state_->stopped_due_to_obstacle_cnt_.GetObj() + 1);
    // X. Status announce.
    int stopped_cnt = p_sync_state_->stopped_due_to_obstacle_cnt_.GetObj();
    PublishStatusMessage("Robot stops due to obstacle in front.",
                         DEFAULT_MESSAGE_CYCLE);

    // X. Wait for avoidance.
    if (IsShortWaitAvoidanceWaypoint(obs_idx, fin_wps)) {
      if (params_.obstacle_short_wait_before_avoid <
          stopped_cnt / params_.status_check_freq) {
        PublishStatusMessage("Rerouting for obstacle avoidance.",
                             DEFAULT_MESSAGE_CYCLE);
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        p_pubs_->onetime_avoidance_req_pub_.publish(header);
        p_sync_state_->stopped_due_to_obstacle_cnt_.SetObj(0);
      }
    } else if (IsLongWaitAvoidanceWaypoint(obs_idx, fin_wps)) {
      if (params_.obstacle_long_wait_before_avoid <
          stopped_cnt / params_.status_check_freq) {
        PublishStatusMessage("Rerouting for obstacle avoidance.",
                             DEFAULT_MESSAGE_CYCLE);
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        p_pubs_->onetime_avoidance_req_pub_.publish(header);
        p_sync_state_->stopped_due_to_obstacle_cnt_.SetObj(0);
      }
    } else {
      PublishStatusMessage("Rerouting is not allowed here. Keep waiting.",
                           DEFAULT_MESSAGE_CYCLE);
      p_sync_state_->stopped_due_to_obstacle_cnt_.SetObj(0);
    }
  }
}

void StatusManagementNodelet::CheckPoseValidity() {
  geometry_msgs::PoseStamped ndt_pose = p_sync_state_->ndt_pose_.GetObj();
  geometry_msgs::PoseWithCovarianceStamped amcl_pose =
      p_sync_state_->amcl_pose_.GetObj();

  // X. During pose initialization, skip.
  if (p_sync_state_->pose_initializing_.GetObj() ||
      ndt_pose.header.stamp.is_zero() || amcl_pose.header.stamp.is_zero()) {
    return;
  }

  // X. Map pose is not stable.
  if (false /*!PoseDiffIsBelowThreshold(ndt_pose.pose, amcl_pose.pose.pose,
                                params_.localization_translation_thresh,
                                params_.localization_orientation_thresh)*/) {
    p_sync_state_->localization_unreliable_cnt_.SetObj(
        std::min(p_sync_state_->localization_unreliable_cnt_.GetObj() + 1,
                 params_.localization_unreliable_cnt_for_reinit));

  } else {
    // X. When map pose is stable.
    if (true/*PoseDiffIsBelowThreshold(ndt_pose.pose, amcl_pose.pose.pose,
                                 params_.localization_translation_reliable_thresh,
                                 params_.localization_orientation_reliable_thresh_deg)*/) {
      try {
        tf::StampedTransform tf_odom_to_map;
        p_subs_->tf_listener_.lookupTransform(
            "map", "odom", ndt_pose.header.stamp, tf_odom_to_map);
        p_sync_state_->last_valid_ndt_pose_queue_.push_back(
            std::make_pair(ndt_pose, tf_odom_to_map));
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }

      // X. If reaches stable count, treat the first one as valid pose.
      if (p_sync_state_->last_valid_ndt_pose_queue_.size() ==
          params_.localization_reliable_count) {
        std::pair<geometry_msgs::PoseStamped, tf::StampedTransform> e =
            p_sync_state_->last_valid_ndt_pose_queue_.front();
        p_sync_state_->last_valid_ndt_pose_queue_.pop_front();
        p_sync_state_->last_valid_ndt_pose_.SetObj(e.first);
        p_sync_state_->last_valid_tf_odom_to_map_.SetObj(e.second);
      }

    } else {
      p_sync_state_->last_valid_ndt_pose_queue_.clear();
    }

    // X. Decrement unreliable count.
    p_sync_state_->localization_unreliable_cnt_.SetObj(
        std::max(0, p_sync_state_->localization_unreliable_cnt_.GetObj() - 1));
  }
}

void StatusManagementNodelet::RunPoseInitializerIfNecessary() {
  ros::Time last_pose_init_time = p_sync_state_->last_pose_init_time_.GetObj();
  if (last_pose_init_time.is_zero() ||
      ros::Time::now() - last_pose_init_time >
          ros::Duration(params_.localization_reinit_minimum_period)) {
    if (params_.localization_unreliable_cnt_for_reinit <=
        p_sync_state_->localization_unreliable_cnt_.GetObj()) {
      p_sync_state_->stop_request_.SetObj(true);

      // X. Make sure that robot stops.
      ros::Rate r(params_.robot_stop_check_freq);
      while (p_sync_state_->standstill_cnt_.GetObj() <
             params_.init_pose_standstill_count) {
        PublishStatusMessage("Robot will stop for pose initialization.",
                             DEFAULT_MESSAGE_CYCLE);
        if (p_sync_state_->localization_unreliable_cnt_.GetObj() <
            params_.localization_unreliable_cnt_for_reinit / 2) {
          ROS_WARN("Localization stabilized.");
          p_sync_state_->stop_request_.SetObj(false);
          return;
        }
        r.sleep();
      }

      // X. Choose initial position for search.
      XYZRPY searched_pose;
      p_sync_state_->pose_initializing_.SetObj(true);
      if (!p_sync_state_->last_valid_ndt_pose_.GetObj()
               .header.stamp.is_zero() &&
          ros::Time::now() -
                  p_sync_state_->last_valid_ndt_pose_.GetObj().header.stamp <
              ros::Duration(params_.localization_odom_trust_period) &&
          !params_.reinit_via_gnss) {
        PublishStatusMessage("Reinitialize localization based on odometry.",
                             DEFAULT_MESSAGE_CYCLE);
        searched_pose = TryInitializePose(
            nh_, p_pubs_->voxel_filt_pub_, *p_sync_state_,
            CreateXYZRPYFromPose(p_sync_state_->last_cur_pose_.GetObj().pose),
            false, true, DEFAULT_INIT_GNSS_CHECK_FREQ,
            params_.init_pose_srv_timeout);

      } else {
        PublishStatusMessage("Reinitialize localization based on GNSS.",
                             DEFAULT_MESSAGE_CYCLE);
        searched_pose = TryInitializePose(
            nh_, p_pubs_->voxel_filt_pub_, *p_sync_state_,
            XYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), true, true,
            DEFAULT_INIT_GNSS_CHECK_FREQ, params_.init_pose_srv_timeout);
      }

      // X. Update localizer.
      InitializeLocalizer(p_pubs_->ndt_config_pub_, p_pubs_->mcl_3dl_init_pub_,
                          searched_pose);

      // X. Status Announce.
      PublishStatusMessage("Pose initialization is done. Start control.",
                           DEFAULT_MESSAGE_CYCLE);

      // X. Memorize last update.
      p_sync_state_->last_pose_init_time_.SetObj(ros::Time::now());
      p_sync_state_->stop_request_.SetObj(false);
      p_sync_state_->pose_initializing_.SetObj(false);
      p_sync_state_->localization_unreliable_cnt_.SetObj(
          std::max(0, p_sync_state_->localization_unreliable_cnt_.GetObj() -
                          params_.localization_unreliable_cnt_for_reinit / 2));
    }
  }
}

}  // namespace koichi_robotics_lib

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(koichi_robotics_lib::StatusManagementNodelet,
                       nodelet::Nodelet)
