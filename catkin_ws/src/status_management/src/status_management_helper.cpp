
#include <status_management/status_management_helper.h>

// ROS
#include <ros/ros.h>

// Google
#include <glog/logging.h>

namespace koichi_robotics_lib {

/**
 * Implementation of CourseConfigMgmt
 */
CourseConfigMgmt::CourseConfigMgmt(const std::string &course_yaml_path,
                                   const unsigned int start_idx) {
  std::map<unsigned int, file_config> file_confs =
      YamlConfigReader::ReadFileConfig(course_yaml_path);

  for (const auto &p : file_confs) {
    autorun_config auto_conf;
    auto_conf.init_conf_ =
        YamlConfigReader::ReadInitConfig(p.second.course_start_yaml_file_);
    auto_conf.file_conf_ = p.second;
    course_configs_.insert(std::make_pair(p.first, auto_conf));
  }

  cur_itr_ = course_configs_.lower_bound(start_idx);
  CHECK(cur_itr_ != course_configs_.cend());
  last_update_time_ = ros::Time::now();
}

autorun_config CourseConfigMgmt::GetCurrentConfig() { return cur_itr_->second; }

bool CourseConfigMgmt::NextConfig(uint32_t min_time_from_last_call) {
  ros::Duration min_time_for_update(min_time_from_last_call);
  ros::Time cur_time = ros::Time::now();
  if (min_time_for_update < cur_time - last_update_time_) {
    cur_itr_++;
    last_update_time_ = cur_time;
    LOG(INFO) << "Course scenario updated. Scenario IDX : " << cur_itr_->first;
  } else {
    LOG(WARNING)
        << "Duration between NextConfig call is too short. Last call : "
        << last_update_time_ << ", Cur call : " << cur_time;
  }
  return cur_itr_ != course_configs_.cend();
}

StatusManagementNodeletParams ReadStatusManagementNodeletParams(
    ros::NodeHandle &pnh) {
  StatusManagementNodeletParams params;
  CHECK(pnh.getParam("course_config_yamlpath", params.course_config_yaml))
      << "[StatusManagementNodelet] Parameter init_from_gnss cannot be read.";

  CHECK(pnh.getParam("start_course_idx", params.start_course_idx))
      << "[StatusManagementNodelet] Parameter start_course_idx cannot be read.";

  CHECK(pnh.getParam("reinit_via_gnss", params.reinit_via_gnss))
      << "[StatusManagementNodelet] Parameter reinit_via_gnss cannot be read.";

  CHECK(pnh.getParam("init_pose_srv_timeout", params.init_pose_srv_timeout))
      << "[StatusManagementNodelet] Parameter init_pose_srv_timeout cannot be "
         "read.";

  CHECK(pnh.getParam("status_check_freq", params.status_check_freq))
      << "[StatusManagementNodelet] Parameter status_check_freq cannot be "
         "read.";
  CHECK(pnh.getParam("robot_stop_check_freq", params.robot_stop_check_freq))
      << "[StatusManagementNodelet] Parameter robot_stop_check_freq cannot be "
         "read.";

  CHECK(pnh.getParam("standstill_vx_thr", params.standstill_vx_thr))
      << "[StatusManagementNodelet] Parameter standstill_vx_thr cannot be "
         "read.";
  CHECK(pnh.getParam("standstill_wx_thr", params.standstill_wx_thr))
      << "[StatusManagementNodelet] Parameter standstill_wx_thr cannot be "
         "read.";
  CHECK(pnh.getParam("localization_reinit_minimum_period",
                     params.localization_reinit_minimum_period))
      << "[StatusManagementNodelet] Parameter "
         "localization_reinit_minimum_period cannot be read.";
  CHECK(pnh.getParam("localization_odom_trust_period",
                     params.localization_odom_trust_period))
      << "[StatusManagementNodelet] Parameter localization_odom_trust_period "
         "cannot be read.";
  CHECK(pnh.getParam("localization_unreliable_cnt_for_reinit",
                     params.localization_unreliable_cnt_for_reinit))
      << "[StatusManagementNodelet] Parameter "
         "localization_unreliable_cnt_for_reinit cannot be read.";
  CHECK(pnh.getParam("localization_reliable_count",
                     params.localization_reliable_count))
      << "[StatusManagementNodelet] Parameter localization_reliable_count "
         "cannot be read.";
  CHECK(pnh.getParam("init_pose_standstill_count",
                     params.init_pose_standstill_count))
      << "[StatusManagementNodelet] Parameter init_pose_standstill_count "
         "cannot be read.";

  CHECK(pnh.getParam("localization_translation_thresh",
                     params.localization_translation_thresh))
      << "[StatusManagementNodelet] Parameter localization_translation_thresh "
         "cannot be read.";
  CHECK(pnh.getParam("localization_orientation_thresh_deg",
                     params.localization_orientation_thresh))
      << "[StatusManagementNodelet] Parameter localization_orientation_thresh "
         "cannot be read.";
  params.localization_orientation_thresh =
      params.localization_orientation_thresh / 180.0 * M_PI;

  CHECK(pnh.getParam("localization_translation_reliable_thresh",
                     params.localization_translation_reliable_thresh))
      << "[StatusManagementNodelet] Parameter "
         "localization_translation_reliable_thresh cannot be read.";
  CHECK(pnh.getParam("localization_orientation_reliable_thresh_deg",
                     params.localization_orientation_reliable_thresh))
      << "[StatusManagementNodelet] Parameter "
         "localization_orientation_reliable_thresh_deg cannot be read.";
  params.localization_orientation_reliable_thresh =
      params.localization_orientation_reliable_thresh / 180.0 * M_PI;

  CHECK(
      pnh.getParam("obstacle_distance_thresh", params.obstacle_distance_thresh))
      << "[StatusManagementNodelet] Parameter obstacle_distance_thresh cannot "
         "be read.";
  CHECK(pnh.getParam("obstacle_short_wait_before_avoid",
                     params.obstacle_short_wait_before_avoid))
      << "[StatusManagementNodelet] Parameter obstacle_short_wait_before_avoid "
         "cannot be read.";
  CHECK(pnh.getParam("obstacle_long_wait_before_avoid",
                     params.obstacle_long_wait_before_avoid))
      << "[StatusManagementNodelet] Parameter obstacle_long_wait_before_avoid "
         "cannot be read.";

  return params;
}

void CallMapLoadService(ros::NodeHandle &nh,
                        const std::string &map_pcd_filepath) {
  const int SERVICE_TIMEOUT = 10;
  {
    ros::ServiceClient client =
        nh.serviceClient<autoware_msgs::String>("points_map_load");
    if (!client.waitForExistence(ros::Duration(SERVICE_TIMEOUT))) {
      LOG(FATAL) << "Failed to wait for service : points_map_load";
    }

    autoware_msgs::String srv;
    srv.request.str = map_pcd_filepath;
    if (!client.call(srv)) {
      LOG(FATAL) << "Failed to call service : points_map_load";
    }
  }
}

void CallWaypointLoadService(ros::NodeHandle &nh,
                             const std::string &waypoint_csv_filepath) {
  // X. Call map loader
  const int SERVICE_TIMEOUT = 10;
  {
    ros::ServiceClient client =
        nh.serviceClient<autoware_msgs::String>("lane_load_from_csv");
    if (!client.waitForExistence(ros::Duration(SERVICE_TIMEOUT))) {
      LOG(FATAL) << "Failed to wait for service : lane_load_from_csv";
    }

    autoware_msgs::String srv;
    srv.request.str = waypoint_csv_filepath;
    if (!client.call(srv)) {
      LOG(FATAL) << "Failed to call service : lane_load_from_csv";
    }
  }
}

void CallWorld2MapLoadService(ros::NodeHandle &nh,
                              const std::string &world_to_map_json_filepath) {
  const int SERVICE_TIMEOUT = 10;
  {
    ros::ServiceClient client =
        nh.serviceClient<autoware_msgs::String>("load_world_to_map_json");
    if (!client.waitForExistence(ros::Duration(SERVICE_TIMEOUT))) {
      LOG(FATAL) << "Failed to wait for service : load_world_to_map_json";
    }

    autoware_msgs::String srv;
    srv.request.str = world_to_map_json_filepath;
    if (!client.call(srv)) {
      LOG(FATAL) << "Failed to call service : load_world_to_map_json";
    }
  }
}

bool InitializePoseViaGNSS(SyncState &sync_state, XYZRPY &init_pose,
                           const double gnss_check_freq) {
  // X. Wait update from GNSS.
  ros::Rate r(gnss_check_freq);
  while (sync_state.gnss_local_.GetObj().header.stamp.is_zero() && ros::ok()) {
    ROS_WARN("Waiting for GNSS data to be received.");
    r.sleep();
  }

  // X. Update init pose.
  init_pose = CreateXYZRPYFromPose(sync_state.gnss_local_.GetObj().pose);
  init_pose.roll_ = 0.0;
  init_pose.pitch_ = 0.0;
  init_pose.yaw_ = 0.0;

  return true;
}

bool CallPoseInitializeService(ros::NodeHandle &nh,
                               ros::Publisher &voxel_filt_pub,
                               XYZRPY &init_pose,
                               const double init_pose_srv_timeout) {
  voxel_filt_pub.publish(CONFIG_VOXELFILT_POSE_SEARCH());
  messages::initialize_pose srv = POSE_INIT_REQUEST_FULL(init_pose);
  ros::ServiceClient client =
      nh.serviceClient<messages::initialize_pose>("/initialize_pose");
  if (!client.waitForExistence(ros::Duration(init_pose_srv_timeout))) {
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

  // X. Set finer resoluiton to voxelfilter.
  voxel_filt_pub.publish(CONFIG_VOXELFILT_POSE_RUN());

  return true;
}

XYZRPY TryInitializePose(ros::NodeHandle &nh, ros::Publisher &voxel_filt_pub,
                         SyncState &sync_state, const XYZRPY &base_pose,
                         const bool via_gnss, const bool pose_init,
                         const double gnss_check_freq,
                         const double init_pose_srv_timeout) {
  XYZRPY init_pose = base_pose;
  if (via_gnss) {
    InitializePoseViaGNSS(sync_state, init_pose, gnss_check_freq);
  }

  if (pose_init) {
    CallPoseInitializeService(nh, voxel_filt_pub, init_pose,
                              init_pose_srv_timeout);
  }
  return init_pose;
}

bool InitializeLocalizer(ros::Publisher &ndt_config_pub,
                         ros::Publisher &mcl_3dl_init_pub,
                         const XYZRPY &init_pose) {
  // X. Initialize ndt localizer.
  ndt_config_pub.publish(CONFIG_DEFAULT_NDT(init_pose));

  // X. Initialize amcl localizer.
  mcl_3dl_init_pub.publish(
      CreatePoseWithCovarianceStamped(init_pose, ros::Time::now()));

  return true;
}

bool CreateInitPoseFromConfig(const init_config &conf, SyncState &sync_state,
                              XYZRPY &init_pose, const double odom_wait_freq) {
  if (!conf.pose_initializer_) {
    ros::Rate r(odom_wait_freq);
    while (ros::ok() && sync_state.odom_.GetObj().header.stamp.is_zero()) {
      r.sleep();
    }
    nav_msgs::Odometry odom = sync_state.odom_.GetObj();

    geometry_msgs::Pose init_pose_msg = CreatePoseFromXYZRPY(XYZRPY(
        conf.init_pose_.x_, conf.init_pose_.y_, conf.init_pose_.z_,
        conf.init_pose_.roll_, conf.init_pose_.pitch_, conf.init_pose_.yaw_));
    geometry_msgs::PoseStamped init_pose_stamped;
    ros::Time cur_time = ros::Time::now();
    init_pose_stamped.header.stamp = cur_time;
    init_pose_stamped.header.frame_id = "map";
    init_pose_stamped.pose = init_pose_msg;
    sync_state.last_valid_ndt_pose_.SetObj(init_pose_stamped);

    tf::Transform trans_odom_to_map =
        CreateTransformFromPose(init_pose_msg, odom.pose.pose);

    sync_state.last_valid_tf_odom_to_map_.SetObj(
        tf::StampedTransform(trans_odom_to_map, cur_time, "map", "odom"));
  }

  init_pose.x_ = conf.init_pose_.x_;
  init_pose.y_ = conf.init_pose_.y_;
  init_pose.z_ = conf.init_pose_.z_;
  init_pose.roll_ = conf.init_pose_.roll_;
  init_pose.pitch_ = conf.init_pose_.pitch_;
  init_pose.yaw_ = conf.init_pose_.yaw_;
}

}  // namespace koichi_robotics_lib