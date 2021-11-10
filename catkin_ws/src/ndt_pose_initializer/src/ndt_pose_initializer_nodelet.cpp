
// Original
#include <messages/initialize_pose.h>
#include <sensor_msgs/PointCloud2.h>

// ROS
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/tf.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

// STL
#include <algorithm>
#include <cmath>
#include <mutex>
#include <numeric>
#include <thread>

// Glog
#include <glog/logging.h>

namespace {

double NormalizeAngleFrom_0_to_2pi(double angle) {
  double tmp_angle = angle;
  while (true) {
    if (tmp_angle < 0) {
      tmp_angle += (2 * M_PI);
    } else if (2 * M_PI <= tmp_angle) {
      tmp_angle -= (2 * M_PI);
    } else {
      break;
    }
  }
  return tmp_angle;
}

struct Range {
 public:
  Range(double x_range, double y_range, double yaw_range, double x_step,
        double y_step, double yaw_step)
      : x_range_(x_range),
        y_range_(y_range),
        yaw_range_(yaw_range),
        x_step_(x_step),
        y_step_(y_step),
        yaw_step_(NormalizeAngleFrom_0_to_2pi(yaw_step)) {}

  double x_range_, y_range_, yaw_range_;
  double x_step_, y_step_, yaw_step_;
};

void CreatePoseCollections(const Range &range, const double x, const double y,
                           const double z, const double roll,
                           const double pitch, const double yaw,
                           const Eigen::Matrix4f &baselink_to_lidar,
                           std::vector<Eigen::Matrix4f> &pose_collections) {
  const double MINIMUM_STEP = 0.1;
  const double MINIMUM_DYAW = 1.0 / 180 * M_PI;

  double step_dx = std::max(range.x_step_, MINIMUM_STEP);
  double step_dy = std::max(range.y_step_, MINIMUM_STEP);
  double step_dyaw = std::max(range.yaw_step_, MINIMUM_DYAW);

  int num_x_step = range.x_range_ / step_dx + 1;
  int num_y_step = range.y_range_ / step_dy + 1;
  int num_yaw_step = range.yaw_range_ / step_dyaw + 1;

  int pose_num = num_x_step * num_y_step * num_yaw_step;
  pose_collections.clear();
  pose_collections.reserve(pose_num);

  ROS_WARN("Num pose num : %d", pose_num);
  ROS_WARN("Num x step   : %d", num_x_step);
  ROS_WARN("Num y step   : %d", num_y_step);
  ROS_WARN("Num yaw step : %d", num_yaw_step);
  ROS_WARN("Yaw Step     : %lf", range.yaw_step_);
  ROS_WARN("Range yaw    : %lf", range.yaw_range_);

  for (int idx_x = -(num_x_step / 2 + 1); idx_x < (num_x_step / 2 + 1);
       idx_x++) {
    for (int idx_y = -(num_y_step / 2 + 1); idx_y < (num_y_step / 2 + 1);
         idx_y++) {
      for (int idx_yaw = -(num_yaw_step / 2 + 1);
           idx_yaw < (num_yaw_step / 2 + 1); idx_yaw++) {
        Eigen::Translation3f trans;
        trans.x() = x + step_dx * idx_x;
        trans.y() = y + step_dy * idx_y;
        trans.z() = z;

        Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f test_pose =
            ((trans * rot_z * rot_y * rot_x) * baselink_to_lidar).matrix();
        pose_collections.push_back(test_pose);
      }
    }
  }
}

}  // namespace

namespace koichi_robotics_lib {

class NdtPoseInitializerNodelet : public nodelet::Nodelet {
 public:
  NdtPoseInitializerNodelet();

  ~NdtPoseInitializerNodelet();

  virtual void onInit();

  void Initialize();

  void ReadParams();

  bool ServeInitializePose(messages::initialize_poseRequest &req,
                           messages::initialize_poseResponse &res);

  void PointsMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void MapCheckingFunc();

  void LocalizingFunc(const int thread_idx,
                      const messages::initialize_poseRequest &req,
                      const std::vector<Eigen::Matrix4f> &pose_collections,
                      const int st_idx, const int end_idx,
                      std::vector<Eigen::Matrix4f> &final_pose_collections,
                      std::vector<double> &fitness_scores);

  void FilteredPointsCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

 private:
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer server_;
  ros::Subscriber points_sub_;

  // Map data.
  pcl::PointCloud<pcl::PointXYZ> map_data_;
  int points_in_map_;

  // Scan data.
  pcl::PointCloud<pcl::PointXYZ> scan_data_;

  // Transform
  Eigen::Matrix4f tf_baselink_to_lidar_;

  // Status
  bool map_received_, scan_received_;

  // Thread
  std::thread st_thread_, map_check_thread_;
  std::mutex map_mtx_, scan_mtx_;

  static const int DEFAULT_SUBSCRIBE_QUEUE = 10;
};

NdtPoseInitializerNodelet::NdtPoseInitializerNodelet()
    : nh_(),
      pnh_(),
      server_(),
      points_sub_(),
      map_data_(),
      points_in_map_(0),
      scan_data_(),
      tf_baselink_to_lidar_(),
      map_received_(false),
      scan_received_(false),
      st_thread_(),
      map_check_thread_(),
      map_mtx_(),
      scan_mtx_() {}

NdtPoseInitializerNodelet::~NdtPoseInitializerNodelet() {}

void NdtPoseInitializerNodelet::onInit() {
  st_thread_ = std::thread(&NdtPoseInitializerNodelet::Initialize, this);
}

void NdtPoseInitializerNodelet::Initialize() {
  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();

  ReadParams();

  map_check_thread_ =
      std::thread(&NdtPoseInitializerNodelet::MapCheckingFunc, this);

  points_sub_ =
      nh_.subscribe("filtered_points", DEFAULT_SUBSCRIBE_QUEUE,
                    &NdtPoseInitializerNodelet::FilteredPointsCallback, this);

  ros::Rate loop_rate(5);
  while (!(map_received_ && scan_received_)) {
    ROS_WARN(
        "Waiting /points_map and /filtered_scan to start /initialize_pose "
        "service");
    loop_rate.sleep();
  }
  server_ = nh_.advertiseService(
      "initialize_pose", &NdtPoseInitializerNodelet::ServeInitializePose, this);
}

void NdtPoseInitializerNodelet::ReadParams() {
  // X. Load global paramers.
  double tf_x, tf_y, tf_z;
  double tf_roll, tf_pitch, tf_yaw;
  CHECK(nh_.getParam("tf_x", tf_x)) << "Parameter tf_x cannot be read.";
  CHECK(nh_.getParam("tf_y", tf_y)) << "Parameter tf_y cannot be read.";
  CHECK(nh_.getParam("tf_z", tf_z)) << "Parameter tf_z cannot be read.";
  CHECK(nh_.getParam("tf_roll", tf_roll))
      << "Parameter tf_roll cannot be read.";
  CHECK(nh_.getParam("tf_pitch", tf_pitch))
      << "Parameter tf_pitch cannot be read.";
  CHECK(nh_.getParam("tf_yaw", tf_yaw)) << "Parameter tf_yaw cannot be read.";

  Eigen::Quaternionf rot_baselink_to_lidar =
      Eigen::AngleAxisf(tf_yaw, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(tf_pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(tf_roll, Eigen::Vector3f::UnitX());

  Eigen::Translation3f trans_baselink_to_lidar(tf_x, tf_y, tf_z);
  tf_baselink_to_lidar_ =
      (trans_baselink_to_lidar * rot_baselink_to_lidar).matrix();
}

bool NdtPoseInitializerNodelet::ServeInitializePose(
    messages::initialize_poseRequest &req,
    messages::initialize_poseResponse &res) {
  ROS_WARN("[Pose Initializer] Service called");

  // X. Create pose collections.
  std::vector<Eigen::Matrix4f> pose_collections;
  CreatePoseCollections(Range(req.x_range, req.y_range, req.yaw_range,
                              req.x_step, req.y_step, req.yaw_step),
                        req.x, req.y, req.z, req.roll, req.pitch, req.yaw,
                        tf_baselink_to_lidar_, pose_collections);
  std::vector<Eigen::Matrix4f> final_pose_collections(
      pose_collections.size(), Eigen::Matrix4f::Identity());
  std::vector<double> fitness_scores(pose_collections.size(),
                                     std::numeric_limits<double>::max());

  // X. Parameter setup.
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setResolution(req.resolution);
  ndt.setStepSize(req.step_size);
  ndt.setOulierRatio(req.outlier_ratio);
  ndt.setTransformationEpsilon(req.trans_eps);
  ndt.setMaximumIterations(req.max_itr);

  // X. Get current scan data.
  pcl::PointCloud<pcl::PointXYZ> scan_cloud;
  {
    std::lock_guard<std::mutex> lock(scan_mtx_);
    scan_cloud = scan_data_;
  }

  {
    std::lock_guard<std::mutex> lock(map_mtx_);

    // X. Set target source.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(map_data_));
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(scan_cloud));
    ndt.setInputTarget(target_ptr);
    ndt.setInputSource(source_ptr);

    // X. Align.
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(
          new pcl::PointCloud<pcl::PointXYZ>);

      int cnt = 0;
      int size = pose_collections.size();
      for (int idx = 0; idx < size; idx++) {
        ndt.align(*output_ptr, pose_collections[idx]);
        // X. Collect result and stats.
        bool converged = ndt.hasConverged();

        if (converged) {
          final_pose_collections[idx] = ndt.getFinalTransformation();
          fitness_scores[idx] = ndt.getFitnessScore();
          int num_itr = ndt.getFinalNumIteration();
          double trans_probability = ndt.getTransformationProbability();
        }

        cnt++;
        if (cnt % 100 == 0) {
          ROS_WARN("[Pose Initializer] Current Status : %d : %d", cnt, size);
        }
      }
    }
  }

  // X. Extract final result
  {
    std::vector<size_t> indices(fitness_scores.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::stable_sort(indices.begin(), indices.end(),
                     [&fitness_scores](size_t idx1, size_t idx2) {
                       return fitness_scores[idx1] < fitness_scores[idx2];
                     });

    Eigen::Matrix4f pose =
        final_pose_collections[indices[0]] * tf_baselink_to_lidar_.inverse();

    res.x = pose(0, 3);
    res.y = pose(1, 3);
    res.z = pose(2, 3);

    tf::Matrix3x3 rot;
    rot.setValue(
        static_cast<double>(pose(0, 0)), static_cast<double>(pose(0, 1)),
        static_cast<double>(pose(0, 2)), static_cast<double>(pose(1, 0)),
        static_cast<double>(pose(1, 1)), static_cast<double>(pose(1, 2)),
        static_cast<double>(pose(2, 0)), static_cast<double>(pose(2, 1)),
        static_cast<double>(pose(2, 2)));
    double roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);
    res.roll = roll;
    res.pitch = pitch;
    res.yaw = yaw;
    res.fitness_score = fitness_scores[indices[0]];

    ROS_WARN("x     : %lf", res.x);
    ROS_WARN("y     : %lf", res.y);
    ROS_WARN("z     : %lf", res.z);
    ROS_WARN("roll  : %lf", res.roll);
    ROS_WARN("pitch : %lf", res.pitch);
    ROS_WARN("yaw   : %lf", res.yaw);
  }

  return true;
}

void NdtPoseInitializerNodelet::LocalizingFunc(
    const int thread_idx, const messages::initialize_poseRequest &req,
    const std::vector<Eigen::Matrix4f> &pose_collections, const int st_idx,
    const int end_idx, std::vector<Eigen::Matrix4f> &final_pose_collections,
    std::vector<double> &fitness_scores) {
  // X. Prep
  int num_indices = end_idx - st_idx + 1;
  final_pose_collections.clear();
  final_pose_collections.resize(num_indices, Eigen::Matrix4f::Identity());
  fitness_scores.clear();
  fitness_scores.resize(num_indices, std::numeric_limits<double>::max());

  // X. Parameter setup.
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setResolution(req.resolution);
  ndt.setStepSize(req.step_size);
  ndt.setOulierRatio(req.outlier_ratio);
  ndt.setTransformationEpsilon(req.trans_eps);
  ndt.setMaximumIterations(req.max_itr);

  // X. Get current scan data.
  pcl::PointCloud<pcl::PointXYZ> scan_cloud;
  {
    std::lock_guard<std::mutex> lock(scan_mtx_);
    scan_cloud = scan_data_;
  }

  {
    std::lock_guard<std::mutex> lock(map_mtx_);

    // X. Set target source.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(map_data_));
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(scan_cloud));
    ndt.setInputTarget(target_ptr);
    ndt.setInputSource(source_ptr);

    // X. Align.
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(
          new pcl::PointCloud<pcl::PointXYZ>);

      int current_cnt = 0;
      for (int idx = st_idx; idx < end_idx + 1; idx++) {
        ndt.align(*output_ptr, pose_collections[idx]);
        // X. Collect result and stats.
        bool converged = ndt.hasConverged();
        if (converged) {
          int shifted_idx = idx - st_idx;
          final_pose_collections[shifted_idx] = ndt.getFinalTransformation();
          fitness_scores[shifted_idx] = ndt.getFitnessScore();
          // int num_itr = ndt.getFinalNumIteration();
          // double trans_probability = ndt.getTransformationProbability();
        }

        current_cnt++;
        if (current_cnt % 100 == 0) {
          ROS_WARN("[Pose Initializer] Thread Idx %d, Current Status : %d / %d",
                   thread_idx, current_cnt, num_indices);
        }
      }
    }
  }
}

void NdtPoseInitializerNodelet::PointsMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (points_in_map_ != msg->width) {
    points_in_map_ = msg->width;
    {
      std::lock_guard<std::mutex> lock(map_mtx_);
      pcl::fromROSMsg(*msg, map_data_);
    }
    map_received_ = true;
  }
}

void NdtPoseInitializerNodelet::MapCheckingFunc() {
  ros::NodeHandle map_nh;
  ros::CallbackQueue map_callback_queue;
  map_nh.setCallbackQueue(&map_callback_queue);

  ros::Subscriber map_sub =
      map_nh.subscribe("points_map", DEFAULT_SUBSCRIBE_QUEUE,
                       &NdtPoseInitializerNodelet::PointsMapCallback, this);
  ros::Rate loop_rate(5);
  while (nh_.ok()) {
    map_callback_queue.callAvailable(ros::WallDuration());
    loop_rate.sleep();
  }
  return;
}

void NdtPoseInitializerNodelet::FilteredPointsCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> lock(scan_mtx_);
    pcl::fromROSMsg(*msg, scan_data_);
  }
  scan_received_ = true;
}

}  // namespace koichi_robotics_lib

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(koichi_robotics_lib::NdtPoseInitializerNodelet,
                       nodelet::Nodelet)