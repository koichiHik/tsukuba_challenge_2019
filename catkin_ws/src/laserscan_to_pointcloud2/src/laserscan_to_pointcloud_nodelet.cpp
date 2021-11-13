
// ROS
#include <laser_geometry/laser_geometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace koichi_robotics_lib {

class LasaerScanToPointCloudNodelet : public nodelet::Nodelet {
 public:
  LasaerScanToPointCloudNodelet();

  ~LasaerScanToPointCloudNodelet();

  virtual void onInit();

 private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

 private:
  ros::NodeHandle nh_, pnh_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tf_listener_;
  ros::Publisher pcl_pub_;
  ros::Subscriber scan_sub_;

  static const int DEFAULT_PUBLISH_QUEUE_SIZE = 100;
  static const int DEFAULT_SUBSCRIBE_QUEUE_SIZE = 100;
};

LasaerScanToPointCloudNodelet::LasaerScanToPointCloudNodelet() {}

LasaerScanToPointCloudNodelet::~LasaerScanToPointCloudNodelet() {}

void LasaerScanToPointCloudNodelet::onInit() {
  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();

  // X. Read parameters.

  // X. Prepare pub.
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "pointcloud", DEFAULT_PUBLISH_QUEUE_SIZE, false);

  // X. Prepare sub.
  scan_sub_ = nh_.subscribe("scan", DEFAULT_SUBSCRIBE_QUEUE_SIZE,
                            &LasaerScanToPointCloudNodelet::scanCallback, this);
}

void LasaerScanToPointCloudNodelet::scanCallback(
    const sensor_msgs::LaserScan::ConstPtr &scan) {
  sensor_msgs::PointCloud2 pcl2;
  projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, pcl2,
                                            tf_listener_);
  pcl_pub_.publish(pcl2);
}

}  // namespace koichi_robotics_lib

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(koichi_robotics_lib::LasaerScanToPointCloudNodelet,
                       nodelet::Nodelet)
