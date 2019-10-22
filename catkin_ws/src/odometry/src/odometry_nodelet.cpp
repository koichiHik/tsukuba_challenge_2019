

//
#include <sys/time.h>
#include <cstddef>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>

// 3rd party include
#include "kmsgs/Imu.h"
#include "odometry.h"

namespace koichi_robotics_lib {
class OdometryNodelet : public nodelet::Nodelet {
 public:
  OdometryNodelet();

  ~OdometryNodelet();

  virtual void onInit();

 private:
  void publishTransform(const nav_msgs::Odometry& odomMsg, std::string frameName);

  void sensorCallback(const sensor_msgs::JointStateConstPtr& jointPos);

  void sensorCallback(const kmsgs::ImuConstPtr& imuMsg,
                      const sensor_msgs::JointStateConstPtr& jointPos);

  void calcAndPublishOdom(const ros::Time& stamp);

  void readParams(OdomParams& odomParam);

  void readParam(std::string paramName, bool result);

  void parseJointStatesMsg(const sensor_msgs::JointStateConstPtr& jointPos, double& rRad,
                           double& lRad, double& rVel, double& lVel);

  void fillOdomMessage(nav_msgs::Odometry& odomMsg, std::string frameName, const ros::Time& stamp,
                       const OdomPos& odomPos, const OdomVel& odomVel);

 private:
  // Topic Name
  std::string imuTopicName, jointStatesTopicName, odomWhlTopicName, odomImuTopicName;
  std::string odomImuFrameName, odomWhlFrameName, baselinkFrameName;

  int maxSyncDiff, offset_gyro_z;
  bool use_odom_whl, calc_offset;
  double cycle_time, last_pub_stamp;

  ros::NodeHandle nh, nh_ns;
  nav_msgs::Odometry curOdom;

  // ROS Subsceibers
  message_filters::Subscriber<kmsgs::Imu>* subImu;
  message_filters::Subscriber<sensor_msgs::JointState>* subWhl;

  // ROS Sync Policy
  typedef message_filters::sync_policies::ApproximateTime<kmsgs::Imu, sensor_msgs::JointState>
      OdomSyncPolicy;
  message_filters::Synchronizer<OdomSyncPolicy>* odomSync;

  // ROS Publisher
  ros::Publisher odomPubWhl, odomPubImu;
  tf::TransformBroadcaster* odomBroadCaster;

  // Class
  Odometry odometry;
};
}  // namespace koichi_robotics_lib

using namespace std;
using namespace koichi_robotics_lib;

OdometryNodelet::OdometryNodelet() {}

OdometryNodelet::~OdometryNodelet() {
  odometry.Finalize();
  delete this->odomBroadCaster;
  delete this->subImu;
  delete this->subWhl;
  delete this->odomSync;
}

void OdometryNodelet::onInit() {
  NODELET_DEBUG("Initializing Odometry Nodelet....");

  nh = getMTNodeHandle();
  nh_ns = getMTPrivateNodeHandle();
  odomBroadCaster = new tf::TransformBroadcaster();

  OdomParams params;
  readParams(params);
  odometry.Initialize(params);

  /*
  subImu = new message_filters::Subscriber<kmsgs::Imu>(nh, imuTopicName, 100);
  subWhl = new message_filters::Subscriber<sensor_msgs::JointState>(
      nh, jointStatesTopicName, 100);
  */

  subImu = new message_filters::Subscriber<kmsgs::Imu>(nh, "imu", 100);
  subWhl = new message_filters::Subscriber<sensor_msgs::JointState>(nh, "joint_states", 100);

  if (use_odom_whl) {
    subWhl->registerCallback(&OdometryNodelet::sensorCallback, this);
  } else {
    odomSync = new message_filters::Synchronizer<OdomSyncPolicy>(OdomSyncPolicy(maxSyncDiff),
                                                                 *subImu, *subWhl);
    odomSync->registerCallback(boost::bind(&OdometryNodelet::sensorCallback, this, _1, _2));
  }

  // odomPubWhl = nh.advertise<nav_msgs::Odometry>(odomWhlTopicName, 100);
  // odomPubImu = nh.advertise<nav_msgs::Odometry>(odomImuTopicName, 100);
  odomPubWhl = nh.advertise<nav_msgs::Odometry>("odom_whl", 100);
  odomPubImu = nh.advertise<nav_msgs::Odometry>("odom_imu", 100);
}

void OdometryNodelet::readParams(OdomParams& odomParams) {
  // Topic Name
  /**
  readParam("imu_topic_name",
            nh_ns.param<std::string>("imu_topic_name", imuTopicName, ""));
  readParam("joint_states_topic_name",
            nh_ns.param<std::string>("joint_states_topic_name",
                                     jointStatesTopicName, ""));
  readParam(
      "odom_whl_topic_name",
      nh_ns.param<std::string>("odom_whl_topic_name", odomWhlTopicName, ""));
  readParam(
      "odom_imu_topic_name",
      nh_ns.param<std::string>("odom_imu_topic_name", odomImuTopicName, ""));
  **/

  // Frame Name
  readParam("odometry_imu_frame_name",
            nh_ns.param<std::string>("odometry_imu_frame_name", odomImuFrameName, ""));
  readParam("odometry_whl_frame_name",
            nh_ns.param<std::string>("odometry_whl_frame_name", odomWhlFrameName, ""));
  readParam("base_link_frame_name",
            nh_ns.param<std::string>("base_link_frame_name", baselinkFrameName, ""));

  // Hardware Related
  readParam("gyro_1bit_in_rad",
            nh_ns.param<double>("gyro_1bit_in_rad", odomParams.gyro1BitInRad, 0.0));
  double r_whl_dia, l_whl_dia;
  readParam("l_whl_diameter", nh_ns.param<double>("l_whl_diameter", l_whl_dia, 0.0));
  odomParams.leftDistPerRad = l_whl_dia;
  readParam("r_whl_diameter", nh_ns.param<double>("r_whl_diameter", r_whl_dia, 0.0));
  odomParams.rightDistPerRad = r_whl_dia;
  readParam("wheel_base", nh_ns.param<double>("wheel_base", odomParams.wheelBase, 1.0));

  // Odometry Parameter
  readParam("standstill_thresh",
            nh_ns.param<double>("standstill_thresh", odomParams.standstillThresh, 0.0));
  readParam("use_left_pls", nh_ns.param<bool>("use_left_pls", odomParams.useLeftPls, false));
  readParam("use_right_pls", nh_ns.param<bool>("use_right_pls", odomParams.useRightPls, false));
  readParam("cycle_time", nh_ns.param<double>("cycle_time", odomParams.cycleTime, 0.0));
  cycle_time = odomParams.cycleTime;
  readParam("max_sync_diff", nh_ns.param<int>("max_sync_diff", maxSyncDiff, 8));
  readParam("use_odom_whl", nh_ns.param<bool>("use_odom_whl", use_odom_whl, false));
  readParam("calc_offset", nh_ns.param<bool>("calc_offset", odomParams.calc_offset, true));
  readParam("offset_gyro_z", nh_ns.param<int>("offset_gyro_z", odomParams.offset_gyro_z, 0));

  // Buffer Size
  readParam("imu_yaw_buffer", nh_ns.param<int>("imu_yaw_buffer", odomParams.imuYawBufferSize, 0.0));
  readParam("whl_yaw_buffer", nh_ns.param<int>("whl_yaw_buffer", odomParams.whlYawBufferSize, 0.0));
  readParam("dist_buffer", nh_ns.param<int>("dist_buffer", odomParams.distBufferSize, 0));
}

void OdometryNodelet::readParam(string paramName, bool result) {
  if (!result) {
    ROS_FATAL("Param read failure. : %s", paramName.c_str());
  }
}

void OdometryNodelet::publishTransform(const nav_msgs::Odometry& odomMsg,
                                       std::string odomFrameName) {
  tf::Transform odomTrans;
  tf::Pose tfPose;

  tf::poseMsgToTF(odomMsg.pose.pose, odomTrans);
  odomBroadCaster->sendTransform(
      tf::StampedTransform(odomTrans, ros::Time::now(), odomFrameName, baselinkFrameName));
}

void OdometryNodelet::sensorCallback(const sensor_msgs::JointStateConstPtr& jointPos) {
  double rRad, lRad, rVel, lVel;
  parseJointStatesMsg(jointPos, rRad, lRad, rVel, lVel);
  odometry.SetCurrentPosition(rRad, lRad, rVel, lVel);
  odometry.SetCurrentGyroZ(0.0);

  static double last_stamp = jointPos->header.stamp.toSec();
  if (jointPos->header.stamp.toSec() - last_stamp > cycle_time - cycle_time / 10.0) {
    double new_stamp = jointPos->header.stamp.toSec();
    double dT = new_stamp - last_stamp;
    last_stamp = new_stamp;

    OdomPos odomWhl, odomImu;
    OdomVel velWhl, velImu;
    odometry.CalculateOdometry(odomWhl, odomImu, velWhl, velImu, dT);

    // Published Whl Based Odometry.
    {
      nav_msgs::Odometry odomWhlMsg;
      fillOdomMessage(odomWhlMsg, odomWhlFrameName, jointPos->header.stamp, odomWhl, velWhl);
      publishTransform(odomWhlMsg, odomWhlFrameName);
      odomPubWhl.publish(odomWhlMsg);
    }
  }
}

void OdometryNodelet::sensorCallback(const kmsgs::ImuConstPtr& imuMsg,
                                     const sensor_msgs::JointStateConstPtr& jointPos) {
  double rRad, lRad, rVel, lVel;
  parseJointStatesMsg(jointPos, rRad, lRad, rVel, lVel);
  odometry.SetCurrentPosition(rRad, lRad, rVel, lVel);
  odometry.SetCurrentGyroZ(imuMsg->omegaZ);
  calcAndPublishOdom(imuMsg->header.stamp);
}

void OdometryNodelet::parseJointStatesMsg(const sensor_msgs::JointStateConstPtr& jointPos,
                                          double& rRad, double& lRad, double& rVel, double& lVel) {
  for (int i = 0; i < jointPos->name.size(); i++) {
    string name = jointPos->name[i];
    double jntPos = jointPos->position[i];
    double jntVel = jointPos->velocity[i];

    if (name == "right") {
      rRad = -jntPos;
      rVel = -jntVel;
    } else if (name == "left") {
      lRad = jntPos;
      lVel = jntVel;
    } else {
      ROS_ERROR("Unexpected Joint Name : %s", name.c_str());
    }
  }
}

void OdometryNodelet::calcAndPublishOdom(const ros::Time& stamp) {
  static double lastStamp = stamp.toSec();
  double newStamp = stamp.toSec();
  double dT = newStamp - lastStamp;
  lastStamp = newStamp;

  OdomPos odomWhl, odomImu;
  OdomVel velWhl, velImu;
  odometry.CalculateOdometry(odomWhl, odomImu, velWhl, velImu, dT);

  if (!use_odom_whl) {
    // Published IMU based odometry.
    {
      nav_msgs::Odometry odomImuMsg;
      fillOdomMessage(odomImuMsg, odomImuFrameName, stamp, odomImu, velImu);
      publishTransform(odomImuMsg, odomImuFrameName);
      odomPubImu.publish(odomImuMsg);
    }
  } else {
    // Published Whl Based Odometry.
    {
      nav_msgs::Odometry odomWhlMsg;
      fillOdomMessage(odomWhlMsg, odomWhlFrameName, stamp, odomWhl, velWhl);
      publishTransform(odomWhlMsg, odomWhlFrameName);
      odomPubWhl.publish(odomWhlMsg);
    }
  }
}

void OdometryNodelet::fillOdomMessage(nav_msgs::Odometry& odomMsg, std::string odomFrameName,
                                      const ros::Time& stamp, const OdomPos& odomPos,
                                      const OdomVel& odomVel) {
  // Header
  odomMsg.header.frame_id = odomFrameName;
  odomMsg.child_frame_id = baselinkFrameName;
  odomMsg.header.stamp = stamp;

  // Pose
  odomMsg.pose.pose.position.x = odomPos.x;
  odomMsg.pose.pose.position.y = odomPos.y;
  odomMsg.pose.pose.position.z = 0.0;
  odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odomPos.angz);

  odomMsg.pose.covariance[0] = odomPos.totDist;
  // Twist
  odomMsg.twist.twist.linear.x = odomVel.vx;
  odomMsg.twist.twist.linear.y = odomVel.vy;
  odomMsg.twist.twist.linear.z = 0;
  odomMsg.twist.twist.angular.x = 0;
  odomMsg.twist.twist.angular.y = 0;
  odomMsg.twist.twist.angular.z = odomVel.wz;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(koichi_robotics_lib::OdometryNodelet, nodelet::Nodelet)
