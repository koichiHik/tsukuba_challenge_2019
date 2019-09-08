
// OpenCV
#include <opencv2/core.hpp>

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>

// Original
#include <stixel_cuda.h>


using namespace std;
using namespace stereo;
using namespace image_transport;
using namespace sensor_msgs;
using namespace message_filters;

namespace koichi_robotics_lib {

void ReadParam(std::string param_name, bool result) {
  if (!result) {
    ROS_ERROR("Param read failure. : %s", param_name.c_str());
  }
}

class StixelCudaNodelet : public nodelet::Nodelet {

public:
  StixelCudaNodelet();
  ~StixelCudaNodelet();

  virtual void onInit();

  void DispImageCallback(const sensor_msgs::ImageConstPtr &disp_img);

  void DualImageCallback(const sensor_msgs::ImageConstPtr &leftImg, const sensor_msgs::ImageConstPtr &rightImg);

private:

  void ReadParams(stereo::StixelCudaParams &params);

  sensor_msgs::ImagePtr imageToROSmsg(cv::Mat &img,
                                      const std::string encodingType,
                                      std::string frameId, ros::Time t);

private:
  bool m_stx_img;
  std::string m_base_frame_id, m_disp_frame_id;
  std::string m_disp_img_topic, m_base_img_topic, m_stixel_img_topic;
  StixelCuda m_stx_cuda;
  int m_max_sync_diff;
  cv::Mat u8_disp_mat, u8_base_mat;

  // ROS Related
  ros::NodeHandle m_nh, m_nh_ns;
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber m_disp_img_sub, m_base_img_sub;
  image_transport::Publisher m_stixel_img_pub;

  // Message Filter
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> *m_sync;
};

StixelCudaNodelet::StixelCudaNodelet() {

}

StixelCudaNodelet::~StixelCudaNodelet() {

}

void StixelCudaNodelet::onInit() {

  NODELET_DEBUG("Initializing StixelCuda Nodelet....");

  m_nh = getMTNodeHandle();
  m_nh_ns = getMTPrivateNodeHandle();

  // Read Parameters
  stereo::StixelCudaParams params;
  ReadParams(params);

  m_stx_cuda.Initialize(params);

  // Memory
  u8_disp_mat = cv::Mat(params.rows, params.cols, CV_8UC1);
  u8_base_mat = cv::Mat(params.rows, params.cols, CV_8UC3);

  // Subscriber
  image_transport::ImageTransport it(m_nh);
  m_disp_img_sub.subscribe(it, m_disp_img_topic, 1);

  if (m_stx_img) {
    // Sync
    m_base_img_sub.subscribe(it, m_base_img_topic, 1);
    m_sync = new Synchronizer<SyncPolicy>(SyncPolicy(m_max_sync_diff), m_base_img_sub, m_disp_img_sub);
    m_sync->registerCallback(
      boost::bind(&StixelCudaNodelet::DualImageCallback, this, _1, _2)
    );
    // Publisher
    m_stixel_img_pub = it.advertise(m_stixel_img_topic, 1);

  } else {
    m_disp_img_sub.registerCallback(&StixelCudaNodelet::DispImageCallback, this);
  }

}


void StixelCudaNodelet::ReadParams(stereo::StixelCudaParams &params) {

  // Topic Names
  ReadParam("disp_img_topic", m_nh_ns.param<std::string>("disp_img_topic", m_disp_img_topic, ""));
  ReadParam("base_img_topic", m_nh_ns.param<std::string>("base_img_topic", m_base_img_topic, ""));
  ReadParam("stixel_img_topic", m_nh_ns.param<std::string>("stixel_img_topic", m_stixel_img_topic, ""));

  // Frame Names
  ReadParam("disp_frame_id", m_nh_ns.param<std::string>("disp_frame_id", m_disp_frame_id, ""));
  ReadParam("base_frame_id", m_nh_ns.param<std::string>("base_frame_id", m_base_frame_id, ""));

  // Debug Setting.
  ReadParam("stixel_img", m_nh_ns.param<bool>("stixel_img", m_stx_img, false));
  ReadParam("max_sync_diff", m_nh_ns.param<int>("max_sync_diff", m_max_sync_diff, 100));

  // Stixel Params
  ReadParam("cols", m_nh_ns.param<int>("cols", params.cols, 0));
  ReadParam("rows", m_nh_ns.param<int>("rows", params.rows, 0));
  ReadParam("max_disp", m_nh_ns.param<int>("max_disp", params.max_disp, 0));

  // Prob
  ReadParam("pout", m_nh_ns.param<float>("pout", params.pout, 0.0f));
  ReadParam("pout_sky", m_nh_ns.param<float>("pout_sky", params.pout_sky, 0.0f));
  ReadParam("pord", m_nh_ns.param<float>("pord", params.pord, 0.0f));
  ReadParam("pgrav", m_nh_ns.param<float>("pgrav", params.pgrav, 0.0f));
  ReadParam("pblg", m_nh_ns.param<float>("pblg", params.pblg, 0.0f));
  ReadParam("pground_given_nexist", m_nh_ns.param<float>("pground_given_nexist", params.pground_given_nexist, 0.0f));
  ReadParam("pobject_given_nexist", m_nh_ns.param<float>("pobject_given_nexist", params.pobject_given_nexist, 0.0f));
  ReadParam("psky_given_nexist", m_nh_ns.param<float>("psky_given_nexist", params.psky_given_nexist, 0.0f));
  ReadParam("pnexist_dis", m_nh_ns.param<float>("pnexist_dis", params.pnexist_dis, 0.0f));
  ReadParam("pground", m_nh_ns.param<float>("pground", params.pground, 0.0f));
  ReadParam("pobject", m_nh_ns.param<float>("pobject", params.pobject, 0.0f));
  ReadParam("psky", m_nh_ns.param<float>("psky", params.psky, 0.0f));

  ReadParam("sigma_disparity_object", m_nh_ns.param<float>("sigma_disparity_object", params.sigma_disparity_object, 0.0f));
  ReadParam("sigma_disparity_ground", m_nh_ns.param<float>("sigma_disparity_ground", params.sigma_disparity_ground, 0.0f));
  ReadParam("sigma_sky", m_nh_ns.param<float>("sigma_sky", params.sigma_sky, 0.0f));

  // Camera Params
  ReadParam("v_hor", m_nh_ns.param<int>("v_hor", params.v_hor, 0));
  ReadParam("focal", m_nh_ns.param<float>("focal", params.focal, 0.0f));
  ReadParam("baseline", m_nh_ns.param<float>("baseline", params.baseline, 0.0f));
  ReadParam("camera_center_y", m_nh_ns.param<float>("camera_center_y", params.camera_canter_y, 0.0f));
  ReadParam("column_step", m_nh_ns.param<int>("column_step", params.column_step, 0));
  ReadParam("width_margin", m_nh_ns.param<int>("width_margin", params.width_margin, 0));
  ReadParam("camera_tilt", m_nh_ns.param<float>("camera_tilt", params.camera_tilt, 0.0f));
  ReadParam("sigma_camera_tilt", m_nh_ns.param<float>("sigma_camera_tilt", params.sigma_camera_tilt, 0.0f));
  ReadParam("camera_height", m_nh_ns.param<float>("camera_height", params.camera_height, 0.0f));
  ReadParam("sigma_camera_height", m_nh_ns.param<float>("sigma_camera_height", params.sigma_camera_height, 0.0f));
  ReadParam("alpha_ground", m_nh_ns.param<float>("alpha_ground", params.alpha_ground, 0.0f));

  // Model Parameters
  ReadParam("median_step", m_nh_ns.param<bool>("median_step", params.median_step, false));
  ReadParam("epsilon", m_nh_ns.param<float>("epsilon", params.epsilon, 0.0f));
  ReadParam("range_object_z", m_nh_ns.param<float>("range_object_z", params.range_object_z, 0.0f));

}

void StixelCudaNodelet::DispImageCallback(const sensor_msgs::ImageConstPtr &disp_img) {

  ROS_ERROR("DualImageCallback Starts");
  /*
  memcpy(u8_disp_mat.data, disp_img->data.data(), disp_img->height * disp_img->width);
  
  vector<stereo::Stixel> stixels;
  m_stx_cuda.Run(u8_disp_mat, stixels);
  */
  ROS_ERROR("DualImageCallback Ends");

}

void StixelCudaNodelet::DualImageCallback(const sensor_msgs::ImageConstPtr &base_img, 
                                          const sensor_msgs::ImageConstPtr &disp_img) {

  vector<vector<stereo::Stixel>> stixels;
  cv_bridge::CvImagePtr cv_base_ptr, cv_disp_ptr;
  try {
    ROS_ERROR("Before 1");
    cv_base_ptr = cv_bridge::toCvCopy(base_img, sensor_msgs::image_encodings::BGR8);

    ROS_ERROR("Before 2");
    cv_disp_ptr = cv_bridge::toCvCopy(disp_img, sensor_msgs::image_encodings::MONO8);

    m_stx_cuda.Run(cv_disp_ptr->image, stixels);

    std::cerr << "Stixel Size : " << stixels.size() << std::endl;

    m_stx_cuda.DrawStixel(stixels, cv_base_ptr->image);

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  m_stixel_img_pub.publish(cv_base_ptr->toImageMsg());
}

sensor_msgs::ImagePtr StixelCudaNodelet::imageToROSmsg(
    cv::Mat &img, const std::string encodingType, std::string frameId,
    ros::Time t) {
  sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  sensor_msgs::Image &imgMessage = *ptr;
  imgMessage.header.stamp = t;
  imgMessage.header.frame_id = frameId;
  imgMessage.height = img.rows;
  imgMessage.width = img.cols;
  imgMessage.encoding = encodingType;
  int num = 1;  // for endianness detection
  imgMessage.is_bigendian = !(*(char *)&num == 1);
  imgMessage.step = img.cols * img.elemSize();
  size_t size = imgMessage.step * img.rows;
  imgMessage.data.resize(size);

  if (img.isContinuous()) {
    memcpy((char *)(&imgMessage.data[0]), img.data, size);
  } else {
    uchar *opencvData = img.data;
    uchar *rosData = (uchar *)(&imgMessage.data[0]);
    for (unsigned int i = 0; i < img.rows; i++) {
      memcpy(rosData, opencvData, imgMessage.step);
      rosData += imgMessage.step;
      opencvData += img.step;
    }
  }
  return ptr;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(koichi_robotics_lib::StixelCudaNodelet, nodelet::Nodelet)