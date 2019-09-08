

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/cudaimgproc.hpp>

// ROS
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

// Original
#include <sgm_cuda.h>

using namespace std;
using namespace stereo;
using namespace message_filters;
using namespace image_transport;

namespace koichi_robotics_lib {

void ReadParams(std::string param_name, bool result) {
  if (!result) {
    ROS_FATAL("Param read failure. : %s", param_name.c_str());
  }
}

class SgmCudaNodelet : public nodelet::Nodelet {
 public:
  virtual void onInit();
  SgmCudaNodelet();
  ~SgmCudaNodelet();
  void stereoImageCallback(const sensor_msgs::ImageConstPtr &leftImg,
                           const sensor_msgs::ImageConstPtr &rightImg);

 private:
  void calcAndPublishDisp(cv::Mat &left, cv::Mat &right);
  void readParams();
  sensor_msgs::ImagePtr imageToROSmsg(cv::Mat &img,
                                      const std::string encodingType,
                                      std::string frameId, ros::Time t);

 private:
  std::string disp_frame_id;
  std::string left_img_topic, right_img_topic, disparity_topic;
  SGMCuda cuda;
  uint8_t p1, p2;
  int width, height, maxSyncDiff, sgmFreq;
  cv::Mat left, right, leftGray, rightGray, disp;
  ros::NodeHandle nh, nh_ns;
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber leftImgSub, rightImgSub;
  image_transport::Publisher dispPub;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      StereoSyncPolicy;
  message_filters::Synchronizer<StereoSyncPolicy> *stSync;
};

SgmCudaNodelet::SgmCudaNodelet() {}

SgmCudaNodelet::~SgmCudaNodelet() {
  cuda.end();
  delete stSync;
}

void SgmCudaNodelet::onInit() {
  NODELET_DEBUG("Initializing SgmCuda Nodelet....");

  nh = getMTNodeHandle();
  nh_ns = getMTPrivateNodeHandle();
  image_transport::ImageTransport it(nh);
  readParams();

  left = cv::Mat(height, width, CV_8UC3);
  right = cv::Mat(height, width, CV_8UC3);
  leftGray = cv::Mat(height, width, CV_8UC1);
  rightGray = cv::Mat(height, width, CV_8UC1);
  disp = cv::Mat(height, width, CV_8UC1);

  cuda.initialize(p1, p2);

  leftImgSub.subscribe(it, left_img_topic, 1);
  rightImgSub.subscribe(it, right_img_topic, 1);
  dispPub = it.advertise(disparity_topic, 1);

  stSync = new Synchronizer<StereoSyncPolicy>(StereoSyncPolicy(maxSyncDiff),
                                              leftImgSub, rightImgSub);
  stSync->registerCallback(
      boost::bind(&SgmCudaNodelet::stereoImageCallback, this, _1, _2));
}

void SgmCudaNodelet::readParams() {
  // Topic Names
  ReadParams("left_img_topic",
             nh_ns.param<std::string>("left_img_topic", left_img_topic, ""));
  ReadParams("right_img_topic",
             nh_ns.param<std::string>("right_img_topic", right_img_topic, ""));
  ReadParams("disparity_topic",
             nh_ns.param<std::string>("disparity_topic", disparity_topic, ""));

  // Frame Names
  ReadParams("disp_frame_id",
             nh_ns.param<std::string>("disp_frame_id", disp_frame_id, ""));

  // Parameters
  {
    int intp1, intp2;
    ReadParams("p1", nh_ns.param("p1", intp1, 7));
    ReadParams("p2", nh_ns.param("p2", intp2, 86));
    p1 = (uint8_t)intp1;
    p2 = (uint8_t)intp2;
  }

  ReadParams("width", nh_ns.param("width", width, 1280));
  ReadParams("height", nh_ns.param("height", height, 720));
  ReadParams("max_sync_diff", nh_ns.param("max_sync_diff", maxSyncDiff, 10));
  ReadParams("sgm_freq", nh_ns.param("sgm_freq", sgmFreq, 0));
}

void SgmCudaNodelet::stereoImageCallback(
    const sensor_msgs::ImageConstPtr &leftImg,
    const sensor_msgs::ImageConstPtr &rightImg) {
  static int loopCnt = 0;
  loopCnt++;
  if (loopCnt < sgmFreq) {
    return;
  }

  loopCnt = 0;

  ros::Time t = leftImg->header.stamp;
  memcpy(left.data, leftImg->data.data(), leftImg->height * leftImg->width * 3);
  memcpy(right.data, rightImg->data.data(),
         rightImg->height * rightImg->width * 3);

  //#if !defined(OCV_2_4)
  cv::cuda::GpuMat gpuLeft, gpuRight, gpuGrayLeft, gpuGrayRight;
  gpuLeft.upload(left);
  gpuRight.upload(right);

  cv::cuda::cvtColor(gpuLeft, gpuGrayLeft, CV_RGB2GRAY);
  cv::cuda::cvtColor(gpuRight, gpuGrayRight, CV_RGB2GRAY);

  gpuGrayLeft.download(leftGray);
  gpuGrayRight.download(rightGray);

  cuda.generateDisparityMat(leftGray, rightGray, disp);

  dispPub.publish(imageToROSmsg(disp, sensor_msgs::image_encodings::MONO8,
                                disp_frame_id, t));
}

sensor_msgs::ImagePtr SgmCudaNodelet::imageToROSmsg(
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
PLUGINLIB_EXPORT_CLASS(koichi_robotics_lib::SgmCudaNodelet, nodelet::Nodelet)
