#include <math.h>
#include <sys/stat.h>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <limits>
#include <memory>
#include <thread>

#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#include <sl/Camera.hpp>

#include <cuda.h>
#include <cuda_runtime.h>

using namespace std;

namespace zed_wrapper {

void ReadParams(std::string param_name, bool result) {
  if (!result) {
    ROS_FATAL("Param read failure. : %s", param_name.c_str());
  }
}

class ZEDWrapperNodelet : public nodelet::Nodelet {
 private:
  void device_poll();
  void onInit();

  /* \brief Convert an sl:Mat to a cv::Mat
          * \param mat : the sl::Mat to convert
          */
  cv::Mat toCVMat(sl::Mat &mat);

  /* \brief Image to ros message conversion
          * \param img : the image to publish
          * \param encodingType : the sensor_msgs::image_encodings encoding type
          * \param frameId : the id of the reference frame of the image
          * \param t : the ros::Time to stamp the image
          */
  sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img,
                                      const std::string encodingType,
                                      std::string frameId, ros::Time t);

  /* \brief Publish a cv::Mat image with a ros Publisher
          * \param img : the image to publish
          * \param pub_img : the publisher object to use
          * \param img_frame_id : the id of the reference frame of the image
          * \param t : the ros::Time to stamp the image
          */
  void publishImage(cv::Mat img, image_transport::Publisher &pub_img,
                    string img_frame_id, ros::Time t);

  /* \brief Publish the informations of a camera with a ros Publisher
          * \param cam_info_msg : the information message to publish
          * \param pub_cam_info : the publisher object to use
          * \param t : the ros::Time to stamp the message
          */
  void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg,
                      ros::Publisher pub_cam_info, ros::Time t);

  /* \brief Get the information of the ZED cameras and store them in an
   * information message
          * \param zed : the sl::zed::Camera* pointer to an instance
          * \param left_cam_info_msg : the information message to fill with the
   * left camera informations
          * \param right_cam_info_msg : the information message to fill with the
   * right camera informations
          * \param left_frame_id : the id of the reference frame of the left
   * camera
          * \param right_frame_id : the id of the reference frame of the right
   * camera
          */
  void fillRectCamInfo(sl::Camera *zed,
                       sensor_msgs::CameraInfoPtr left_cam_info_msg,
                       sensor_msgs::CameraInfoPtr right_cam_info_msg,
                       string left_frame_id, string right_frame_id);

  /* \brief Get the information of the ZED cameras and store them in an
   * information message
          * \param zed : the sl::zed::Camera* pointer to an instance
          * \param left_cam_info_msg : the information message to fill with the
   * left camera informations
          * \param right_cam_info_msg : the information message to fill with the
   * right camera informations
          * \param left_frame_id : the id of the reference frame of the left
   * camera
          * \param right_frame_id : the id of the reference frame of the right
   * camera
          */
  void fillCamInfo(sl::Camera *zed,
                   sensor_msgs::CameraInfoPtr left_cam_info_msg,
                   sensor_msgs::CameraInfoPtr right_cam_info_msg,
                   string left_frame_id, string right_frame_id);

  void readParams();

  void registerPublishers();

  void initializeZedCam();

 private:
  ros::NodeHandle nh, nh_ns;

  boost::shared_ptr<boost::thread> device_poll_thread;

  image_transport::Publisher pub_left, pub_right, pub_raw_left, pub_raw_right;

  ros::Publisher pub_left_cam_info, pub_right_cam_info, pub_left_raw_cam_info,
      pub_right_raw_cam_info;

  std::string left_frame_id, right_frame_id;
  std::string left_topic, left_raw_topic, left_cam_info_topic,
      left_raw_cam_info_topic;
  std::string right_topic, right_raw_topic, right_cam_info_topic,
      right_raw_cam_info_topic;

  // Launch file parameters
  int resolution, quality, sensing_mode, rate, gpu_id, zed_id;
  bool publish_raw;

  // zed object
  sl::InitParameters param;
  std::unique_ptr<sl::Camera> zed;

};  // class ZEDROSWrapperNodelet

void ZEDWrapperNodelet::readParams() {
  // Frame Definition
  ReadParams("left_frame_id",
             nh_ns.param<std::string>("left_frame_id", left_frame_id, ""));
  ReadParams("right_frame_id",
             nh_ns.param<std::string>("right_frame_id", right_frame_id, ""));

  // Topic Name
  ReadParams("left_topic",
             nh_ns.param<std::string>("left_topic", left_topic, ""));
  ReadParams("left_raw_topic",
             nh_ns.param<std::string>("left_raw_topic", left_raw_topic, ""));
  ReadParams(
      "left_cam_info_topic",
      nh_ns.param<std::string>("left_cam_info_topic", left_cam_info_topic, ""));
  ReadParams("left_raw_cam_info_topic",
             nh_ns.param<std::string>("left_raw_cam_info_topic",
                                      left_raw_cam_info_topic, ""));

  ReadParams("right_topic",
             nh_ns.param<std::string>("right_topic", right_topic, ""));
  ReadParams("right_raw_topic",
             nh_ns.param<std::string>("right_raw_topic", right_raw_topic, ""));
  ReadParams("right_cam_info_topic",
             nh_ns.param<std::string>("right_cam_info_topic",
                                      right_cam_info_topic, ""));
  ReadParams("right_raw_cam_info_topic",
             nh_ns.param<std::string>("right_raw_cam_info_topic",
                                      right_raw_cam_info_topic, ""));

  // Parameters
  ReadParams("resolution",
             nh_ns.param<int>("resolution", resolution,
                              static_cast<int>(sl::RESOLUTION_HD720)));
  ReadParams("quality",
             nh_ns.param<int>("quality", quality,
                              static_cast<int>(sl::DEPTH_MODE_PERFORMANCE)));
  ReadParams("sensing_mode",
             nh_ns.param<int>("sensing_mode", sensing_mode,
                              static_cast<int>(sl::SENSING_MODE_STANDARD)));
  ReadParams("frame_rate", nh_ns.param<int>("frame_rate", rate, 30));
  ReadParams("gpu_id", nh_ns.param<int>("gpu_id", gpu_id, -1));
  ReadParams("zed_id",
             nh_ns.param<int>("zed_id", zed_id, 0));
  ReadParams("publish_raw",
             nh_ns.param<bool>("publish_raw", publish_raw, false));

  return;
}

void ZEDWrapperNodelet::registerPublishers() {
  // Image publishers
  image_transport::ImageTransport it_zed(nh);
  pub_left = it_zed.advertise(left_topic, 1);    // left
  pub_right = it_zed.advertise(right_topic, 1);  // right

  // Camera info publishers
  pub_left_cam_info =
      nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1);  // left
  pub_right_cam_info =
      nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1);  // right
  pub_left_raw_cam_info = nh.advertise<sensor_msgs::CameraInfo>(
      left_raw_cam_info_topic, 1);  // left
  pub_right_raw_cam_info = nh.advertise<sensor_msgs::CameraInfo>(
      right_raw_cam_info_topic, 1);  // right

  NODELET_INFO_STREAM("Advertized on topic " << left_topic);
  NODELET_INFO_STREAM("Advertized on topic " << right_topic);
  NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);
  NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);
  NODELET_INFO_STREAM("Advertized on topic " << left_raw_cam_info_topic);
  NODELET_INFO_STREAM("Advertized on topic " << right_raw_cam_info_topic);

  if (publish_raw) {
    pub_raw_left = it_zed.advertise(left_raw_topic, 1);    // left raw
    pub_raw_right = it_zed.advertise(right_raw_topic, 1);  // right raw

    NODELET_INFO_STREAM("Advertized on topic " << left_raw_topic);
    NODELET_INFO_STREAM("Advertized on topic " << right_raw_topic);
  }
  return;
}

void ZEDWrapperNodelet::initializeZedCam() {

  // Create the ZED object
  zed.reset(new sl::Camera());

  //CUcontext pctx;
  //cuCtxGetCurrent(&pctx);
  //param.sdk_cuda_ctx = pctx;
  param.camera_fps = rate;
  param.camera_resolution = static_cast<sl::RESOLUTION>(resolution);
  param.camera_linux_id = zed_id;

  param.coordinate_units = sl::UNIT_METER;
  param.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
  param.enable_right_side_measure = false;
  param.depth_mode = static_cast<sl::DEPTH_MODE>(quality);
  param.sdk_verbose = true;
  param.sdk_gpu_id = gpu_id;

  sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
  while (err != sl::SUCCESS) {
    err = zed->open(param);
    NODELET_INFO_STREAM(errorCode2str(err));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  return;
}

void ZEDWrapperNodelet::onInit() {
  nh = getMTNodeHandle();
  nh_ns = getMTPrivateNodeHandle();

  readParams();

  initializeZedCam();

  registerPublishers();

  device_poll_thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&ZEDWrapperNodelet::device_poll, this)));
}

void ZEDWrapperNodelet::device_poll() {
  ros::Rate loop_rate(rate);
  ros::Time old_t = ros::Time::now();
  bool old_image = false;

  // Get the parameters of the ZED images
  int width = zed->getResolution().width;
  int height = zed->getResolution().height;

  cv::Size cvSize(width, height);
  cv::Mat leftImRGB(cvSize, CV_8UC3);
  cv::Mat rightImRGB(cvSize, CV_8UC3);

  // Create and fill the camera information messages
  sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
  sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
  sensor_msgs::CameraInfoPtr left_raw_cam_info_msg(
      new sensor_msgs::CameraInfo());
  sensor_msgs::CameraInfoPtr right_raw_cam_info_msg(
      new sensor_msgs::CameraInfo());

  fillRectCamInfo(zed.get(), left_cam_info_msg, right_cam_info_msg,
                  left_frame_id, right_frame_id);
  fillCamInfo(zed.get(), left_raw_cam_info_msg, right_raw_cam_info_msg,
              left_frame_id, right_frame_id);

  sl::RuntimeParameters runParams;
  runParams.sensing_mode = static_cast<sl::SENSING_MODE>(sensing_mode);
  sl::Mat leftZEDMat, rightZEDMat;

  // Main loop
  while (nh_ns.ok()) {
    // Run the loop only if there is some subscribers
    ros::Time t = ros::Time::now();  // Get current time
    runParams.enable_depth = false;
    runParams.enable_point_cloud = false;
    old_image = zed->grab(runParams);

    if (old_image) {  // Detect if a error occurred (for example: the zed have
                      // been disconnected) and re-initialize the ZED
      NODELET_DEBUG("Wait for a new image to proceed");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      if ((t - old_t).toSec() > 5) {
        // delete the old object before constructing a new one
        zed.reset();
        zed.reset(new sl::Camera());
        NODELET_INFO("Re-openning the ZED");
        sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
        while (err != sl::SUCCESS) {
          err = zed->open(param);  // Try to initialize the ZED
          NODELET_INFO_STREAM(errorCode2str(err));
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
      }
      continue;
    }

    old_t = ros::Time::now();

    zed->retrieveImage(leftZEDMat, sl::VIEW_LEFT);
    zed->retrieveImage(rightZEDMat, sl::VIEW_RIGHT);

    cv::cvtColor(toCVMat(leftZEDMat), leftImRGB, CV_RGBA2RGB);
    cv::cvtColor(toCVMat(rightZEDMat), rightImRGB, CV_RGBA2RGB);

    publishCamInfo(left_cam_info_msg, pub_left_cam_info, t);
    publishCamInfo(right_cam_info_msg, pub_right_cam_info, t);
    publishCamInfo(left_raw_cam_info_msg, pub_left_raw_cam_info, t);
    publishCamInfo(right_raw_cam_info_msg, pub_right_raw_cam_info, t);

    publishImage(leftImRGB, pub_left, left_frame_id, t);
    publishImage(rightImRGB, pub_right, right_frame_id, t);

    // Publish the right image if someone has subscribed to
    if (publish_raw) {
      zed->retrieveImage(leftZEDMat, sl::VIEW_LEFT_UNRECTIFIED);
      zed->retrieveImage(rightZEDMat, sl::VIEW_RIGHT_UNRECTIFIED);

      cv::cvtColor(toCVMat(leftZEDMat), leftImRGB, CV_RGBA2RGB);
      cv::cvtColor(toCVMat(rightZEDMat), rightImRGB, CV_RGBA2RGB);

      publishImage(leftImRGB, pub_raw_left, left_frame_id, t);
      publishImage(rightImRGB, pub_raw_right, right_frame_id, t);
    }

    loop_rate.sleep();
  }  // while loop

  zed.reset();
}

void ZEDWrapperNodelet::publishImage(cv::Mat img,
                                     image_transport::Publisher &pub_img,
                                     string img_frame_id, ros::Time t) {
  pub_img.publish(
      imageToROSmsg(img, sensor_msgs::image_encodings::BGR8, img_frame_id, t));
}

void ZEDWrapperNodelet::publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg,
                                       ros::Publisher pub_cam_info,
                                       ros::Time t) {
  cam_info_msg->header.stamp = t;
  pub_cam_info.publish(cam_info_msg);
}

void ZEDWrapperNodelet::fillRectCamInfo(
    sl::Camera *zed, sensor_msgs::CameraInfoPtr left_cam_info_msg,
    sensor_msgs::CameraInfoPtr right_cam_info_msg, string left_frame_id,
    string right_frame_id) {
  int width = zed->getResolution().width;
  int height = zed->getResolution().height;

  sl::CameraInformation zedParam = zed->getCameraInformation();

  float baseline = zedParam.calibration_parameters.T[0] *
                   0.001;  // baseline converted in meters

  float fx = zedParam.calibration_parameters.left_cam.fx;
  float fy = zedParam.calibration_parameters.left_cam.fy;
  float cx = zedParam.calibration_parameters.left_cam.cx;
  float cy = zedParam.calibration_parameters.left_cam.cy;

  // There is no distorsions since the images are rectified
  double k1 = 0;
  double k2 = 0;
  double k3 = 0;
  double p1 = 0;
  double p2 = 0;

  left_cam_info_msg->distortion_model =
      sensor_msgs::distortion_models::PLUMB_BOB;
  right_cam_info_msg->distortion_model =
      sensor_msgs::distortion_models::PLUMB_BOB;

  left_cam_info_msg->D.resize(5);
  right_cam_info_msg->D.resize(5);
  left_cam_info_msg->D[0] = right_cam_info_msg->D[0] = k1;
  left_cam_info_msg->D[1] = right_cam_info_msg->D[1] = k2;
  left_cam_info_msg->D[2] = right_cam_info_msg->D[2] = k3;
  left_cam_info_msg->D[3] = right_cam_info_msg->D[3] = p1;
  left_cam_info_msg->D[4] = right_cam_info_msg->D[4] = p2;

  left_cam_info_msg->K.fill(0.0);
  right_cam_info_msg->K.fill(0.0);
  left_cam_info_msg->K[0] = right_cam_info_msg->K[0] = fx;
  left_cam_info_msg->K[2] = right_cam_info_msg->K[2] = cx;
  left_cam_info_msg->K[4] = right_cam_info_msg->K[4] = fy;
  left_cam_info_msg->K[5] = right_cam_info_msg->K[5] = cy;
  left_cam_info_msg->K[8] = right_cam_info_msg->K[8] = 1.0;

  left_cam_info_msg->R.fill(0.0);
  right_cam_info_msg->R.fill(0.0);

  left_cam_info_msg->P.fill(0.0);
  right_cam_info_msg->P.fill(0.0);
  left_cam_info_msg->P[0] = right_cam_info_msg->P[0] = fx;
  left_cam_info_msg->P[2] = right_cam_info_msg->P[2] = cx;
  left_cam_info_msg->P[5] = right_cam_info_msg->P[5] = fy;
  left_cam_info_msg->P[6] = right_cam_info_msg->P[6] = cy;
  left_cam_info_msg->P[10] = right_cam_info_msg->P[10] = 1.0;
  right_cam_info_msg->P[3] = (-1 * fx * baseline);

  left_cam_info_msg->width = right_cam_info_msg->width = width;
  left_cam_info_msg->height = right_cam_info_msg->height = height;

  left_cam_info_msg->header.frame_id = left_frame_id;
  right_cam_info_msg->header.frame_id = right_frame_id;
}

void ZEDWrapperNodelet::fillCamInfo(
    sl::Camera *zed, sensor_msgs::CameraInfoPtr left_cam_info_msg,
    sensor_msgs::CameraInfoPtr right_cam_info_msg, string left_frame_id,
    string right_frame_id) {
  int width = zed->getResolution().width;
  int height = zed->getResolution().height;

  sl::CameraInformation zedParam = zed->getCameraInformation();

  float baseline = zedParam.calibration_parameters.T[0] *
                   0.001;  // baseline converted in meters

  left_cam_info_msg->distortion_model =
      sensor_msgs::distortion_models::PLUMB_BOB;
  right_cam_info_msg->distortion_model =
      sensor_msgs::distortion_models::PLUMB_BOB;

  left_cam_info_msg->D.resize(5);
  left_cam_info_msg->D[0] =
      zedParam.calibration_parameters_raw.left_cam.disto[0];
  left_cam_info_msg->D[1] =
      zedParam.calibration_parameters_raw.left_cam.disto[1];
  left_cam_info_msg->D[2] =
      zedParam.calibration_parameters_raw.left_cam.disto[2];
  left_cam_info_msg->D[3] =
      zedParam.calibration_parameters_raw.left_cam.disto[3];
  left_cam_info_msg->D[4] =
      zedParam.calibration_parameters_raw.left_cam.disto[4];

  right_cam_info_msg->D.resize(5);
  right_cam_info_msg->D[0] =
      zedParam.calibration_parameters_raw.right_cam.disto[0];
  right_cam_info_msg->D[1] =
      zedParam.calibration_parameters_raw.right_cam.disto[1];
  right_cam_info_msg->D[2] =
      zedParam.calibration_parameters_raw.right_cam.disto[2];
  right_cam_info_msg->D[3] =
      zedParam.calibration_parameters_raw.right_cam.disto[3];
  right_cam_info_msg->D[4] =
      zedParam.calibration_parameters_raw.right_cam.disto[4];

  left_cam_info_msg->K.fill(0.0);
  left_cam_info_msg->K[0] = zedParam.calibration_parameters_raw.left_cam.fx;
  left_cam_info_msg->K[2] = zedParam.calibration_parameters_raw.left_cam.cx;
  left_cam_info_msg->K[4] = zedParam.calibration_parameters_raw.left_cam.fy;
  left_cam_info_msg->K[5] = zedParam.calibration_parameters_raw.left_cam.cy;
  left_cam_info_msg->K[8] = 1.0;

  right_cam_info_msg->K.fill(0.0);
  right_cam_info_msg->K[0] = zedParam.calibration_parameters_raw.right_cam.fx;
  right_cam_info_msg->K[2] = zedParam.calibration_parameters_raw.right_cam.cx;
  right_cam_info_msg->K[4] = zedParam.calibration_parameters_raw.right_cam.fy;
  right_cam_info_msg->K[5] = zedParam.calibration_parameters_raw.right_cam.cy;
  right_cam_info_msg->K[8] = 1.0;

  left_cam_info_msg->R.fill(0.0);
  right_cam_info_msg->R.fill(0.0);

  left_cam_info_msg->P.fill(0.0);
  left_cam_info_msg->P[0] = zedParam.calibration_parameters.left_cam.fx;
  left_cam_info_msg->P[2] = zedParam.calibration_parameters.left_cam.cx;
  left_cam_info_msg->P[5] = zedParam.calibration_parameters.left_cam.fy;
  left_cam_info_msg->P[6] = zedParam.calibration_parameters.left_cam.cy;
  left_cam_info_msg->P[10] = 1.0;

  right_cam_info_msg->P.fill(0.0);
  right_cam_info_msg->P[0] = zedParam.calibration_parameters.right_cam.fx;
  right_cam_info_msg->P[2] = zedParam.calibration_parameters.right_cam.cx;
  right_cam_info_msg->P[5] = zedParam.calibration_parameters.right_cam.fy;
  right_cam_info_msg->P[6] = zedParam.calibration_parameters.right_cam.cy;
  right_cam_info_msg->P[10] = 1.0;
  right_cam_info_msg->P[3] =
      (-1 * zedParam.calibration_parameters.right_cam.fx * baseline);

  left_cam_info_msg->width = right_cam_info_msg->width = width;
  left_cam_info_msg->height = right_cam_info_msg->height = height;

  left_cam_info_msg->header.frame_id = left_frame_id;
  right_cam_info_msg->header.frame_id = right_frame_id;
}

cv::Mat ZEDWrapperNodelet::toCVMat(sl::Mat &mat) {
  if (mat.getMemoryType() == sl::MEM_GPU) {
    mat.updateCPUfromGPU();
  }

  int cvType;
  switch (mat.getDataType()) {
    case sl::MAT_TYPE_32F_C1:
      cvType = CV_32FC1;
      break;
    case sl::MAT_TYPE_32F_C2:
      cvType = CV_32FC2;
      break;
    case sl::MAT_TYPE_32F_C3:
      cvType = CV_32FC3;
      break;
    case sl::MAT_TYPE_32F_C4:
      cvType = CV_32FC4;
      break;
    case sl::MAT_TYPE_8U_C1:
      cvType = CV_8UC1;
      break;
    case sl::MAT_TYPE_8U_C2:
      cvType = CV_8UC2;
      break;
    case sl::MAT_TYPE_8U_C3:
      cvType = CV_8UC3;
      break;
    case sl::MAT_TYPE_8U_C4:
      cvType = CV_8UC4;
      break;
  }
  return cv::Mat((int)mat.getHeight(), (int)mat.getWidth(), cvType,
                 mat.getPtr<sl::uchar1>(sl::MEM_CPU),
                 mat.getStepBytes(sl::MEM_CPU));
}

sensor_msgs::ImagePtr ZEDWrapperNodelet::imageToROSmsg(
    cv::Mat img, const std::string encodingType, std::string frameId,
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

}  // namespace zed_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_wrapper::ZEDWrapperNodelet, nodelet::Nodelet);
