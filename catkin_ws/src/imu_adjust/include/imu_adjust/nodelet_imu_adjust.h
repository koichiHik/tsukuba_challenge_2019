
#ifndef _NODELET_IMU_ADJUST_H_
#define _NODELET_IMU_ADJUST_H_

// STL
#include <memory>

// ROS
#include <nodelet/nodelet.h>

namespace imu_adjust {

struct NodeletImuAdjustImpl;
class NodeletImuAdjust : public nodelet::Nodelet {
 public:
  NodeletImuAdjust();
  ~NodeletImuAdjust();

  virtual void onInit();

 protected:
  std::unique_ptr<NodeletImuAdjustImpl> p_impl_;
};

}  // namespace imu_adjust

#endif