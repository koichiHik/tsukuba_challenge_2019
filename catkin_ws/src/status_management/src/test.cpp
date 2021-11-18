
#include <iostream>
#include "status_management_util.h"

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv) {
  std::cout << "Hello world" << std::endl;

  geometry_msgs::Pose from_pose;
  geometry_msgs::Pose to_pose;

  tf::Quaternion from_q, to_q;

  from_q.setEulerZYX(-1.985550, 0.008626, 0.024602);

  std::cout << from_q.getX() << std::endl;
  std::cout << from_q.getY() << std::endl;
  std::cout << from_q.getZ() << std::endl;
  std::cout << from_q.getW() << std::endl;

  return 0;
}