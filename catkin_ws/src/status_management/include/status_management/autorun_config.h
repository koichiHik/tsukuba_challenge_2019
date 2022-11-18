
#ifndef __STATUS_MANAGEMENT_AUTORUN_CONFIG_H__
#define __STATUS_MANAGEMENT_AUTORUN_CONFIG_H__

// STL
#include <map>
#include <string>

namespace koichi_robotics_lib {

struct pose {
  double x_;
  double y_;
  double z_;
  double roll_;
  double pitch_;
  double yaw_;
};

struct init_config {
  bool use_gps_;
  bool pose_initializer_;
  bool init_via_gnss_;
  int plane_number_;
  pose init_pose_;
};

struct file_config {
  unsigned int course_no_;
  bool init_pose_;
  std::string map_pcd_file_;
  std::string lane_csv_file_;
  std::string world_to_map_json_file_;
  std::string course_start_yaml_file_;
};

struct autorun_config {
  init_config init_conf_;
  file_config file_conf_;
};

class YamlConfigReader {
 public:
  static init_config ReadInitConfig(const std::string &path);

  static std::map<unsigned int, file_config> ReadFileConfig(
      const std::string &path);
};

}  // namespace koichi_robotics_lib

#endif