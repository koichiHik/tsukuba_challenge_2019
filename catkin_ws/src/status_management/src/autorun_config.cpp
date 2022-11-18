
// Original
#include <status_management/autorun_config.h>

// YAML
#include <yaml-cpp/yaml.h>

// Glog
#include <glog/logging.h>

// STL
#include <iostream>

namespace koichi_robotics_lib {

init_config YamlConfigReader::ReadInitConfig(const std::string &path) {
  init_config conf;

  LOG(INFO) << "Path : " << path;
  YAML::Node node = YAML::LoadFile(path);

  conf.use_gps_ = node["use_gps"].as<bool>();
  conf.pose_initializer_ = node["pose_initializer"].as<bool>();
  conf.init_via_gnss_ = node["init_via_gnss"].as<bool>();
  conf.plane_number_ = node["plane_number"].as<int>();

  conf.init_pose_.x_ = node["init_pose"]["x"].as<double>();
  conf.init_pose_.y_ = node["init_pose"]["y"].as<double>();
  conf.init_pose_.z_ = node["init_pose"]["z"].as<double>();
  conf.init_pose_.roll_ = node["init_pose"]["roll"].as<double>();
  conf.init_pose_.pitch_ = node["init_pose"]["pitch"].as<double>();
  conf.init_pose_.yaw_ = node["init_pose"]["yaw"].as<double>();

  return conf;
}

std::map<unsigned int, file_config> YamlConfigReader::ReadFileConfig(
    const std::string &path) {
  YAML::Node yamlnodes = YAML::LoadFile(path);

  std::map<unsigned int, file_config> file_configs;
  for (YAML::const_iterator itr = yamlnodes.begin(); itr != yamlnodes.end();
       ++itr) {
    file_config conf;
    conf.course_no_ = itr->first.as<unsigned int>();
    conf.init_pose_ = itr->second["init_pose"].as<bool>();
    conf.map_pcd_file_ = itr->second["map_pcd_file"].as<std::string>();
    conf.lane_csv_file_ = itr->second["smooth_waypoint_csv"].as<std::string>();
    conf.world_to_map_json_file_ =
        itr->second["world_to_map_json"].as<std::string>();
    conf.course_start_yaml_file_ =
        itr->second["course_start_yaml"].as<std::string>();

    file_configs.insert(std::make_pair(conf.course_no_, std::move(conf)));
  }

  return file_configs;
}

}  // namespace koichi_robotics_lib
