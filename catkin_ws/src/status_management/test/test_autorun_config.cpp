
// Original
#include <status_management/autorun_config.h>

// Google
#include <glog/logging.h>
#include <gtest/gtest.h>

// Boost
#include <boost/filesystem.hpp>

// STL
#include <string>

using namespace koichi_robotics_lib;

TEST(YamlConfigReader, case0) {
  boost::filesystem::path thisfile(__FILE__);
  boost::filesystem::path yamlfile =
      thisfile.parent_path() / boost::filesystem::path("data/autorun.yaml");

  std::map<unsigned int, file_config> conf_map =
      YamlConfigReader::ReadFileConfig(yamlfile.generic_string());

  std::map<unsigned int, file_config>::const_iterator citr = conf_map.begin();

  ASSERT_EQ(citr->first, 1);
  ASSERT_EQ(citr->second.course_no_, 1);
  ASSERT_EQ(citr->second.map_pcd_file_, "/Desktop/sample_1.pcd");
  ASSERT_EQ(citr->second.lane_csv_file_, "/Desktop/smooth_waypoint_1.csv");
  ASSERT_EQ(citr->second.course_start_yaml_file_, "/course_start_1.yaml");
  ASSERT_EQ(citr->second.world_to_map_json_file_,
            "/Desktop/world_to_map_1.json");

  citr++;
  ASSERT_EQ(citr->first, 2);
  ASSERT_EQ(citr->second.course_no_, 2);
  ASSERT_EQ(citr->second.map_pcd_file_, "/Desktop/sample_2.pcd");
  ASSERT_EQ(citr->second.lane_csv_file_, "/Desktop/smooth_waypoint_2.csv");
  ASSERT_EQ(citr->second.course_start_yaml_file_, "/course_start_2.yaml");
  ASSERT_EQ(citr->second.world_to_map_json_file_,
            "/Desktop/world_to_map_2.json");
}

TEST(YamlConfigReader, case1) {
  boost::filesystem::path thisfile(__FILE__);
  boost::filesystem::path yamlfile =
      thisfile.parent_path() /
      boost::filesystem::path("data/course_start.yaml");

  init_config init_conf =
      YamlConfigReader::ReadInitConfig(yamlfile.generic_string());

  ASSERT_FALSE(init_conf.use_gps_);
  ASSERT_FALSE(init_conf.pose_initializer_);
  ASSERT_FALSE(init_conf.init_via_gnss_);

  ASSERT_DOUBLE_EQ(init_conf.init_pose_.x_, 14.59);
  ASSERT_DOUBLE_EQ(init_conf.init_pose_.y_, 2.17);
  ASSERT_DOUBLE_EQ(init_conf.init_pose_.z_, -0.04);
  ASSERT_DOUBLE_EQ(init_conf.init_pose_.roll_, -0.019271653270274233);
  ASSERT_DOUBLE_EQ(init_conf.init_pose_.pitch_, -0.003258504617599481);
  ASSERT_DOUBLE_EQ(init_conf.init_pose_.yaw_, -0.03945708572714498);

  ASSERT_EQ(init_conf.plane_number_, 9);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}