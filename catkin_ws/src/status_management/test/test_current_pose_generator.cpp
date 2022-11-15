
#include <status_management/current_pose_generator.h>

// Source file
#include "current_pose_generator.cpp"

// Eigen
#include <Eigen/Geometry>
#include <Eigen/SVD>

// Google
#include <glog/logging.h>
#include <gtest/gtest.h>

using namespace koichi_robotics_lib;

Eigen::Matrix3d CreateRotationMatrixEulerXYZ(const double rx, const double ry,
                                             const double rz) {
  return (Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()))
      .toRotationMatrix();
}

TEST(AverageTranslations, case0) {
  std::vector<Eigen::Vector3d,
              Eigen::aligned_allocator<std::pair<const int, Eigen::Vector4d>>>
      translations;
  translations.push_back(Eigen::Vector3d(0, 0, 0));
  translations.push_back(Eigen::Vector3d(0, 10, 100));
  translations.push_back(Eigen::Vector3d(0, 20, 200));
  translations.push_back(Eigen::Vector3d(0, 30, 300));

  Eigen::Vector3d v = AverageTranslations(translations);

  ASSERT_TRUE(v.isApprox(Eigen::Vector3d(0, 15.0, 150.0)));
}

TEST(AverageRotations, case0) {
  std::vector<Eigen::Matrix3d> Rs;

  Rs.push_back(CreateRotationMatrixEulerXYZ(
      -30.0 / 180.0 * M_PI, 0 / 180.0 * M_PI, 0 / 180.0 * M_PI));
  Rs.push_back(CreateRotationMatrixEulerXYZ(
      30.0 / 180.0 * M_PI, 0 / 180.0 * M_PI, 0 / 180.0 * M_PI));

  double PRES = 1e-6;
  Eigen::Matrix3d R_filt = AverageRotations(Rs);
  Eigen::Vector3d euler(2, 3);
  euler = R_filt.eulerAngles(0, 1, 2);
  EXPECT_NEAR(euler(0), 0.0 / 180.0 * M_PI, PRES);
  EXPECT_NEAR(euler(1), 0.0 / 180.0 * M_PI, PRES);
  EXPECT_NEAR(euler(2), 0.0 / 180.0 * M_PI, PRES);
  // ASSERT_EQ(eulers(1, 0), 0);
  // ASSERT_EQ(eulers(1, 1), 0);
  // ASSERT_EQ(eulers(1, 2), 0);

  // ASSERT_TRUE(R.isApprox(aq.toRotationMatrix()));
}

TEST(AverageRotations, case1) {
  std::vector<Eigen::Matrix3d> Rs;

  Rs.push_back(CreateRotationMatrixEulerXYZ(
      0.0 / 180.0 * M_PI, 0 / 180.0 * M_PI, 0 / 180.0 * M_PI));
  Rs.push_back(CreateRotationMatrixEulerXYZ(
      30.0 / 180.0 * M_PI, 0 / 180.0 * M_PI, 0 / 180.0 * M_PI));

  double PRES = 1e-6;
  Eigen::Matrix3d R_filt = AverageRotations(Rs);
  Eigen::Vector3d euler(2, 3);
  euler = R_filt.eulerAngles(0, 1, 2);
  EXPECT_NEAR(euler(0), 15.0 / 180.0 * M_PI, PRES);
  EXPECT_NEAR(euler(1), 0.0 / 180.0 * M_PI, PRES);
  EXPECT_NEAR(euler(2), 0.0 / 180.0 * M_PI, PRES);
  // ASSERT_EQ(eulers(1, 0), 0);
  // ASSERT_EQ(eulers(1, 1), 0);
  // ASSERT_EQ(eulers(1, 2), 0);

  // ASSERT_TRUE(R.isApprox(aq.toRotationMatrix()));
}

TEST(AverageRotations, case2) {
  std::vector<Eigen::Matrix3d> Rs;

  Rs.push_back(CreateRotationMatrixEulerXYZ(
      0.0 / 180.0 * M_PI, 0 / 180.0 * M_PI, 0 / 180.0 * M_PI));
  Rs.push_back(CreateRotationMatrixEulerXYZ(
      179.0 / 180.0 * M_PI, 0 / 180.0 * M_PI, 0 / 180.0 * M_PI));

  double PRES = 1e-6;
  Eigen::Matrix3d R_filt = AverageRotations(Rs);
  Eigen::Vector3d euler(2, 3);
  euler = R_filt.eulerAngles(0, 1, 2);

  LOG(INFO) << "Euler 0 : " << euler(0);
  LOG(INFO) << "Euler 1 : " << euler(1);
  LOG(INFO) << "Euler 2 : " << euler(2);

  EXPECT_NEAR(euler(0), 89.5 / 180.0 * M_PI, PRES);
  EXPECT_NEAR(euler(1), 0.0 / 180.0 * M_PI, PRES);
  EXPECT_NEAR(euler(2), 0.0 / 180.0 * M_PI, PRES);
  // ASSERT_EQ(eulers(1, 0), 0);
  // ASSERT_EQ(eulers(1, 1), 0);
  // ASSERT_EQ(eulers(1, 2), 0);

  // ASSERT_TRUE(R.isApprox(aq.toRotationMatrix()));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}