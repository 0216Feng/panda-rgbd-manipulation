#include <cstdint>
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "panda_manipulation_cpp/trajectory_quality.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace panda_manipulation_cpp
{
namespace
{

trajectory_msgs::msg::JointTrajectoryPoint point(
  const double time_s,
  const std::vector<double> & positions,
  const std::vector<double> & velocities = {},
  const std::vector<double> & accelerations = {})
{
  trajectory_msgs::msg::JointTrajectoryPoint result;
  result.positions = positions;
  result.velocities = velocities;
  result.accelerations = accelerations;
  result.time_from_start.sec = static_cast<std::int32_t>(std::floor(time_s));
  result.time_from_start.nanosec = static_cast<std::uint32_t>(
    std::llround((time_s - std::floor(time_s)) * 1e9));
  return result;
}

TEST(TrajectoryQuality, EmptyTrajectoryHasExplicitZeroCounts)
{
  const trajectory_msgs::msg::JointTrajectory trajectory;
  const auto metrics = compute_trajectory_quality(trajectory);
  EXPECT_EQ(metrics.point_count, 0U);
  EXPECT_EQ(metrics.joint_count, 0U);
  EXPECT_DOUBLE_EQ(metrics.joint_path_length_rad, 0.0);
  EXPECT_FALSE(metrics.joint_path_tortuosity.has_value());
  EXPECT_EQ(metrics.invalid_position_segment_count, 0U);
  EXPECT_EQ(metrics.nonpositive_duration_segment_count, 0U);
}

TEST(TrajectoryQuality, MatchesReferenceContinuityMetrics)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"panda_joint1", "panda_joint2"};
  trajectory.points = {
    point(0.1, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}),
    point(0.3, {0.02, -0.04}, {0.1, -0.2}, {0.3, -0.4}),
    point(0.8, {0.03, -0.05}, {0.0, 0.01}, {0.0, -0.02}),
  };

  const auto metrics = compute_trajectory_quality(trajectory);
  EXPECT_EQ(metrics.point_count, 3U);
  EXPECT_EQ(metrics.joint_count, 2U);
  EXPECT_NEAR(metrics.duration_s, 0.8, 1e-9);
  ASSERT_TRUE(metrics.min_segment_duration_s.has_value());
  EXPECT_NEAR(*metrics.min_segment_duration_s, 0.2, 1e-9);
  EXPECT_NEAR(metrics.max_joint_step_rad, 0.04, 1e-9);
  EXPECT_EQ(metrics.max_joint_step_joint, "panda_joint2");
  EXPECT_NEAR(metrics.max_implied_velocity_rad_s, 0.2, 1e-9);
  EXPECT_EQ(metrics.max_implied_velocity_joint, "panda_joint2");
  EXPECT_NEAR(metrics.max_reported_velocity_rad_s, 0.2, 1e-9);
  EXPECT_NEAR(metrics.max_reported_acceleration_rad_s2, 0.4, 1e-9);
  EXPECT_NEAR(
    metrics.joint_path_length_rad,
    std::sqrt(0.02 * 0.02 + 0.04 * 0.04) + std::sqrt(0.01 * 0.01 + 0.01 * 0.01),
    1e-9);
  EXPECT_NEAR(
    metrics.endpoint_joint_displacement_rad,
    std::sqrt(0.03 * 0.03 + 0.05 * 0.05), 1e-9);
  ASSERT_TRUE(metrics.joint_path_tortuosity.has_value());
  EXPECT_GE(*metrics.joint_path_tortuosity, 1.0);
  EXPECT_GT(metrics.integrated_squared_acceleration, 0.0);
  EXPECT_GT(metrics.max_implied_acceleration_rad_s2, 0.0);
  ASSERT_TRUE(metrics.terminal_max_abs_velocity_rad_s.has_value());
  EXPECT_NEAR(*metrics.terminal_max_abs_velocity_rad_s, 0.01, 1e-9);
  ASSERT_TRUE(metrics.terminal_max_abs_acceleration_rad_s2.has_value());
  EXPECT_NEAR(*metrics.terminal_max_abs_acceleration_rad_s2, 0.02, 1e-9);
}

TEST(TrajectoryQuality, ReportsNormalizedJointLimitMarginAndViolation)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"panda_joint1", "panda_joint2"};
  trajectory.points = {
    point(0.0, {0.0, 0.0}),
    point(1.0, {0.8, 1.1}),
  };
  const JointLimitMap limits = {
    {"panda_joint1", {-1.0, 1.0}},
    {"panda_joint2", {-1.0, 1.0}},
  };

  const auto metrics = compute_trajectory_quality(trajectory, limits);
  ASSERT_TRUE(metrics.min_normalized_joint_limit_margin.has_value());
  EXPECT_NEAR(*metrics.min_normalized_joint_limit_margin, -0.05, 1e-9);
  EXPECT_EQ(metrics.min_joint_limit_margin_joint, "panda_joint2");
  ASSERT_TRUE(metrics.min_joint_limit_margin_point.has_value());
  EXPECT_EQ(*metrics.min_joint_limit_margin_point, 1U);
}

TEST(TrajectoryQuality, IgnoresIncompleteAndNonFinitePositionSegments)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"panda_joint1", "panda_joint2"};
  trajectory.points = {
    point(0.0, {0.0, 0.0}),
    point(0.5, {0.1}),
    point(1.0, {std::numeric_limits<double>::quiet_NaN(), 0.2}),
  };

  const auto metrics = compute_trajectory_quality(trajectory);
  EXPECT_DOUBLE_EQ(metrics.joint_path_length_rad, 0.0);
  EXPECT_DOUBLE_EQ(metrics.max_joint_step_rad, 0.0);
  EXPECT_FALSE(metrics.joint_path_tortuosity.has_value());
  EXPECT_EQ(metrics.invalid_position_segment_count, 2U);
}

TEST(TrajectoryQuality, DoesNotBridgeAccelerationAcrossInvalidTiming)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"panda_joint1"};
  trajectory.points = {
    point(0.0, {0.0}),
    point(1.0, {1.0}),
    point(0.5, {1.5}),
    point(1.5, {1.5}),
  };

  const auto metrics = compute_trajectory_quality(trajectory);
  EXPECT_EQ(metrics.nonpositive_duration_segment_count, 1U);
  EXPECT_DOUBLE_EQ(metrics.max_implied_acceleration_rad_s2, 0.0);
  EXPECT_DOUBLE_EQ(metrics.integrated_squared_acceleration, 0.0);
  EXPECT_NEAR(metrics.joint_path_length_rad, 1.5, 1e-9);
}

}  // namespace
}  // namespace panda_manipulation_cpp
