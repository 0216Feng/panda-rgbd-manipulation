#ifndef PANDA_MANIPULATION_CPP__TRAJECTORY_QUALITY_HPP_
#define PANDA_MANIPULATION_CPP__TRAJECTORY_QUALITY_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace panda_manipulation_cpp
{

struct JointLimit
{
  double lower;
  double upper;
};

using JointLimitMap = std::unordered_map<std::string, JointLimit>;

struct TrajectoryQuality
{
  std::size_t point_count{0};
  std::size_t joint_count{0};
  std::size_t invalid_position_segment_count{0};
  std::size_t nonpositive_duration_segment_count{0};
  double duration_s{0.0};
  std::optional<double> min_segment_duration_s;
  double max_joint_step_rad{0.0};
  std::string max_joint_step_joint;
  double max_implied_velocity_rad_s{0.0};
  std::string max_implied_velocity_joint;
  double max_reported_velocity_rad_s{0.0};
  double max_reported_acceleration_rad_s2{0.0};
  double max_implied_acceleration_rad_s2{0.0};
  double integrated_squared_acceleration{0.0};
  double joint_path_length_rad{0.0};
  double endpoint_joint_displacement_rad{0.0};
  std::optional<double> joint_path_tortuosity;
  std::optional<double> terminal_max_abs_velocity_rad_s;
  std::optional<double> terminal_max_abs_acceleration_rad_s2;
  std::optional<double> min_normalized_joint_limit_margin;
  std::string min_joint_limit_margin_joint;
  std::optional<std::size_t> min_joint_limit_margin_point;
};

TrajectoryQuality compute_trajectory_quality(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const JointLimitMap & joint_limits = {});

}  // namespace panda_manipulation_cpp

#endif  // PANDA_MANIPULATION_CPP__TRAJECTORY_QUALITY_HPP_
