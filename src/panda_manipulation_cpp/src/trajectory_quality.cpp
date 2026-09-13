#include "panda_manipulation_cpp/trajectory_quality.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace panda_manipulation_cpp
{
namespace
{

double time_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

std::optional<double> max_finite_magnitude(const std::vector<double> & values)
{
  std::optional<double> result;
  for (const double value : values) {
    if (!std::isfinite(value)) {
      continue;
    }
    const double magnitude = std::abs(value);
    result = result ? std::max(*result, magnitude) : magnitude;
  }
  return result;
}

bool complete_finite_positions(
  const trajectory_msgs::msg::JointTrajectoryPoint & point,
  const std::size_t joint_count)
{
  if (point.positions.size() < joint_count) {
    return false;
  }
  return std::all_of(
    point.positions.begin(), point.positions.begin() + joint_count,
    [](const double value) {return std::isfinite(value);});
}

}  // namespace

TrajectoryQuality compute_trajectory_quality(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const JointLimitMap & joint_limits)
{
  TrajectoryQuality metrics;
  metrics.point_count = trajectory.points.size();
  metrics.joint_count = trajectory.joint_names.size();
  if (trajectory.points.empty()) {
    return metrics;
  }

  const auto & points = trajectory.points;
  const std::size_t joint_count = trajectory.joint_names.size();
  metrics.duration_s = time_seconds(points.back().time_from_start);

  std::vector<std::vector<double>> implied_velocities;
  std::vector<double> segment_durations;
  std::vector<std::size_t> implied_velocity_segment_end_indices;
  for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
    const auto reported_velocity = max_finite_magnitude(points[point_index].velocities);
    if (reported_velocity) {
      metrics.max_reported_velocity_rad_s =
        std::max(metrics.max_reported_velocity_rad_s, *reported_velocity);
    }
    const auto reported_acceleration =
      max_finite_magnitude(points[point_index].accelerations);
    if (reported_acceleration) {
      metrics.max_reported_acceleration_rad_s2 =
        std::max(metrics.max_reported_acceleration_rad_s2, *reported_acceleration);
    }

    if (complete_finite_positions(points[point_index], joint_count)) {
      for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
        const auto limit = joint_limits.find(trajectory.joint_names[joint_index]);
        if (limit == joint_limits.end()) {
          continue;
        }
        const double range = limit->second.upper - limit->second.lower;
        if (!(range > 0.0) || !std::isfinite(range)) {
          continue;
        }
        const double position = points[point_index].positions[joint_index];
        const double margin = std::min(
          position - limit->second.lower,
          limit->second.upper - position) / range;
        if (!metrics.min_normalized_joint_limit_margin ||
          margin < *metrics.min_normalized_joint_limit_margin)
        {
          metrics.min_normalized_joint_limit_margin = margin;
          metrics.min_joint_limit_margin_joint = trajectory.joint_names[joint_index];
          metrics.min_joint_limit_margin_point = point_index;
        }
      }
    }

    if (point_index == 0) {
      continue;
    }

    const double segment_duration =
      time_seconds(points[point_index].time_from_start) -
      time_seconds(points[point_index - 1].time_from_start);
    if (segment_duration > 0.0 && std::isfinite(segment_duration)) {
      metrics.min_segment_duration_s = metrics.min_segment_duration_s ?
        std::min(*metrics.min_segment_duration_s, segment_duration) : segment_duration;
    }

    if (!complete_finite_positions(points[point_index - 1], joint_count) ||
      !complete_finite_positions(points[point_index], joint_count))
    {
      ++metrics.invalid_position_segment_count;
      continue;
    }

    double squared_segment_length = 0.0;
    std::vector<double> segment_velocity(joint_count, 0.0);
    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      const double delta =
        points[point_index].positions[joint_index] -
        points[point_index - 1].positions[joint_index];
      const double step = std::abs(delta);
      squared_segment_length += delta * delta;
      if (step > metrics.max_joint_step_rad) {
        metrics.max_joint_step_rad = step;
        metrics.max_joint_step_joint = trajectory.joint_names[joint_index];
      }
      if (segment_duration > 0.0 && std::isfinite(segment_duration)) {
        const double implied_velocity = delta / segment_duration;
        segment_velocity[joint_index] = implied_velocity;
        if (std::abs(implied_velocity) > metrics.max_implied_velocity_rad_s) {
          metrics.max_implied_velocity_rad_s = std::abs(implied_velocity);
          metrics.max_implied_velocity_joint = trajectory.joint_names[joint_index];
        }
      }
    }
    metrics.joint_path_length_rad += std::sqrt(squared_segment_length);
    if (segment_duration > 0.0 && std::isfinite(segment_duration)) {
      implied_velocities.push_back(std::move(segment_velocity));
      segment_durations.push_back(segment_duration);
      implied_velocity_segment_end_indices.push_back(point_index);
    } else {
      ++metrics.nonpositive_duration_segment_count;
    }
  }

  if (points.size() >= 2 &&
    complete_finite_positions(points.front(), joint_count) &&
    complete_finite_positions(points.back(), joint_count))
  {
    double squared_displacement = 0.0;
    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      const double delta =
        points.back().positions[joint_index] - points.front().positions[joint_index];
      squared_displacement += delta * delta;
    }
    metrics.endpoint_joint_displacement_rad = std::sqrt(squared_displacement);
    if (metrics.endpoint_joint_displacement_rad > 1e-9) {
      metrics.joint_path_tortuosity =
        metrics.joint_path_length_rad / metrics.endpoint_joint_displacement_rad;
    }
  }

  for (std::size_t segment_index = 1;
    segment_index < implied_velocities.size(); ++segment_index)
  {
    if (implied_velocity_segment_end_indices[segment_index] !=
      implied_velocity_segment_end_indices[segment_index - 1] + 1)
    {
      continue;
    }
    const double acceleration_duration =
      0.5 * (segment_durations[segment_index - 1] + segment_durations[segment_index]);
    if (!(acceleration_duration > 0.0)) {
      continue;
    }
    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      const double acceleration =
        (implied_velocities[segment_index][joint_index] -
        implied_velocities[segment_index - 1][joint_index]) /
        acceleration_duration;
      metrics.max_implied_acceleration_rad_s2 =
        std::max(metrics.max_implied_acceleration_rad_s2, std::abs(acceleration));
      metrics.integrated_squared_acceleration +=
        acceleration * acceleration * acceleration_duration;
    }
  }

  metrics.terminal_max_abs_velocity_rad_s =
    max_finite_magnitude(points.back().velocities);
  metrics.terminal_max_abs_acceleration_rad_s2 =
    max_finite_magnitude(points.back().accelerations);
  return metrics;
}

}  // namespace panda_manipulation_cpp
