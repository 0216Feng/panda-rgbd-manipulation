#include <algorithm>
#include <cstddef>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "moveit_msgs/msg/display_trajectory.hpp"
#include "panda_manipulation_cpp/trajectory_quality.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace panda_manipulation_cpp
{
namespace
{

std::string json_string(const std::string & value)
{
  std::ostringstream stream;
  stream << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (character < 0x20) {
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          stream << character;
        }
    }
  }
  stream << '"';
  return stream.str();
}

void append_optional_number(
  std::ostringstream & stream, const std::optional<double> & value)
{
  if (value) {
    stream << *value;
  } else {
    stream << "null";
  }
}

void append_optional_index(
  std::ostringstream & stream, const std::optional<std::size_t> & value)
{
  if (value) {
    stream << *value;
  } else {
    stream << "null";
  }
}

std::string metrics_json(
  const TrajectoryQuality & metrics,
  const std::size_t index,
  const std::vector<std::string> & joint_names)
{
  std::ostringstream stream;
  stream << std::setprecision(10);
  stream << "{\"index\":" << index << ",\"joint_names\":[";
  for (std::size_t joint_index = 0; joint_index < joint_names.size(); ++joint_index) {
    if (joint_index > 0) {
      stream << ',';
    }
    stream << json_string(joint_names[joint_index]);
  }
  stream << "],\"point_count\":" << metrics.point_count
         << ",\"joint_count\":" << metrics.joint_count
         << ",\"invalid_position_segment_count\":" <<
    metrics.invalid_position_segment_count
         << ",\"nonpositive_duration_segment_count\":" <<
    metrics.nonpositive_duration_segment_count
         << ",\"duration_s\":" << metrics.duration_s
         << ",\"min_segment_duration_s\":";
  append_optional_number(stream, metrics.min_segment_duration_s);
  stream << ",\"max_joint_step_rad\":" << metrics.max_joint_step_rad
         << ",\"max_joint_step_joint\":" << json_string(metrics.max_joint_step_joint)
         << ",\"max_implied_velocity_rad_s\":" << metrics.max_implied_velocity_rad_s
         << ",\"max_implied_velocity_joint\":" <<
    json_string(metrics.max_implied_velocity_joint)
         << ",\"max_reported_velocity_rad_s\":" <<
    metrics.max_reported_velocity_rad_s
         << ",\"max_reported_acceleration_rad_s2\":" <<
    metrics.max_reported_acceleration_rad_s2
         << ",\"max_implied_acceleration_rad_s2\":" <<
    metrics.max_implied_acceleration_rad_s2
         << ",\"integrated_squared_acceleration\":" <<
    metrics.integrated_squared_acceleration
         << ",\"joint_path_length_rad\":" << metrics.joint_path_length_rad
         << ",\"endpoint_joint_displacement_rad\":" <<
    metrics.endpoint_joint_displacement_rad
         << ",\"joint_path_tortuosity\":";
  append_optional_number(stream, metrics.joint_path_tortuosity);
  stream << ",\"terminal_max_abs_velocity_rad_s\":";
  append_optional_number(stream, metrics.terminal_max_abs_velocity_rad_s);
  stream << ",\"terminal_max_abs_acceleration_rad_s2\":";
  append_optional_number(stream, metrics.terminal_max_abs_acceleration_rad_s2);
  stream << ",\"min_normalized_joint_limit_margin\":";
  append_optional_number(stream, metrics.min_normalized_joint_limit_margin);
  stream << ",\"min_joint_limit_margin_joint\":" <<
    json_string(metrics.min_joint_limit_margin_joint)
         << ",\"min_joint_limit_margin_point\":";
  append_optional_index(stream, metrics.min_joint_limit_margin_point);
  stream << '}';
  return stream.str();
}

}  // namespace

class TrajectoryMetricsNode : public rclcpp::Node
{
public:
  TrajectoryMetricsNode()
  : Node("trajectory_metrics_node")
  {
    const std::string input_topic =
      declare_parameter<std::string>("input_topic", "/display_planned_path");
    input_topic_ = input_topic;
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "/trajectory_quality_metrics");
    const std::vector<std::string> default_joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"};
    const std::vector<double> default_lower = {
      -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    const std::vector<double> default_upper = {
      2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    const auto joint_names =
      declare_parameter<std::vector<std::string>>("joint_limit_names", default_joint_names);
    const auto lower =
      declare_parameter<std::vector<double>>("joint_limit_lower", default_lower);
    const auto upper =
      declare_parameter<std::vector<double>>("joint_limit_upper", default_upper);
    if (joint_names.size() != lower.size() || joint_names.size() != upper.size()) {
      throw std::invalid_argument(
              "joint_limit_names, joint_limit_lower and joint_limit_upper must have equal lengths");
    }
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      joint_limits_.emplace(joint_names[index], JointLimit{lower[index], upper[index]});
    }

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    publisher_ = create_publisher<std_msgs::msg::String>(output_topic, output_qos);
    subscription_ = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      input_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&TrajectoryMetricsNode::on_trajectory, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "C++17 trajectory metrics: %s -> %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void on_trajectory(const moveit_msgs::msg::DisplayTrajectory::SharedPtr message)
  {
    std::ostringstream payload;
    payload << std::setprecision(10);
    payload << "{\"schema_version\":1,\"source_topic\":" << json_string(input_topic_)
            << ",\"trajectory_count\":" << message->trajectory.size()
            << ",\"metrics\":[";
    double total_duration = 0.0;
    double total_path_length = 0.0;
    double max_joint_step = 0.0;
    double total_smoothness = 0.0;
    for (std::size_t index = 0; index < message->trajectory.size(); ++index) {
      if (index > 0) {
        payload << ',';
      }
      const auto & trajectory = message->trajectory[index].joint_trajectory;
      const auto metrics = compute_trajectory_quality(trajectory, joint_limits_);
      payload << metrics_json(metrics, index, trajectory.joint_names);
      total_duration += metrics.duration_s;
      total_path_length += metrics.joint_path_length_rad;
      max_joint_step = std::max(max_joint_step, metrics.max_joint_step_rad);
      total_smoothness += metrics.integrated_squared_acceleration;
    }
    payload << "],\"aggregate\":{\"duration_s\":" << total_duration
            << ",\"joint_path_length_rad\":" << total_path_length
            << ",\"max_joint_step_rad\":" << max_joint_step
            << ",\"integrated_squared_acceleration\":" << total_smoothness
            << "}}";

    std_msgs::msg::String output;
    output.data = payload.str();
    publisher_->publish(output);
    RCLCPP_INFO(
      get_logger(),
      "Published quality for %zu trajectory segment(s): path=%.4f rad, max_step=%.4f rad",
      message->trajectory.size(), total_path_length, max_joint_step);
  }

  JointLimitMap joint_limits_;
  std::string input_topic_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_;
};

}  // namespace panda_manipulation_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<panda_manipulation_cpp::TrajectoryMetricsNode>());
  rclcpp::shutdown();
  return 0;
}
