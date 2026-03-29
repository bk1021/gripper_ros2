#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "gripper_ros2/action/calibrate_force.hpp"

using Calibrate = gripper_ros2::action::CalibrateForce;
using GoalHandleCalib = rclcpp_action::ClientGoalHandle<Calibrate>;

namespace {

std::string endpoint(std::string ns, const std::string & leaf) {
  if (ns.empty()) {
    ns = "/";
  }
  if (ns.back() == '/') {
    ns.pop_back();
  }
  if (ns.empty()) {
    return "/" + leaf;
  }
  if (ns.front() != '/') {
    ns = "/" + ns;
  }
  return ns + "/" + leaf;
}

void print_usage() {
  std::cout
      << "bridge_calibrate_action_client usage:\n"
      << "  --ns <bridge_ns> (default /gripper/gripper_ros_bridge)\n"
      << "  --server-timeout <sec> (default 3.0)\n"
      << "  --result-timeout <sec> (default 30.0)\n";
}

}  // namespace

int main(int argc, char ** argv) {
  std::string ns = "/gripper/gripper_ros_bridge";
  double server_timeout = 3.0;
  double result_timeout = 30.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ns" && i + 1 < argc) {
      ns = argv[++i];
    } else if (arg == "--server-timeout" && i + 1 < argc) {
      server_timeout = std::stod(argv[++i]);
    } else if (arg == "--result-timeout" && i + 1 < argc) {
      result_timeout = std::stod(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    }
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bridge_calibrate_action_client");

  const auto action_name = endpoint(ns, "calibrate_force");
  auto client = rclcpp_action::create_client<Calibrate>(node, action_name);
  if (!client->wait_for_action_server(std::chrono::duration<double>(server_timeout))) {
    RCLCPP_ERROR(
        node->get_logger(), "Calibrate action server not available: %s", action_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  Calibrate::Goal goal;

  auto goal_handle_future = client->async_send_goal(goal);
  const auto goal_rc = rclcpp::spin_until_future_complete(
      node, goal_handle_future, std::chrono::duration<double>(server_timeout));
  if (goal_rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed sending calibrate goal");
    rclcpp::shutdown();
    return 2;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Calibrate goal rejected");
    rclcpp::shutdown();
    return 3;
  }

  auto result_future = client->async_get_result(goal_handle);
  const auto result_rc = rclcpp::spin_until_future_complete(
      node, result_future, std::chrono::duration<double>(result_timeout));
  if (result_rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for calibrate result");
    rclcpp::shutdown();
    return 4;
  }

  const auto wrapped = result_future.get();
  const bool success = wrapped.result ? wrapped.result->success : false;
  const float threshold = wrapped.result ? wrapped.result->contact_threshold_result : 0.0f;
  const float pc3_zero = wrapped.result ? wrapped.result->pc3_zero_adc : 0.0f;
  const float pc2_zero = wrapped.result ? wrapped.result->pc2_zero_adc : 0.0f;
  const std::string message = wrapped.result ? wrapped.result->message : "<no result payload>";

  RCLCPP_INFO(
      node->get_logger(),
      "Calibrate finished with code=%d success=%s threshold=%.2f pc3_zero=%.2f pc2_zero=%.2f message=%s",
      static_cast<int>(wrapped.code), success ? "true" : "false", threshold, pc3_zero, pc2_zero,
      message.c_str());

  rclcpp::shutdown();
  return success ? 0 : 5;
}
