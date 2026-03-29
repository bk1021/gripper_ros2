#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "gripper_ros2/action/open.hpp"

using Open = gripper_ros2::action::Open;
using GoalHandleOpen = rclcpp_action::ClientGoalHandle<Open>;

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

bool parse_bool(const std::string & s) {
  return s == "1" || s == "true" || s == "TRUE" || s == "yes" || s == "on";
}

void print_usage() {
  std::cout
      << "bridge_open_action_client usage:\n"
      << "  --ns <bridge_ns> (default /gripper/gripper_ros_bridge)\n"
      << "  --wait <true|false> (default true)\n"
  << "  --feedback <true|false> (default true)\n"
      << "  --server-timeout <sec> (default 3.0)\n"
      << "  --result-timeout <sec> (default 20.0)\n";
}

}  // namespace

int main(int argc, char ** argv) {
  std::string ns = "/gripper/gripper_ros_bridge";
  bool wait_for_completion = true;
  bool print_feedback = true;
  double server_timeout = 3.0;
  double result_timeout = 20.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ns" && i + 1 < argc) {
      ns = argv[++i];
    } else if (arg == "--wait" && i + 1 < argc) {
      wait_for_completion = parse_bool(argv[++i]);
    } else if (arg == "--feedback" && i + 1 < argc) {
      print_feedback = parse_bool(argv[++i]);
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
  auto node = std::make_shared<rclcpp::Node>("bridge_open_action_client");

  const auto action_name = endpoint(ns, "open");
  auto client = rclcpp_action::create_client<Open>(node, action_name);
  if (!client->wait_for_action_server(std::chrono::duration<double>(server_timeout))) {
    RCLCPP_ERROR(node->get_logger(), "Open action server not available: %s", action_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  Open::Goal goal;
  goal.wait_for_completion = wait_for_completion;

  rclcpp_action::Client<Open>::SendGoalOptions goal_options;
  goal_options.feedback_callback = [node, print_feedback](
      GoalHandleOpen::SharedPtr,
      const std::shared_ptr<const Open::Feedback> feedback) {
    if (!print_feedback) {
      return;
    }
    RCLCPP_INFO(
        node->get_logger(),
        "[open feedback] current_position_rad=%.5f distance_to_open_rad=%.5f",
        feedback->current_position_rad,
        feedback->distance_to_open_rad);
  };

  auto goal_handle_future = client->async_send_goal(goal, goal_options);
  const auto goal_rc = rclcpp::spin_until_future_complete(
      node, goal_handle_future, std::chrono::duration<double>(server_timeout));
  if (goal_rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed sending open goal");
    rclcpp::shutdown();
    return 2;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Open goal rejected");
    rclcpp::shutdown();
    return 3;
  }

  auto result_future = client->async_get_result(goal_handle);
  const auto result_rc = rclcpp::spin_until_future_complete(
      node, result_future, std::chrono::duration<double>(result_timeout));
  if (result_rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for open result");
    rclcpp::shutdown();
    return 4;
  }

  const auto wrapped = result_future.get();
  const bool success = wrapped.result ? wrapped.result->success : false;
  const std::string message = wrapped.result ? wrapped.result->message : "<no result payload>";

  RCLCPP_INFO(
      node->get_logger(), "Open finished with code=%d success=%s message=%s",
      static_cast<int>(wrapped.code), success ? "true" : "false", message.c_str());

  rclcpp::shutdown();
  return success ? 0 : 5;
}
