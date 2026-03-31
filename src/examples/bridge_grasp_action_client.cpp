#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "gripper_ros2/action/grasp.hpp"

using Grasp = gripper_ros2::action::Grasp;
using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<Grasp>;

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
      << "bridge_grasp_action_client usage:\n"
      << "  --ns <bridge_ns> (default /gripper/gripper_ros_bridge)\n"
      << "  --force <grams> (default 180)\n"
      << "  --tol <grams> (default 5)\n"
      << "  --settle <sec> (default 0.5)\n"
      << "  --timeout <sec> (default 8.0)\n"
      << "  --strategy <0|1> (default 0)\n"
      << "  --feedback <true|false> (default true)\n"
      << "  --server-timeout <sec> (default 3.0)\n"
      << "  --result-timeout <sec> (default 20.0)\n";
}

}  // namespace

int main(int argc, char ** argv) {
  std::string ns = "/gripper/gripper_ros_bridge";
  int force = 180;
  int tol = 5;
  double settle = 0.5;
  double timeout = 8.0;
  int control_strategy = 0;
  bool print_feedback = true;
  double server_timeout = 3.0;
  double result_timeout = 20.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ns" && i + 1 < argc) {
      ns = argv[++i];
    } else if (arg == "--force" && i + 1 < argc) {
      force = std::stoi(argv[++i]);
    } else if (arg == "--tol" && i + 1 < argc) {
      tol = std::stoi(argv[++i]);
    } else if (arg == "--settle" && i + 1 < argc) {
      settle = std::stod(argv[++i]);
    } else if (arg == "--timeout" && i + 1 < argc) {
      timeout = std::stod(argv[++i]);
    } else if (arg == "--strategy" && i + 1 < argc) {
      control_strategy = std::stoi(argv[++i]);
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
  auto node = std::make_shared<rclcpp::Node>("bridge_grasp_action_client");

  const auto action_name = endpoint(ns, "grasp");
  auto client = rclcpp_action::create_client<Grasp>(node, action_name);
  if (!client->wait_for_action_server(std::chrono::duration<double>(server_timeout))) {
    RCLCPP_ERROR(node->get_logger(), "Grasp action server not available: %s", action_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  Grasp::Goal goal;
  goal.target_force_grams = force;
  goal.tolerance_grams = static_cast<float>(tol);
  goal.settle_time_sec = static_cast<float>(settle);
  goal.timeout_sec = static_cast<float>(timeout);
  goal.control_strategy = static_cast<uint8_t>(control_strategy);

  rclcpp_action::Client<Grasp>::SendGoalOptions goal_options;
  goal_options.feedback_callback = [node, print_feedback](
      GoalHandleGrasp::SharedPtr,
      const std::shared_ptr<const Grasp::Feedback> feedback) {
    if (!print_feedback) {
      return;
    }
    RCLCPP_INFO(
        node->get_logger(),
        "[grasp feedback] target_force=%.3f avg_force=%.3f error=%.3f state=%u",
        feedback->actual_target_force_grams,
        feedback->current_avg_force_grams,
        feedback->force_error_grams,
        static_cast<unsigned>(feedback->current_state));
  };

  auto goal_handle_future = client->async_send_goal(goal, goal_options);
  const auto goal_rc = rclcpp::spin_until_future_complete(
      node, goal_handle_future, std::chrono::duration<double>(server_timeout));
  if (goal_rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed sending grasp goal");
    rclcpp::shutdown();
    return 2;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Grasp goal rejected");
    rclcpp::shutdown();
    return 3;
  }

  auto result_future = client->async_get_result(goal_handle);
  const auto result_rc = rclcpp::spin_until_future_complete(
      node, result_future, std::chrono::duration<double>(result_timeout));
  if (result_rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for grasp result");
    rclcpp::shutdown();
    return 4;
  }

  const auto wrapped = result_future.get();
  const bool success = wrapped.result ? wrapped.result->success : false;
  const double achieved = wrapped.result ? wrapped.result->final_avg_force_grams : 0.0;
  const int final_state = wrapped.result ? static_cast<int>(wrapped.result->final_state) : -1;
  const std::string message = wrapped.result ? wrapped.result->message : "<no result payload>";

  RCLCPP_INFO(
      node->get_logger(),
      "Grasp finished with code=%d success=%s final_state=%d final_avg_force=%.2f message=%s",
      static_cast<int>(wrapped.code), success ? "true" : "false", final_state, achieved,
      message.c_str());

  rclcpp::shutdown();
  return success ? 0 : 5;
}
