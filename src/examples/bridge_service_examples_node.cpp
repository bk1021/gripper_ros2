#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gripper_ros2/gripper_constants.hpp"
#include "gripper_ros2/srv/get_param.hpp"
#include "gripper_ros2/srv/set_float32.hpp"
#include "gripper_ros2/srv/set_param.hpp"
#include "gripper_ros2/srv/set_state.hpp"

using namespace std::chrono_literals;

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

template <typename ServiceT, typename FillFn>
std::shared_ptr<typename ServiceT::Response> call_service(
    const rclcpp::Node::SharedPtr & node,
    const std::string & service_name,
    FillFn fill,
    double service_wait_sec = 2.0,
    double call_timeout_sec = 3.0) {
  auto client = node->create_client<ServiceT>(service_name);
  if (!client->wait_for_service(std::chrono::duration<double>(service_wait_sec))) {
    RCLCPP_ERROR(node->get_logger(), "Service not available: %s", service_name.c_str());
    return nullptr;
  }

  auto req = std::make_shared<typename ServiceT::Request>();
  fill(*req);

  auto fut = client->async_send_request(req);
  const auto rc = rclcpp::spin_until_future_complete(
      node, fut, std::chrono::duration<double>(call_timeout_sec));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Service call timeout/failure: %s", service_name.c_str());
    return nullptr;
  }

  return fut.get();
}

void print_usage() {
  std::cout
      << "bridge_service_examples_node usage:\n"
      << "  --mode clear_idle|param_io|force_service|manual_step|config_ops\n"
      << "  --ns <bridge_ns> (default /gripper/gripper_ros_bridge)\n"
      << "  --param-id <uint8> (for param_io, default 18)\n"
      << "  --value <float> (for param_io, default 20.0)\n"
      << "  --force <float> (for force_service, default 150.0)\n"
      << "  --position <float> (for manual_step, default -0.8)\n";
}

}  // namespace

int main(int argc, char ** argv) {
  std::string ns = "/gripper/gripper_ros_bridge";
  std::string mode = "clear_idle";
  int param_id = 18;
  float value = 20.0f;
  float force = 150.0f;
  float position = -0.8f;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ns" && i + 1 < argc) {
      ns = argv[++i];
    } else if (arg == "--mode" && i + 1 < argc) {
      mode = argv[++i];
    } else if (arg == "--param-id" && i + 1 < argc) {
      param_id = std::stoi(argv[++i]);
    } else if (arg == "--value" && i + 1 < argc) {
      value = std::stof(argv[++i]);
    } else if (arg == "--force" && i + 1 < argc) {
      force = std::stof(argv[++i]);
    } else if (arg == "--position" && i + 1 < argc) {
      position = std::stof(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    }
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bridge_service_examples_node");
  RCLCPP_INFO(node->get_logger(), "Using bridge namespace: %s", ns.c_str());

  bool ok = true;

  if (mode == "clear_idle") {
    auto clear_res = call_service<std_srvs::srv::Trigger>(
        node, endpoint(ns, "clear_fault"), [](auto &) {});
    if (!clear_res) {
      ok = false;
    } else {
      RCLCPP_INFO(
          node->get_logger(), "clear_fault: success=%s message=%s",
          clear_res->success ? "true" : "false", clear_res->message.c_str());
      ok = ok && clear_res->success;
    }

    auto state_res = call_service<gripper_ros2::srv::SetState>(
        node, endpoint(ns, "set_state"), [](auto & req) {
          req.target_state = static_cast<uint8_t>(gripper::State::IDLE);
        });
    if (!state_res) {
      ok = false;
    } else {
      RCLCPP_INFO(
          node->get_logger(), "set_state(IDLE): success=%s message=%s",
          state_res->success ? "true" : "false", state_res->message.c_str());
      ok = ok && state_res->success;
    }
  } else if (mode == "param_io") {
    auto set_res = call_service<gripper_ros2::srv::SetParam>(
        node, endpoint(ns, "set_param"), [param_id, value](auto & req) {
          req.param_id = static_cast<uint8_t>(param_id);
          req.value = value;
        });
    if (!set_res) {
      ok = false;
    } else {
      RCLCPP_INFO(
          node->get_logger(),
          "set_param(id=%d): success=%s confirmed=%.5f message=%s",
          param_id, set_res->success ? "true" : "false", set_res->confirmed_value,
          set_res->message.c_str());
      ok = ok && set_res->success;
    }

    auto get_res = call_service<gripper_ros2::srv::GetParam>(
        node, endpoint(ns, "get_param"), [param_id](auto & req) {
          req.param_id = static_cast<uint8_t>(param_id);
        });
    if (!get_res) {
      ok = false;
    } else {
      RCLCPP_INFO(
          node->get_logger(), "get_param(id=%d): success=%s value=%.5f message=%s",
          param_id, get_res->success ? "true" : "false", get_res->value,
          get_res->message.c_str());
      ok = ok && get_res->success;
    }
  } else if (mode == "force_service") {
    auto force_res = call_service<gripper_ros2::srv::SetFloat32>(
        node, endpoint(ns, "set_target_force"), [force](auto & req) { req.value = force; });
    if (!force_res) {
      ok = false;
    } else {
      RCLCPP_INFO(
          node->get_logger(), "set_target_force(%.2f): success=%s message=%s",
          force, force_res->success ? "true" : "false", force_res->message.c_str());
      ok = ok && force_res->success;
    }
  } else if (mode == "manual_step") {
    auto mode_res = call_service<gripper_ros2::srv::SetState>(
        node, endpoint(ns, "set_state"), [](auto & req) {
          req.target_state = static_cast<uint8_t>(gripper::State::MANUAL_TUNE);
        });
    if (!mode_res || !mode_res->success) {
      ok = false;
    }

    auto step_res = call_service<gripper_ros2::srv::SetFloat32>(
        node, endpoint(ns, "set_manual_position"), [position](auto & req) {
          req.value = position;
        });
    if (!step_res) {
      ok = false;
    } else {
      RCLCPP_INFO(
          node->get_logger(), "set_manual_position(%.3f): success=%s message=%s",
          position, step_res->success ? "true" : "false", step_res->message.c_str());
      ok = ok && step_res->success;
    }
  } else if (mode == "config_ops") {
    auto fetch_res = call_service<std_srvs::srv::Trigger>(
        node, endpoint(ns, "fetch_config"), [](auto &) {});
    if (!fetch_res) {
      ok = false;
    } else {
      RCLCPP_INFO(
          node->get_logger(), "fetch_config: success=%s message=%s",
          fetch_res->success ? "true" : "false", fetch_res->message.c_str());
      ok = ok && fetch_res->success;
    }

    auto save_res = call_service<std_srvs::srv::Trigger>(
        node, endpoint(ns, "save_config"), [](auto &) {});
    if (!save_res) {
      ok = false;
    } else {
      RCLCPP_INFO(
          node->get_logger(), "save_config: success=%s message=%s",
          save_res->success ? "true" : "false", save_res->message.c_str());
      ok = ok && save_res->success;
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Unknown mode: %s", mode.c_str());
    print_usage();
    ok = false;
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
