#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

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
      << "bridge_force_topic_example_node usage:\n"
      << "  --ns <bridge_ns> (default /gripper/gripper_ros_bridge)\n"
      << "  --force <float> (default 200.0)\n"
      << "  --seconds <float> publish duration (default 2.0)\n"
      << "  --hz <float> publish rate (default 20.0)\n";
}

}  // namespace

int main(int argc, char ** argv) {
  std::string ns = "/gripper/gripper_ros_bridge";
  float force = 200.0f;
  double seconds = 2.0;
  double hz = 20.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ns" && i + 1 < argc) {
      ns = argv[++i];
    } else if (arg == "--force" && i + 1 < argc) {
      force = std::stof(argv[++i]);
    } else if (arg == "--seconds" && i + 1 < argc) {
      seconds = std::stod(argv[++i]);
    } else if (arg == "--hz" && i + 1 < argc) {
      hz = std::stod(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    }
  }

  if (hz <= 0.0) {
    std::cerr << "hz must be > 0\n";
    return 2;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bridge_force_topic_example_node");
  const std::string topic = endpoint(ns, "target_force");
  auto pub = node->create_publisher<std_msgs::msg::Float32>(topic, 10);

  RCLCPP_INFO(
      node->get_logger(), "Publishing force %.2f g to %s for %.2fs at %.2f Hz",
      force, topic.c_str(), seconds, hz);

  std_msgs::msg::Float32 msg;
  msg.data = force;

  rclcpp::WallRate rate(hz);
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (elapsed >= seconds) {
      break;
    }
    pub->publish(msg);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
