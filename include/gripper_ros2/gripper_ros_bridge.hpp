#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "gripper_ros2/msg/gripper_status.hpp"
#include "gripper_ros2/msg/gripper_motor_state.hpp"
#include "gripper_ros2/msg/gripper_config.hpp"
#include "gripper_ros2/srv/set_param.hpp"
#include "gripper_ros2/srv/get_param.hpp"
#include "gripper_ros2/srv/set_state.hpp"
#include "gripper_ros2/srv/set_float32.hpp"
#include "gripper_ros2/action/grasp.hpp"
#include "gripper_ros2/action/open.hpp"
#include "gripper_ros2/action/calibrate_force.hpp"

#include "gripper_ros2/serial_transport.hpp"

#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

class GripperROSBridge : public rclcpp::Node {
public:
  GripperROSBridge();
  ~GripperROSBridge() override;

private:
  using GripperStatus = gripper_ros2::msg::GripperStatus;
  using GripperMotorState = gripper_ros2::msg::GripperMotorState;
  using GripperConfig = gripper_ros2::msg::GripperConfig;
  using DiagArray = diagnostic_msgs::msg::DiagnosticArray;
  using Trigger = std_srvs::srv::Trigger;
  using SetStateSrv = gripper_ros2::srv::SetState;
  using SetParamSrv = gripper_ros2::srv::SetParam;
  using GetParamSrv = gripper_ros2::srv::GetParam;
  using SetFloat32Srv = gripper_ros2::srv::SetFloat32;
  using GraspAction = gripper_ros2::action::Grasp;
  using OpenAction = gripper_ros2::action::Open;
  using CalibAction = gripper_ros2::action::CalibrateForce;

  static constexpr uint8_t kConfigParamMaxId = 25;
  static constexpr size_t kConfigParamCount = 26;
  static constexpr uint8_t kOpenPosParamId = 5;

  static bool is_busy_state(uint8_t state);
  static int clamp_force_grams(float grams);

  void start_action_worker(std::function<void()> worker_fn);
  void reap_finished_action_workers_locked();
  void wait_for_action_workers();

  void open_serial();
  bool send(uint32_t can_id, const std::vector<uint8_t>& payload);

  GripperConfig get_live_config_snapshot();
  float get_live_config_field(uint8_t id);
  void publish_live_config_snapshot();
  bool refresh_live_config_from_firmware(double timeout_s = 0.5);

  void dispatch_rx(uint32_t id, const uint8_t* d, uint8_t len);
  void parse_status(const uint8_t* d);
  void parse_motor(const uint8_t* d);
  void parse_config_rx(const uint8_t* d);

  void on_clear_fault(Trigger::Request::SharedPtr req,
                      Trigger::Response::SharedPtr res);
  void on_set_state(SetStateSrv::Request::SharedPtr req,
                    SetStateSrv::Response::SharedPtr res);
  void on_set_param(SetParamSrv::Request::SharedPtr req,
                    SetParamSrv::Response::SharedPtr res);
  void on_get_param(GetParamSrv::Request::SharedPtr req,
                    GetParamSrv::Response::SharedPtr res);
  void on_set_force_srv(SetFloat32Srv::Request::SharedPtr req,
                        SetFloat32Srv::Response::SharedPtr res);
  void on_set_manual_pos(SetFloat32Srv::Request::SharedPtr req,
                         SetFloat32Srv::Response::SharedPtr res);
  void on_save_config(Trigger::Request::SharedPtr req,
                      Trigger::Response::SharedPtr res);
  void on_fetch_config(Trigger::Request::SharedPtr req,
                       Trigger::Response::SharedPtr res);

  void on_target_force(float grams);

  rclcpp_action::GoalResponse handle_grasp_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GraspAction::Goal> goal);
  void execute_grasp(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspAction>> gh);

  rclcpp_action::GoalResponse handle_open_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const OpenAction::Goal> goal);
  void execute_open(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<OpenAction>> gh);

  rclcpp_action::GoalResponse handle_calib_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const CalibAction::Goal> goal);
  void execute_calib(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibAction>> gh);

  void publish_diagnostics();

  bool verify_target_force_readback(float requested_grams,
                                    double timeout_s,
                                    float tolerance_grams,
                                    const std::string& timeout_message,
                                    const std::string& mismatch_message,
                                    std::string& error_message);

    std::optional<float> request_param_sync(uint8_t id, double timeout_s = 0.5, int max_attempts = 3);

  GripperStatus get_last_status();
  GripperMotorState get_last_motor();
  uint8_t get_last_state();

  static void set_config_field(GripperConfig& c, uint8_t id, float v);
  static float get_config_field(const GripperConfig& c, uint8_t id);

  // Transport
  SerialTransport serial_;

  // ROS interfaces
  rclcpp::Publisher<GripperStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<GripperMotorState>::SharedPtr motor_pub_;
  rclcpp::Publisher<GripperConfig>::SharedPtr config_pub_;
  rclcpp::Publisher<DiagArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub_;

  rclcpp::Service<Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::Service<SetStateSrv>::SharedPtr set_state_srv_;
  rclcpp::Service<SetParamSrv>::SharedPtr set_param_srv_;
  rclcpp::Service<GetParamSrv>::SharedPtr get_param_srv_;
  rclcpp::Service<SetFloat32Srv>::SharedPtr set_force_srv_;
  rclcpp::Service<SetFloat32Srv>::SharedPtr set_manual_srv_;
  rclcpp::Service<Trigger>::SharedPtr save_config_srv_;
  rclcpp::Service<Trigger>::SharedPtr fetch_config_srv_;

  rclcpp_action::Server<GraspAction>::SharedPtr grasp_as_;
  rclcpp_action::Server<OpenAction>::SharedPtr open_as_;
  rclcpp_action::Server<CalibAction>::SharedPtr calib_as_;

  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Shared telemetry
  std::mutex status_mutex_;
  GripperStatus last_status_{};
  GripperMotorState last_motor_{};

  // Live config cache (updated by RX callback)
  std::mutex config_mutex_;
  std::condition_variable config_cv_;
  std::array<uint64_t, kConfigParamCount> config_update_seq_{};
  GripperConfig live_config_{};

  // One waiter per param id for synchronous readback
  std::mutex promise_mutex_;
  std::map<uint8_t, std::promise<float>> pending_promises_;

  // Managed action workers
  std::mutex action_workers_mutex_;
  std::vector<std::future<void>> action_workers_;
  std::atomic<bool> shutting_down_{false};

  int64_t last_force_tx_ms_{0};
};
