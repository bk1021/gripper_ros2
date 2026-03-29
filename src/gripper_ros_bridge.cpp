#include "gripper_ros2/gripper_ros_bridge.hpp"
#include "gripper_ros2/gripper_constants.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <utility>

using namespace gripper;
using namespace std::chrono_literals;

namespace {

std::vector<uint8_t> pack_float_be(float f) {
  uint32_t raw = 0;
  std::memcpy(&raw, &f, sizeof(float));
  return {
      static_cast<uint8_t>(raw >> 24),
      static_cast<uint8_t>(raw >> 16),
      static_cast<uint8_t>(raw >> 8),
      static_cast<uint8_t>(raw)};
}

float unpack_float_be(const uint8_t* d) {
  uint32_t raw = (static_cast<uint32_t>(d[0]) << 24) |
                 (static_cast<uint32_t>(d[1]) << 16) |
                 (static_cast<uint32_t>(d[2]) << 8) |
                 static_cast<uint32_t>(d[3]);
  float value = 0.0f;
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}

int16_t unpack_i16_be(const uint8_t* d) {
  return static_cast<int16_t>((d[0] << 8) | d[1]);
}

}  // namespace

GripperROSBridge::GripperROSBridge() : Node("gripper_ros_bridge") {
  declare_parameter("port", "/dev/ttyUSB0");
  declare_parameter("baudrate", 921600);
  declare_parameter("rx_timeout_ms", 500);
  declare_parameter("force_rate_limit_ms", 50);

  serial_.set_rx_callback([this](uint32_t id, const uint8_t* d, uint8_t len) {
    dispatch_rx(id, d, len);
  });

  status_pub_ = create_publisher<GripperStatus>("~/status", 10);
  motor_pub_ = create_publisher<GripperMotorState>("~/motor_state", 10);
  config_pub_ =
      create_publisher<GripperConfig>("~/config", rclcpp::QoS(1).transient_local());
  diag_pub_ = create_publisher<DiagArray>("~/diagnostics", 10);

  force_sub_ = create_subscription<std_msgs::msg::Float32>(
      "~/target_force", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        on_target_force(msg->data);
      });

  using std::placeholders::_1;
  using std::placeholders::_2;

  clear_fault_srv_ = create_service<Trigger>(
      "~/clear_fault", std::bind(&GripperROSBridge::on_clear_fault, this, _1, _2));

  set_state_srv_ = create_service<SetStateSrv>(
      "~/set_state", std::bind(&GripperROSBridge::on_set_state, this, _1, _2));

  set_param_srv_ = create_service<SetParamSrv>(
      "~/set_param", std::bind(&GripperROSBridge::on_set_param, this, _1, _2));

  get_param_srv_ = create_service<GetParamSrv>(
      "~/get_param", std::bind(&GripperROSBridge::on_get_param, this, _1, _2));

  set_force_srv_ = create_service<SetFloat32Srv>(
      "~/set_target_force",
      std::bind(&GripperROSBridge::on_set_force_srv, this, _1, _2));

  set_manual_srv_ = create_service<SetFloat32Srv>(
      "~/set_manual_position",
      std::bind(&GripperROSBridge::on_set_manual_pos, this, _1, _2));

  save_config_srv_ = create_service<Trigger>(
      "~/save_config", std::bind(&GripperROSBridge::on_save_config, this, _1, _2));

  fetch_config_srv_ = create_service<Trigger>(
      "~/fetch_config", std::bind(&GripperROSBridge::on_fetch_config, this, _1, _2));

  grasp_as_ = rclcpp_action::create_server<GraspAction>(
      this, "~/grasp",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const GraspAction::Goal> goal) {
        return handle_grasp_goal(uuid, goal);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspAction>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspAction>> gh) {
        start_action_worker([this, gh]() { execute_grasp(gh); });
      });

  open_as_ = rclcpp_action::create_server<OpenAction>(
      this, "~/open",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const OpenAction::Goal> goal) {
        return handle_open_goal(uuid, goal);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<OpenAction>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<OpenAction>> gh) {
        start_action_worker([this, gh]() { execute_open(gh); });
      });

  calib_as_ = rclcpp_action::create_server<CalibAction>(
      this, "~/calibrate",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const CalibAction::Goal> goal) {
        return handle_calib_goal(uuid, goal);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibAction>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibAction>> gh) {
        start_action_worker([this, gh]() { execute_calib(gh); });
      });

  diag_timer_ = create_wall_timer(1s, [this]() { publish_diagnostics(); });

  open_serial();

  const bool startup_cfg_ok = refresh_live_config_from_firmware();
  if (!startup_cfg_ok) {
    RCLCPP_WARN(
        get_logger(),
        "Startup config fetch incomplete. One or more parameter reads timed out.");
  }

  RCLCPP_INFO(get_logger(), "GripperROSBridge ready on %s",
              get_parameter("port").as_string().c_str());
}

GripperROSBridge::~GripperROSBridge() {
  shutting_down_.store(true);
  config_cv_.notify_all();
  wait_for_action_workers();
  serial_.close();
}

bool GripperROSBridge::is_busy_state(uint8_t state) {
  return state == static_cast<uint8_t>(State::ZEROING_MOTOR) ||
         state == static_cast<uint8_t>(State::CALIBRATING_FORCE) ||
         state == static_cast<uint8_t>(State::SAVING_FLASH);
}

int GripperROSBridge::clamp_force_grams(float grams) {
  return std::clamp(static_cast<int>(grams), 0, 65535);
}

void GripperROSBridge::start_action_worker(std::function<void()> worker_fn) {
  std::lock_guard<std::mutex> lock(action_workers_mutex_);
  reap_finished_action_workers_locked();
  action_workers_.emplace_back(
      std::async(std::launch::async, std::move(worker_fn)));
}

void GripperROSBridge::reap_finished_action_workers_locked() {
  auto it = action_workers_.begin();
  while (it != action_workers_.end()) {
    if (it->valid() && it->wait_for(0ms) == std::future_status::ready) {
      try {
        it->get();
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Action worker failed: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Action worker failed: unknown exception");
      }
      it = action_workers_.erase(it);
    } else {
      ++it;
    }
  }
}

void GripperROSBridge::wait_for_action_workers() {
  std::vector<std::future<void>> workers;
  {
    std::lock_guard<std::mutex> lock(action_workers_mutex_);
    workers = std::move(action_workers_);
  }

  for (auto& worker : workers) {
    if (!worker.valid()) {
      continue;
    }

    try {
      worker.get();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Action worker failed on shutdown: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Action worker failed on shutdown: unknown exception");
    }
  }
}

void GripperROSBridge::open_serial() {
  const auto port = get_parameter("port").as_string();
  const auto baud = get_parameter("baudrate").as_int();
  if (!serial_.open(port, baud)) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", port.c_str());
  }
}

bool GripperROSBridge::send(uint32_t can_id, const std::vector<uint8_t>& payload) {
  if (!serial_.send_frame(can_id, payload)) {
    RCLCPP_ERROR(get_logger(), "TX failed (serial not open or write error)");
    return false;
  }
  return true;
}

GripperROSBridge::GripperConfig GripperROSBridge::get_live_config_snapshot() {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return live_config_;
}

float GripperROSBridge::get_live_config_field(uint8_t id) {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return get_config_field(live_config_, id);
}

void GripperROSBridge::publish_live_config_snapshot() {
  auto cfg = get_live_config_snapshot();
  cfg.header.stamp = now();
  config_pub_->publish(cfg);
}

bool GripperROSBridge::refresh_live_config_from_firmware(double timeout_s) {
  bool all_ok = true;
  for (uint8_t id = 0; id <= kConfigParamMaxId; ++id) {
    std::this_thread::sleep_for(10ms);
    auto val = request_param_sync(id, timeout_s);
    if (!val) {
      RCLCPP_WARN(get_logger(), "Timeout fetching param %d (%s)", id, param_name(id));
      all_ok = false;
    }
  }

  publish_live_config_snapshot();
  return all_ok;
}

void GripperROSBridge::dispatch_rx(uint32_t id, const uint8_t* d, uint8_t) {
  switch (id) {
    case CAN_ID_STATUS:    parse_status(d);    break;
    case CAN_ID_MOTOR:     parse_motor(d);     break;
    case CAN_ID_CONFIG_RX: parse_config_rx(d); break;
    default: break;
  }
}

void GripperROSBridge::parse_status(const uint8_t* d) {
  GripperStatus msg;
  msg.header.stamp = now();
  msg.state = d[0];
  msg.fault = d[1];
  msg.pc3_grams = unpack_i16_be(d + 2) / 10.0f;
  msg.pc2_grams = unpack_i16_be(d + 4) / 10.0f;
  msg.avg_grams = (msg.pc3_grams + msg.pc2_grams) / 2.0f;
  msg.pid_output = unpack_i16_be(d + 6) / 10000.0f;
  status_pub_->publish(msg);

  std::lock_guard<std::mutex> lock(status_mutex_);
  last_status_ = msg;
}

void GripperROSBridge::parse_motor(const uint8_t* d) {
  GripperMotorState msg;
  msg.header.stamp = now();

  const int32_t pos_raw = (static_cast<int32_t>(d[0]) << 24) |
                          (static_cast<int32_t>(d[1]) << 16) |
                          (static_cast<int32_t>(d[2]) << 8) |
                          static_cast<int32_t>(d[3]);

  msg.position_rad = pos_raw / 10000.0f;
  msg.velocity_rad_s = unpack_i16_be(d + 4) / 1000.0f;
  msg.torque_nm = unpack_i16_be(d + 6) / 1000.0f;
  motor_pub_->publish(msg);

  std::lock_guard<std::mutex> lock(status_mutex_);
  last_motor_ = msg;
}

void GripperROSBridge::parse_config_rx(const uint8_t* d) {
  if (d[0] != CMD_GET_PARAM) {
    return;
  }

  const uint8_t param_id = d[1];
  const float value = unpack_float_be(d + 2);

  {
    std::lock_guard<std::mutex> lock(promise_mutex_);
    auto it = pending_promises_.find(param_id);
    if (it != pending_promises_.end()) {
      it->second.set_value(value);
      pending_promises_.erase(it);
    }
  }

  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    set_config_field(live_config_, param_id, value);
    if (param_id <= kConfigParamMaxId) {
      ++config_update_seq_[param_id];
    }
  }

  config_cv_.notify_all();
}

void GripperROSBridge::on_clear_fault(
    Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr res) {
  if (!send(CAN_ID_CMD, {CMD_CLEAR_FAULT})) {
    res->success = false;
    res->message = "TX failed";
    return;
  }

  res->success = true;
  res->message = "Sent clear_fault";
}

void GripperROSBridge::on_set_state(
    SetStateSrv::Request::SharedPtr req,
    SetStateSrv::Response::SharedPtr res) {
  if (is_busy_state(get_last_state())) {
    res->success = false;
    res->message = "Gripper busy. State transition rejected";
    return;
  }

  if (!send(CAN_ID_CMD, {CMD_SET_STATE, req->target_state})) {
    res->success = false;
    res->message = "TX failed";
    return;
  }

  res->success = true;
  res->message = "OK";
}

void GripperROSBridge::on_set_param(
    SetParamSrv::Request::SharedPtr req,
    SetParamSrv::Response::SharedPtr res) {
  if (req->param_id > kConfigParamMaxId) {
    res->success = false;
    res->message = "param_id out of range (0-22)";
    return;
  }

  // Firmware exposes these as runtime readback values.
  // 15/16 are read-only sensor-zero values and cannot be set.
  if (req->param_id == 15 || req->param_id == 16) {
    res->success = false;
    res->message = "param_id is read-only (15, 16)";
    return;
  }

  // target_force_grams (id 17) is writable, but not via set_param.
  // Use ~/set_target_force service or ~/target_force topic instead.
  if (req->param_id == 17) {
    res->success = false;
    res->message = "param_id 17 must be set via ~/set_target_force or ~/target_force";
    return;
  }

  const auto fb = pack_float_be(req->value);
  if (!send(CAN_ID_CMD,
            {CMD_SET_PARAM, req->param_id, fb[0], fb[1], fb[2], fb[3]})) {
    res->success = false;
    res->message = "TX failed";
    return;
  }

  auto confirmed = request_param_sync(req->param_id);
  if (!confirmed) {
    res->success = false;
    res->message = "Readback timeout. Param written to RAM but unconfirmed";
    return;
  }

  publish_live_config_snapshot();
  res->success = true;
  res->confirmed_value = *confirmed;
  res->message = "OK";
}

void GripperROSBridge::on_get_param(
    GetParamSrv::Request::SharedPtr req,
    GetParamSrv::Response::SharedPtr res) {
  if (req->param_id > kConfigParamMaxId) {
    res->success = false;
    res->message = "param_id out of range (0-22)";
    return;
  }

  auto val = request_param_sync(req->param_id);
  if (!val) {
    res->success = false;
    res->message = "Timeout. No response from firmware";
    return;
  }

  res->success = true;
  res->value = *val;
  res->message = "OK";
}

void GripperROSBridge::on_set_force_srv(
    SetFloat32Srv::Request::SharedPtr req,
    SetFloat32Srv::Response::SharedPtr res) {
  const int force = clamp_force_grams(req->value);
  if (!send(CAN_ID_CMD, {CMD_SET_FORCE, uint8_t(force >> 8), uint8_t(force)})) {
    res->success = false;
    res->message = "TX failed";
    return;
  }

  res->success = true;
  res->message = "OK";
}

void GripperROSBridge::on_set_manual_pos(
    SetFloat32Srv::Request::SharedPtr req,
    SetFloat32Srv::Response::SharedPtr res) {
  if (get_last_state() != static_cast<uint8_t>(State::MANUAL_TUNE)) {
    res->success = false;
    res->message = "Not in MANUAL_TUNE state";
    return;
  }

  const auto fb = pack_float_be(req->value);
  if (!send(CAN_ID_CMD, {CMD_MANUAL_POS, fb[0], fb[1], fb[2], fb[3]})) {
    res->success = false;
    res->message = "TX failed";
    return;
  }

  res->success = true;
  res->message = "OK";
}

void GripperROSBridge::on_save_config(
    Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr res) {
  if (is_busy_state(get_last_state())) {
    res->success = false;
    res->message = "Gripper busy. Cannot save now";
    return;
  }

  if (!send(CAN_ID_CMD, {CMD_SET_STATE, static_cast<uint8_t>(State::SAVING_FLASH)})) {
    res->success = false;
    res->message = "TX failed";
    return;
  }

  res->success = true;
  res->message = "Flash save triggered. Firmware freezes about 1 second then returns to IDLE.";
}

void GripperROSBridge::on_fetch_config(
    Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr res) {
  const bool all_ok = refresh_live_config_from_firmware(0.5);
  res->success = all_ok;
  res->message = all_ok ? "Config fetched and published to ~/config"
                        : "Partial fetch. Check warnings";
}

void GripperROSBridge::on_target_force(float grams) {
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();

  const auto limit_ms = get_parameter("force_rate_limit_ms").as_int();
  if ((now_ms - last_force_tx_ms_) < limit_ms) {
    return;
  }

  last_force_tx_ms_ = now_ms;
  const int force = clamp_force_grams(grams);
  (void)send(CAN_ID_CMD, {CMD_SET_FORCE, uint8_t(force >> 8), uint8_t(force)});
}

rclcpp_action::GoalResponse GripperROSBridge::handle_grasp_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const GraspAction::Goal> goal) {
  const auto state = get_last_state();
  if (state == static_cast<uint8_t>(State::FAULT) ||
      state == static_cast<uint8_t>(State::GRASPING) ||
      state == static_cast<uint8_t>(State::ADMITTANCE_CTRL) ||
      state == static_cast<uint8_t>(State::APPROACHING)) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->target_force_grams <= 0.0f || goal->timeout_sec <= 0.0f) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void GripperROSBridge::execute_grasp(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspAction>> gh) {
  const auto goal = gh->get_goal();

  const int force = clamp_force_grams(goal->target_force_grams);
  if (!send(CAN_ID_CMD, {CMD_SET_FORCE, uint8_t(force >> 8), uint8_t(force)})) {
    auto result = std::make_shared<GraspAction::Result>();
    result->success = false;
    result->message = "TX failed while setting target force";
    gh->abort(result);
    return;
  }

  if (!send(CAN_ID_CMD, {CMD_SET_STATE, static_cast<uint8_t>(State::APPROACHING)})) {
    auto result = std::make_shared<GraspAction::Result>();
    result->success = false;
    result->message = "TX failed while commanding APPROACHING";
    gh->abort(result);
    return;
  }

  auto feedback = std::make_shared<GraspAction::Feedback>();
  const auto deadline = now() + rclcpp::Duration::from_seconds(goal->timeout_sec);
  rclcpp::Rate rate(20);

  while (rclcpp::ok() && !shutting_down_.load()) {
    if (gh->is_canceling()) {
      (void)send(CAN_ID_CMD, {CMD_SET_STATE, static_cast<uint8_t>(State::OPENING)});
      auto result = std::make_shared<GraspAction::Result>();
      result->success = false;
      result->message = "Cancelled";
      gh->canceled(result);
      return;
    }

    const auto status = get_last_status();
    feedback->current_avg_force_grams = status.avg_grams;
    feedback->force_error_grams = goal->target_force_grams - status.avg_grams;
    feedback->current_state = status.state;
    gh->publish_feedback(feedback);

    if (status.state == static_cast<uint8_t>(State::GRASPING) &&
        std::abs(feedback->force_error_grams) < 10.0f) {
      auto result = std::make_shared<GraspAction::Result>();
      result->success = true;
      result->final_state = status.state;
      result->final_avg_force_grams = status.avg_grams;
      result->message = "Force settled";
      gh->succeed(result);
      return;
    }

    if (status.state == static_cast<uint8_t>(State::ABORTED) ||
        status.state == static_cast<uint8_t>(State::FAULT)) {
      auto result = std::make_shared<GraspAction::Result>();
      result->success = false;
      result->final_state = status.state;
      result->message = "Gripper aborted or faulted";
      gh->abort(result);
      return;
    }

    if (now() > deadline) {
      (void)send(CAN_ID_CMD, {CMD_SET_STATE, static_cast<uint8_t>(State::OPENING)});
      auto result = std::make_shared<GraspAction::Result>();
      result->success = false;
      result->message = "Timeout";
      gh->abort(result);
      return;
    }

    rate.sleep();
  }

  if (gh->is_active()) {
    auto result = std::make_shared<GraspAction::Result>();
    result->success = false;
    result->message = "Node shutting down";
    gh->abort(result);
  }
}

rclcpp_action::GoalResponse GripperROSBridge::handle_open_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const OpenAction::Goal>) {
  if (get_last_state() == static_cast<uint8_t>(State::FAULT)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void GripperROSBridge::execute_open(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<OpenAction>> gh) {
  const auto goal = gh->get_goal();
  if (!send(CAN_ID_CMD, {CMD_SET_STATE, static_cast<uint8_t>(State::OPENING)})) {
    auto result = std::make_shared<OpenAction::Result>();
    result->success = false;
    gh->abort(result);
    return;
  }

  if (!goal->wait_for_completion) {
    auto result = std::make_shared<OpenAction::Result>();
    result->success = true;
    gh->succeed(result);
    return;
  }

  auto feedback = std::make_shared<OpenAction::Feedback>();
  rclcpp::Rate rate(20);
  const auto deadline = now() + rclcpp::Duration::from_seconds(10.0);

  while (rclcpp::ok() && !shutting_down_.load()) {
    if (gh->is_canceling()) {
      auto result = std::make_shared<OpenAction::Result>();
      result->success = false;
      gh->canceled(result);
      return;
    }

    const auto motor = get_last_motor();
    const float open_pos = get_live_config_field(kOpenPosParamId);
    feedback->current_position_rad = motor.position_rad;
    feedback->distance_to_open_rad = open_pos - motor.position_rad;
    gh->publish_feedback(feedback);

    if (get_last_state() == static_cast<uint8_t>(State::IDLE)) {
      auto result = std::make_shared<OpenAction::Result>();
      result->success = true;
      result->final_position_rad = motor.position_rad;
      gh->succeed(result);
      return;
    }

    if (now() > deadline) {
      auto result = std::make_shared<OpenAction::Result>();
      result->success = false;
      gh->abort(result);
      return;
    }

    rate.sleep();
  }

  if (gh->is_active()) {
    auto result = std::make_shared<OpenAction::Result>();
    result->success = false;
    gh->abort(result);
  }
}

rclcpp_action::GoalResponse GripperROSBridge::handle_calib_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const CalibAction::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void GripperROSBridge::execute_calib(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<CalibAction>> gh) {
  if (!send(CAN_ID_CMD, {CMD_SET_STATE, static_cast<uint8_t>(State::CALIBRATING_FORCE)})) {
    auto result = std::make_shared<CalibAction::Result>();
    result->success = false;
    result->message = "TX failed while commanding CALIBRATING_FORCE";
    gh->abort(result);
    return;
  }

  const auto confirm_deadline = now() + rclcpp::Duration(0, 200000000);
  while (rclcpp::ok() && !shutting_down_.load() &&
         get_last_state() != static_cast<uint8_t>(State::CALIBRATING_FORCE)) {
    if (now() > confirm_deadline) {
      RCLCPP_ERROR(get_logger(),
                   "Calibration state not confirmed within 200ms. Firmware rejected?");
      auto result = std::make_shared<CalibAction::Result>();
      result->success = false;
      result->message = "State transition not confirmed by firmware";
      gh->abort(result);
      return;
    }
    std::this_thread::sleep_for(20ms);
  }

  rclcpp::Rate rate(10);
  const auto deadline = now() + rclcpp::Duration::from_seconds(30.0);

  while (rclcpp::ok() && !shutting_down_.load()) {
    if (gh->is_canceling()) {
      (void)send(CAN_ID_CMD, {CMD_SET_STATE, static_cast<uint8_t>(State::OPENING)});
      auto result = std::make_shared<CalibAction::Result>();
      result->success = false;
      result->message = "Cancelled";
      gh->canceled(result);
      return;
    }

    const auto status = get_last_status();

    if (status.state == static_cast<uint8_t>(State::SAVING_FLASH) ||
        status.state == static_cast<uint8_t>(State::IDLE)) {
      auto thresh = request_param_sync(14);
      auto result = std::make_shared<CalibAction::Result>();
      result->success = true;
      result->contact_threshold_result = thresh.value_or(0.0f);
      result->message = "Calibration complete";
      gh->succeed(result);
      return;
    }

    if (status.state == static_cast<uint8_t>(State::FAULT)) {
      auto result = std::make_shared<CalibAction::Result>();
      result->success = false;
      result->message = "Faulted during calibration";
      gh->abort(result);
      return;
    }

    if (now() > deadline) {
      auto result = std::make_shared<CalibAction::Result>();
      result->success = false;
      result->message = "Timeout (30s). Calibration did not complete";
      gh->abort(result);
      return;
    }

    rate.sleep();
  }

  if (gh->is_active()) {
    auto result = std::make_shared<CalibAction::Result>();
    result->success = false;
    result->message = "Node shutting down";
    gh->abort(result);
  }
}

void GripperROSBridge::publish_diagnostics() {
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status_msg;
  status_msg.hardware_id = get_parameter("port").as_string();
  status_msg.name = "gripper/bridge";

  const auto status = get_last_status();
  const auto age = serial_.rx_age_ms();

  if (!serial_.is_open()) {
    status_msg.level = 2;
    status_msg.message = "Serial disconnected";
  } else if (age > static_cast<uint64_t>(get_parameter("rx_timeout_ms").as_int())) {
    status_msg.level = 2;
    status_msg.message = "RX timeout. No data from STM32";
  } else if (status.fault != 0) {
    status_msg.level = 1;
    status_msg.message = "Gripper fault active";
  } else {
    status_msg.level = 0;
    status_msg.message = "Nominal";
  }

  auto kv = [](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  };

  status_msg.values.push_back(kv("state", std::to_string(status.state)));
  status_msg.values.push_back(kv("fault", std::to_string(status.fault)));
  status_msg.values.push_back(kv("rx_age_ms", std::to_string(age)));
  status_msg.values.push_back(kv("pc3_g", std::to_string(status.pc3_grams)));
  status_msg.values.push_back(kv("pc2_g", std::to_string(status.pc2_grams)));

  arr.status.push_back(status_msg);
  diag_pub_->publish(arr);
}

std::optional<float> GripperROSBridge::request_param_sync(uint8_t id,
                                                          double timeout_s) {
  if (shutting_down_.load()) {
    return std::nullopt;
  }

  if (id > kConfigParamMaxId) {
    return std::nullopt;
  }

  std::promise<float> prom;
  auto future = prom.get_future();
  bool inserted = false;

  {
    std::lock_guard<std::mutex> lock(promise_mutex_);
    auto [it, ok] = pending_promises_.try_emplace(id, std::move(prom));
    inserted = ok;
    if (!ok) {
      RCLCPP_WARN(get_logger(),
                  "request_param_sync: param %d (%s) already pending; waiting on existing request",
                  id, param_name(id));
      (void)it;
    }
  }

  if (inserted) {
    if (!send(CAN_ID_CMD, {CMD_GET_PARAM, id})) {
      std::lock_guard<std::mutex> lock(promise_mutex_);
      pending_promises_.erase(id);
      return std::nullopt;
    }

    if (shutting_down_.load()) {
      std::lock_guard<std::mutex> lock(promise_mutex_);
      pending_promises_.erase(id);
      return std::nullopt;
    }

    if (future.wait_for(std::chrono::duration<double>(timeout_s)) ==
        std::future_status::ready) {
      return future.get();
    }

    std::lock_guard<std::mutex> lock(promise_mutex_);
    pending_promises_.erase(id);
    return std::nullopt;
  }

  std::unique_lock<std::mutex> lock(config_mutex_);
  const uint64_t seq = config_update_seq_[id];
  const bool updated = config_cv_.wait_for(
      lock, std::chrono::duration<double>(timeout_s), [this, id, seq]() {
        return shutting_down_.load() || config_update_seq_[id] > seq;
      });

  if (!updated || shutting_down_.load()) {
    return std::nullopt;
  }

  return get_config_field(live_config_, id);
}

GripperROSBridge::GripperStatus GripperROSBridge::get_last_status() {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return last_status_;
}

GripperROSBridge::GripperMotorState GripperROSBridge::get_last_motor() {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return last_motor_;
}

uint8_t GripperROSBridge::get_last_state() {
  return get_last_status().state;
}

void GripperROSBridge::set_config_field(GripperConfig& c, uint8_t id, float v) {
  switch (id) {
    case 0:  c.kp = v;                 break;
    case 1:  c.ki = v;                 break;
    case 2:  c.kd = v;                 break;
    case 3:  c.max_integral = v;       break;
    case 4:  c.max_torque = v;         break;
    case 5:  c.open_pos = v;           break;
    case 6:  c.close_pos = v;          break;
    case 7:  c.approach_limit = v;     break;
    case 8:  c.open_kp = v;            break;
    case 9:  c.open_kd = v;            break;
    case 10: c.approach_kd = v;        break;
    case 11: c.approach_vel = v;       break;
    case 12: c.deadband = v;           break;
    case 13: c.lpf_alpha = v;          break;
    case 14: c.contact_threshold = v;  break;
    case 15: c.pc3_zero_adc = v;       break;
    case 16: c.pc2_zero_adc = v;       break;
    case 17: c.target_force_grams = v; break;
    case 18: c.telemetry_ms = v;       break;
    case 19: c.virtual_mass = v;       break;
    case 20: c.virtual_damping = v;    break;
    case 21: c.admit_max_vel = v;      break;
    case 22: c.admit_kd = v;           break;
    default: break;
  }
}

float GripperROSBridge::get_config_field(const GripperConfig& c, uint8_t id) {
  switch (id) {
    case 0:  return c.kp;
    case 1:  return c.ki;
    case 2:  return c.kd;
    case 3:  return c.max_integral;
    case 4:  return c.max_torque;
    case 5:  return c.open_pos;
    case 6:  return c.close_pos;
    case 7:  return c.approach_limit;
    case 8:  return c.open_kp;
    case 9:  return c.open_kd;
    case 10: return c.approach_kd;
    case 11: return c.approach_vel;
    case 12: return c.deadband;
    case 13: return c.lpf_alpha;
    case 14: return c.contact_threshold;
    case 15: return c.pc3_zero_adc;
    case 16: return c.pc2_zero_adc;
    case 17: return c.target_force_grams;
    case 18: return c.telemetry_ms;
    case 19: return c.virtual_mass;
    case 20: return c.virtual_damping;
    case 21: return c.admit_max_vel;
    case 22: return c.admit_kd;
    default: return 0.0f;
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<GripperROSBridge>();
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
