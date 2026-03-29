# gripper_ros2 C++ bridge examples

These are ROS 2 C++ nodes demonstrating how to use the bridge APIs.

Source files are located in `src/examples/`.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/g1_ws
colcon build --packages-select gripper_ros2
source install/setup.bash
```

## Common argument

All examples support:

- `--ns` bridge base namespace (default: `/gripper/gripper_ros_bridge`)

Example:

```bash
ros2 run gripper_ros2 bridge_open_action_client -- --ns /gripper/gripper_ros_bridge
```

## Examples

- `bridge_service_examples_node`
  - modes:
    - `clear_idle`
    - `param_io`
    - `force_service`
    - `manual_step`
    - `config_ops`
- `bridge_force_topic_example_node`
- `bridge_open_action_client`
- `bridge_grasp_action_client`
- `bridge_calibrate_action_client`

## Commands

```bash
# 1) Clear fault + set IDLE
ros2 run gripper_ros2 bridge_service_examples_node -- --mode clear_idle

# 2) Set/get telemetry ms (param 18)
ros2 run gripper_ros2 bridge_service_examples_node -- --mode param_io --param-id 18 --value 20.0

# 3) Set target force via service
ros2 run gripper_ros2 bridge_service_examples_node -- --mode force_service --force 150.0

# 4) Publish force via topic for 2s at 20 Hz
ros2 run gripper_ros2 bridge_force_topic_example_node -- --force 200.0 --seconds 2.0 --hz 20.0

# 5) Open action
ros2 run gripper_ros2 bridge_open_action_client -- --wait true

# 6) Grasp action (PID)
ros2 run gripper_ros2 bridge_grasp_action_client -- --force 180 --tol 5 --settle 0.5 --timeout 8 --admit false

# 7) Grasp action (admittance)
ros2 run gripper_ros2 bridge_grasp_action_client -- --force 180 --tol 5 --settle 0.5 --timeout 8 --admit true

# 8) Calibrate action
ros2 run gripper_ros2 bridge_calibrate_action_client

# 9) Manual tune one step
ros2 run gripper_ros2 bridge_service_examples_node -- --mode manual_step --position -0.8

# 10) Fetch + save config
ros2 run gripper_ros2 bridge_service_examples_node -- --mode config_ops
```
