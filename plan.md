# Hybrid Force Motion Controller Plan

## 1. High-Level Objectives
- Deliver a ROS 2 velocity-space hybrid force/motion controller (`hybrid_force_motion_controller`) that commands 3-DOF Cartesian velocity (x, y, z) while holding a 5 N normal force and executing a 5 cm tangential slide on a movable 9 cm hemisphere.
- Reuse the existing `ur_admittance_controller` wrench pipeline (`/netft/proc_probe`) so gravity/bias compensation stays centralized; integrate cleanly with UR bringup via `forward_velocity_controller`.
- Keep the workflow identical between hardware and simulation after the operator captures a start pose; the only difference is how you drive into the hover pose (joint teleport command in sim vs. manual jog on hardware).

## 2. System Architecture (Mid-Level View)
1. **Package layout**
   - `include/hybrid_force_motion_controller/` — main node + helper classes (parameters, state machine, Jacobian utilities).
   - `src/hybrid_force_motion_node.cpp` — executes the force/motion FSM, publishes the Cartesian twist command, and handles services/state.
   - `src/cartesian_velocity_controller_node.cpp` — subscribes to the twist command, enforces Cartesian/joint limits, performs IK, and drives `/forward_velocity_controller/commands`.
   - `config/hybrid_force_motion_config.yaml` — gains (normal PI with soft-start, tangential P), limits, thresholds, frame names.
   - `launch/hybrid_force_motion_sim.launch.py` — launches UR Gazebo, the dome world, the bridge service, and the hybrid controller. Hardware runs the same node after your standard UR bringup.
   - `reference/friction_normal_estimator.md` — keeps the friction-aware normal math (migrated from `Untitled.md`).
2. **External dependencies** — `rclcpp`, `rclcpp_components`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`, `Eigen3`, `kdl_parser`, `urdfdom`, and direct include access to `ur_admittance_controller` headers for shared constants.
3. **Runtime nodes**
   - `wrench_node` (from `ur_admittance_controller`) publishes `/netft/proc_sensor`, `/netft/proc_probe`, and the new `/netft/proc_base` topic used by the hybrid controller.
   - `hybrid_force_motion_node` subscribes to `/netft/proc_base`, `/joint_states`, TF, and optional `/hybrid_force_motion_controller/direction`; publishes the diagnostic state topic plus `/hybrid_force_motion_controller/twist_cmd`.
   - `cartesian_velocity_controller_node` consumes `/hybrid_force_motion_controller/twist_cmd`, applies the configured limits, runs the KDL IK, and publishes `/forward_velocity_controller/commands`.
   - Simulation helpers: CLI commands only (`gz service /world/<world>/set_pose` for the dome and `ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory ...` for joint presets). No dedicated teleporter nodes remain.

## 3. End-to-End Test Checklist
### Simulation
- Launch UR sim + dome + hybrid stack  
  ```bash
  ros2 launch hybrid_force_motion_controller hybrid_force_motion_sim.launch.py ur_type:=ur5e
  ```
- Publish the joint teleport command to drop the arm into contact:
  ```bash
  ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'], points: [{positions: [0.0, -1.3, 1.7, -1.9, -1.57, 0.0], time_from_start: {sec: 5}}]}"
  ```
- Switch controllers so `forward_velocity_controller` owns the joints:
  ```bash
  ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller
  ```
- Capture the start pose and run:
  ```bash
  ros2 service call /hybrid_force_motion_controller/set_start_pose std_srvs/srv/Trigger {}
  ros2 service call /hybrid_force_motion_controller/start_motion std_srvs/srv/Trigger {}
  ```
- Monitor `/hybrid_force_motion_controller/state`, `/netft/proc_base`, and TF `contact_frame` to confirm the −Z seek, 1 s dwell, and 5 cm tangential traverse complete. Exercise `pause_motion`, `resume_motion`, and `stop_motion` to verify tangential distance handling and FAULT recovery.

### Hardware
- Bring up UR driver and sensor TF tree (`ur_hardware_bringup.launch.py`) and start the NetFT driver + `ur_admittance_controller` wrench node.
- Verify `/netft/proc_base`, `/joint_states`, and `/forward_velocity_controller/commands` topics exist; switch controllers only when `/forward_velocity_controller/commands` has zero publishers.
- Launch the hybrid stack:
  ```bash
  ros2 launch hybrid_force_motion_controller hybrid_force_motion_hardware.launch.py
  ```
- Jog to the desired hover pose, then:
  ```bash
  ros2 service call /hybrid_force_motion_controller/set_start_pose std_srvs/srv/Trigger {}
  ros2 service call /hybrid_force_motion_controller/start_motion std_srvs/srv/Trigger {}
  ```
- Watch `/hybrid_force_motion_controller/state`, `/netft/proc_base`, and `/hybrid_force_motion_controller/twist_cmd`. Keep an e-stop ready; kill the launch or run `/hybrid_force_motion_controller/stop_motion` if the path deviates. The Cartesian velocity controller zeros commands automatically if the twist stream stops for `twist_timeout_s` seconds.

## 4. Detailed Design

### 4.1 Force & Motion Control Loop
- **Frame conventions** — normalize everything in the `base_link` frame: `/netft/proc_base` supplies the compensated wrench expressed in base, `search_direction`/`contact_normal`/`tangential_dir` live in base, and `/hybrid_force_motion_controller/twist_cmd` is a base-frame twist consumed by the Cartesian velocity node.
- **Pre-contact normal locking** — while the FSM is in SEEK or DWELL, clamp `\hat{n}` to the configured `search_direction` (default base −Z) so the press direction can’t be hijacked by residual wrench bias; only tangential motion is withheld until the 5 N band and dwell timer complete.
- **Surface-normal estimation (friction-aware)** — implement Algorithm 1 from `reference/friction_normal_estimator.md`. Project the measured wrench onto the direction of motion, estimate Coulomb friction `\bar{\mu}`, subtract the tangential component, and normalize to obtain `\hat{n}_\text{surf}`. Use this for all projections once the slide is active.
- **Velocity command decomposition**
  - Tangential: `v_t = clamp(k_t * s_rem, ±v_t_max)` where `s_rem` is remaining tangential distance along the projected hint vector `\hat{t}`.
  - Normal: PI regulator `v_n = clamp(k_p e_F + k_i \int e_F dt, ±v_n_max)` to remove steady-state error from friction/bias; apply a slew-rate limit on `v_n` plus a configurable soft-start (`normal.softstart_time_s`) that ramps the internal force setpoint from 0→5 N over 0.5–1 s after RUNNING begins.
  - Compose the Cartesian twist `[v_t \hat{t} + v_n \hat{n}_\text{surf}, 0,0,0]` and publish it on `/hybrid_force_motion_controller/twist_cmd`; the dedicated velocity node handles IK and the only global twist clamp.
- **Friction-aware enhancements** — expose parameters to toggle the estimator on/off, to tune the moving average on `\mu`, and to require both a minimum contact load (`contact_force_min_threshold_N`) and a minimum tangential sliding speed before updating the frame. Default to enabled to match the dome-tracking literature cited in the reference note.

### 4.2 State Machine & Operator Interfaces
- States: `WAIT_FOR_START_POSE → READY → RUNNING ↔ PAUSED → COMPLETED`. Add `ABORTED` (operator stop) and `FAULT` (internal error). Only `set_start_pose` transitions out of `FAULT` or `ABORTED`.
- Services (all `std_srvs/Trigger`):
  - `/hybrid_force_motion_controller/set_start_pose` — capture TF pose, wrench snapshot, zero integrators, transition to `READY` (replaces `init_robot`).
  - `/start_motion`, `/pause_motion`, `/resume_motion`, `/stop_motion` — drive state transitions. `stop_motion` enters `ABORTED`.
  - `/scaled_joint_trajectory_controller/joint_trajectory` (sim) — command topic used to drop the UR arm joints into a preset pose before the controller takes over.
- Diagnostic topic `/hybrid_force_motion_controller/state` publishes current state, tangential progress (measured from the actual FK pose projected along the instantaneous tangential axis), normal force, and error flags for logging/monitoring. The Cartesian velocity command is exposed on `/hybrid_force_motion_controller/twist_cmd`, downstream of which the new node produces `/forward_velocity_controller/commands`.

### 4.3 Safety, Faults, and Contact Management
- **Contact-loss detector** — if `(F · \hat{n}_\text{surf}) < disengage_threshold` for `N` consecutive control cycles, immediately zero commands and enter `FAULT` with reason `CONTACT_LOST`.
- **Fault triggers** — enter `FAULT` when any occur: NaNs/Infs in inputs, TF lookup failure persisting more than `M` cycles, Jacobian conditioning worse than `σ_min < σ_th` for `M` cycles, velocity/force limit violations, or the contact-loss detector firing. Require `set_start_pose` to exit.
- **PI anti-windup** — clamp the integral state when `v_n` saturates or when contact is lost.
- **TF publication** — broadcast `contact_frame` (origin at tool, axes `[\hat{t}, \hat{n}_\text{surf}, \hat{b}]`) so RViz can show the live normal/tangent and confirm teleport accuracy.

### 4.4 Simulation Convenience Workflow
- **Dome repositioning** — use Gazebo Sim’s native CLI to set the model pose without launching any extra nodes:
  ```
  gz service -s /world/contact_dome/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: "contact_dome", position: { x: X, y: Y, z: Z }, orientation: { x: 0, y: 0, z: 0, w: 1 }'
  ```
  Replace the pose block as needed; everything is expressed in the `world` frame and follows the `gz.msgs.Pose` layout described in Gazebo’s CLI examples.
- **Arm repositioning** — publish a one-shot `JointTrajectory` message to `/scaled_joint_trajectory_controller/joint_trajectory` with the desired joint list/positions. Edit the `positions` array inline for each pose you need.
- After moving the dome and snapping the joints, call `set_start_pose`, then proceed with the hybrid controller; hardware users simply jog before calling the same service.

## 5. Validation & Delivery Checklist
1. **Unit tests** — gtests for:
   - Friction-aware normal estimator (compare against analytic cases from `reference/friction_normal_estimator.md`).
   - State-machine transitions given synthetic service calls and contact-loss events.
   - Jacobian DLS solver on representative UR joint states.
2. **Integration tests** — launch test in Gazebo headless mode with scripted dome/teleport moves, ensuring contact frame TFs, services, and PI loop behave as expected.
3. **Manual verification** — document RViz overlays (TF contact frame), topic commands, and service sequences in README.
4. **Documentation** — keep README focused on operator workflow (setup, services, sim vs. hardware). Reference this `plan.md` for design details and `reference/friction_normal_estimator.md` for estimator theory.
5. **Future enhancements (tracked but out of scope for initial delivery)** — null-space orientation optimization (align tool axis with estimated normal), automated pose sequencer, deeper comparison to literature.

This structure keeps the plan hierarchical (objectives → architecture → detailed design → validation) while avoiding duplication with README, which remains an operator-facing guide.
