# Hybrid Force Motion Controller Plan

## 1. High-Level Objectives
- Deliver a ROS 2 velocity-space hybrid force/motion controller (`hybrid_force_motion_controller`) that commands 3-DOF Cartesian velocity (x, y, z) while holding a 5 N normal force and executing a 5 cm tangential slide on a movable 9 cm hemisphere.
- Reuse the existing `ur_admittance_controller` wrench pipeline (`/netft/proc_probe`) so gravity/bias compensation stays centralized; integrate cleanly with UR bringup via `forward_velocity_controller`.
- Keep the workflow identical between hardware and simulation after the operator captures a start pose; only the "get into contact" step differs (teleport utilities vs. manual jogging).

## 2. System Architecture (Mid-Level View)
1. **Package layout**
   - `include/hybrid_force_motion_controller/` — main node + helper classes (parameters, state machine, Jacobian utilities).
   - `src/hybrid_force_motion_node.cpp` — executes the control loop, TF logic, operator services.
   - `config/hybrid_force_motion_config.yaml` — gains (normal PI with soft-start, tangential P), limits, thresholds, frame names.
   - `launch/hybrid_force_motion_sim.launch.py` / `hybrid_force_motion_hw.launch.py` — orchestrate sim/hardware stacks, including convenience teleport nodes when in sim.
   - `reference/friction_normal_estimator.md` — keeps the friction-aware normal math (migrated from `Untitled.md`).
2. **External dependencies** — `rclcpp`, `rclcpp_components`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`, `Eigen3`, `kdl_parser`, `urdfdom`, and direct include access to `ur_admittance_controller` headers for shared constants.
3. **Runtime nodes**
   - `wrench_node` (from `ur_admittance_controller`) publishes `/netft/proc_probe`.
   - `hybrid_force_motion_node` subscribes to `/netft/proc_probe`, `/joint_states`, TF, and optional `/hybrid_force_motion_controller/direction`; publishes `/forward_velocity_controller/commands`, `tf` contact frame, and a diagnostic topic `/hybrid_force_motion_controller/state`.
   - Simulation-only helpers: `dome_teleporter` (Gazebo entity state client) and `ee_teleporter` (IK solver that snaps the arm pose).

## 3. Implementation Phases & Tests (Build Pipeline)
1. **Phase 1 — Environment & Teleport Bring-Up**
   - Deliverables:
     - Gazebo world file `worlds/contact_dome.sdf` bundled with `teleport_bringup.launch.py`, which includes `ur_simulation_gz/ur_sim_control.launch.py` so URDF + dome spawn together.
     - `scripts/dome_teleporter.py` CLI/node that adjusts the dome pose at runtime through `/gazebo/set_entity_state`, accepting raw CLI params or a waypoint YAML (A, B, C… positions).
     - `scripts/ee_teleporter.py` service that solves IK (using the URDF / KDL chain shared with `ur_admittance_controller`) and calls `/set_model_configuration` so the robot snaps into contact—no controller required yet.
   - Test 1 checklist:
     1. `ros2 launch ur_simulation_gz ur_sim_control.launch.py ...` brings up the URDF+world without errors; dome is visible in RViz/Gazebo.
     2. Running the `dome_teleporter` command updates the dome pose, confirmed via RViz TF or Gazebo GUI.
     3. Calling `/hybrid_force_motion_controller/teleport_tool` places the tool exactly at the requested base pose (verified by TF and joint state echo).
     4. No hybrid controller running yet; success = manual/teleport contact achieved repeatedly.
2. **Phase 2 — Hybrid Force/Motion Control**
   - Deliverables:
     - Full controller node with PI soft-start normal regulation, tangential displacement tracking, friction-aware normal estimator, contact frame TF publication, and the state-machine services.
     - Contact-loss detector and FAULT handling tied into the teleporter workflow (i.e., after teleporting you must call `set_start_pose` before `start_motion`).
   - Test 2 checklist:
     1. With Phase 1 environment running, call `set_start_pose` followed by `start_motion`; verify RUNNING begins only after engage force threshold is exceeded.
     2. Observe the normal soft-start ramp (force ramps smoothly to 5 N), TF contact frame aligns with the dome, and tangential displacement halts at 0.05 m.
     3. Drop the force below threshold to confirm the contact-loss detector trips FAULT and zeroes commands; `set_start_pose` recovers.
     4. Exercise pause/resume/stop services and ensure diagnostics reflect the correct state transitions.
3. **Future phases (scoped but not yet scheduled)** — null-space orientation optimizer, automated waypoint sequencer, additional hardware soak tests. These inherit from the first two phases but are not required before initial delivery.

This phased approach keeps the build linear: Phase 1 must pass before Phase 2 starts, mirroring the methodology used in `ur_admittance_controller` (URDF bring-up → wrench pipeline → controller).

## 4. Detailed Design

### 4.1 Force & Motion Control Loop
- **Surface-normal estimation (friction-aware)** — implement Algorithm 1 from `reference/friction_normal_estimator.md`. Project the measured wrench onto the direction of motion, estimate Coulomb friction `\bar{\mu}`, subtract the tangential component, and normalize to obtain `\hat{n}_\text{surf}`. Use this for all projections.
- **Velocity command decomposition**
  - Tangential: `v_t = clamp(k_t * s_rem, ±v_t_max)` where `s_rem` is remaining tangential distance along the projected hint vector `\hat{t}`.
  - Normal: PI regulator `v_n = clamp(k_p e_F + k_i \int e_F dt, ±v_n_max)` to remove steady-state error from friction/bias; apply a slew-rate limit on `v_n` plus a configurable soft-start (`normal.softstart_time_s`) that ramps the internal force setpoint from 0→5 N over 0.5–1 s after RUNNING begins.
  - Compose the Cartesian twist `[v_t \hat{t} + v_n \hat{n}_\text{surf}, 0,0,0]`, run through a damped least-squares Jacobian pseudo-inverse, and clip by per-joint velocity limits.
- **Friction-aware enhancements** — expose parameters to toggle the estimator on/off and to tune the moving average on `\mu`. Default to enabled to match the dome-tracking literature cited in the reference note.

### 4.2 State Machine & Operator Interfaces
- States: `WAIT_FOR_START_POSE → READY → RUNNING ↔ PAUSED → COMPLETED`. Add `ABORTED` (operator stop) and `FAULT` (internal error). Only `set_start_pose` transitions out of `FAULT` or `ABORTED`.
- Services (all `std_srvs/Trigger`):
  - `/hybrid_force_motion_controller/set_start_pose` — capture TF pose, wrench snapshot, zero integrators, transition to `READY` (replaces `init_robot`).
  - `/start_motion`, `/pause_motion`, `/resume_motion`, `/stop_motion` — drive state transitions. `stop_motion` enters `ABORTED`.
  - `/teleport_tool` (sim only, custom srv) — accept a base-frame pose, solve IK, and set joint states instantly for point-to-point contact presets.
- Diagnostic topic `/hybrid_force_motion_controller/state` publishes current state, tangential progress, normal force, and error flags for logging/monitoring.

### 4.3 Safety, Faults, and Contact Management
- **Contact-loss detector** — if `(F · \hat{n}_\text{surf}) < disengage_threshold` for `N` consecutive control cycles, immediately zero commands and enter `FAULT` with reason `CONTACT_LOST`.
- **Fault triggers** — enter `FAULT` when any occur: NaNs/Infs in inputs, TF lookup failure persisting more than `M` cycles, Jacobian conditioning worse than `σ_min < σ_th` for `M` cycles, velocity/force limit violations, or the contact-loss detector firing. Require `set_start_pose` to exit.
- **PI anti-windup** — clamp the integral state when `v_n` saturates or when contact is lost.
- **TF publication** — broadcast `contact_frame` (origin at tool, axes `[\hat{t}, \hat{n}_\text{surf}, \hat{b}]`) so RViz can show the live normal/tangent and confirm teleport accuracy.

### 4.4 Simulation Convenience Workflow
- **Dome teleporter** — node/CLI that wraps Gazebo’s `/set_entity_state` to place the hemisphere relative to `base_link` (x/y/z plus optional RPY). Accepts YAML/JSON waypoint files so you can iterate through A→B→C quickly.
- **End-effector teleporter** — uses the same KDL chain to compute joint positions for a requested base pose, publishes to Gazebo’s joint state interface (or a dedicated service) to “snap” the robot. No trajectories, so it stays a simulation-only helper.
- After each teleport, call `set_start_pose`, then proceed with the hybrid controller; hardware users simply jog before calling the same service.

## 5. Validation & Delivery Checklist
1. **Unit tests** — gtests for:
   - Friction-aware normal estimator (compare against analytic cases from `reference/friction_normal_estimator.md`).
   - State-machine transitions given synthetic service calls and contact-loss events.
   - Jacobian DLS solver on representative UR joint states.
2. **Integration tests** — launch test in Gazebo headless mode with scripted dome/teleport moves, ensuring contact frame TFs, services, and PI loop behave as expected.
3. **Manual verification** — document RViz overlays (TF contact frame), topic commands, and service sequences in README.
4. **Documentation** — keep README focused on operator workflow (setup, services, sim vs. hardware). Reference this `plan.md` for design details and `reference/friction_normal_estimator.md` for estimator theory.
5. **Future enhancements (tracked but out of scope for initial delivery)** — null-space orientation optimization (align tool axis with estimated normal), automated waypoint sequencer, deeper comparison to literature.

This structure keeps the plan hierarchical (objectives → architecture → detailed design → validation) while avoiding duplication with README, which remains an operator-facing guide.
