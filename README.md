# hybrid_force_motion_controller

Hybrid force-motion velocity controller that reuses the `ur_admittance_controller` wrench pipeline, holds a **5 N ± 0.2 N** normal load, automatically dwells for ~1 s once the load stabilizes, and then slides **exactly 5 cm per run** in the tangent plane of an otherwise unknown curved surface while commanding `/forward_velocity_controller/commands`.

> **Status**: design frozen (see `plan.md` for architecture, `reference/friction_normal_estimator.md` for the surface-normal math). This README focuses on operator workflow.

## Setup
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ajaygunalan/ur_simulation_gz.git
git clone https://github.com/ajaygunalan/ur_admittance_controller.git
git clone <this repo> hybrid_force_motion_controller
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select hybrid_force_motion_controller && source install/setup.bash
```

For hardware, also build the NetFT stack as in `ur_admittance_controller/README.md`:
```bash
colcon build --packages-select netft_utils netft_interfaces
```

## Usage

### Simulation

1. Launch the sim stack  
   `hybrid_force_motion_sim.launch.py` starts UR Gazebo, the dome world, the bridge, the wrench pipeline (`wrench_node`), the hybrid node, and the Cartesian velocity controller.  
   ```bash
   ros2 launch hybrid_force_motion_controller hybrid_force_motion_sim.launch.py ur_type:=ur5e
   ```
2. Initialize the equilibrium / hover pose (new terminal)  
   This runs the same `init_robot` helper used on hardware, setting a reasonable equilibrium pose for the wrench pipeline before you move the arm.  
   ```bash
   ros2 run ur_admittance_controller init_robot
   ```
3. Move the arm into the hover pose you want  
   ```bash
   ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'], points: [{positions: [0.0, -1.3, 1.7, -1.9, -1.57, 0.0], time_from_start: {sec: 5}}]}"
   ```
4. Give the velocity controller ownership of the joints  
   ```bash
   ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller
   ```
5. Capture the start pose and run the hybrid motion  
   ```bash
   ros2 service call /hybrid_force_motion_controller/set_start_pose std_srvs/srv/Trigger {}
   ros2 service call /hybrid_force_motion_controller/start_motion std_srvs/srv/Trigger {}
   ```
Monitor `/hybrid_force_motion_controller/state`, `/netft/proc_base` (or `/netft/proc_probe` for probe-frame forces), and TF `contact_frame` while the sequence runs. Use `pause_motion`, `resume_motion`, or `stop_motion` as needed.

### Hardware Bringup

Prerequisites (same as `ur_admittance_controller`):
- [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [netft_utils](https://github.com/ajaygunalan/netft_utils)
- Real-time or low-latency kernel

Network (example):

| Device       | IP Address       |
|-------------|------------------|
| F/T Sensor  | 169.254.120.10   |
| UR5e        | 169.254.120.1    |
| Robo Laptop | 169.254.120.200  |

Subnet: `255.255.255.0` (verify with `ping 169.254.120.1`).  
Assumes you have already run UR calibration and wrench calibration as in `ur_admittance_controller/README.md`.

#### Daily Operation (Hardware)

Terminal 1 – UR driver + TF tree:
```bash
ros2 launch ur_admittance_controller ur_hardware_bringup.launch.py \
  robot_ip:=169.254.120.1 \
  kinematics_params_file:=$HOME/ur5e_calibration.yaml
```
On the teach pendant: load and play “External Control”.

Terminal 2 – F/T sensor driver:
```bash
ros2 run netft_utils netft_node --address 169.254.120.10 --frame_id netft_link1
```

Terminal 3 – Equilibrium / hover pose (optional per session):
```bash
ros2 run ur_admittance_controller init_robot
```
Then jog the arm into the hover/approach pose you want.

Terminal 4 – Wrench pipeline (gravity/bias compensation to `/netft/proc_base`):
```bash
ros2 run ur_admittance_controller wrench_node
```

Terminal 5 – Give `forward_velocity_controller` ownership of the joints:
```bash
ros2 topic info -v /forward_velocity_controller/commands  # MUST show 0 publishers
ros2 control switch_controllers \
  --deactivate scaled_joint_trajectory_controller \
  --activate forward_velocity_controller
```

Terminal 6 – Hybrid controller stack:
```bash
ros2 launch hybrid_force_motion_controller hybrid_force_motion_hardware.launch.py
```

Any terminal – Run the hybrid motion:
```bash
ros2 service call /hybrid_force_motion_controller/set_start_pose std_srvs/srv/Trigger {}
```
Then start the motion:
```bash
ros2 service call /hybrid_force_motion_controller/start_motion std_srvs/srv/Trigger {}
```
Monitor `/hybrid_force_motion_controller/state`, `/netft/proc_base`, `/hybrid_force_motion_controller/twist_cmd`, and `/forward_velocity_controller/commands`. Keep an e-stop handy; call `/hybrid_force_motion_controller/stop_motion` or kill the launch if anything deviates. The Cartesian velocity controller times out (`twist_timeout_s`, default 0.1 s) and zeros joint commands if the hybrid node stops publishing.

## Operator Interfaces
| Type   | Name | Notes |
|--------|------|-------|
| Service | `/hybrid_force_motion_controller/set_start_pose` (`std_srvs/Trigger`) | Captures current TF pose & wrench, zeros integrators, transitions to READY prior to the contact press. |
| Service | `/hybrid_force_motion_controller/start_motion` | Runs the entire sequence automatically: +Z approach to 5 N ± 0.2 N, ~1 s dwell, then a 5 cm tangential traverse projected into the live contact frame. |
| Service | `/hybrid_force_motion_controller/pause_motion` / `/resume_motion` | Freeze and continue the current 5 cm segment; remaining distance is preserved while the 5 N load hold stays active. |
| Service | `/hybrid_force_motion_controller/stop_motion` | Aborts the run, zeros the tangential distance accumulator, and requires `set_start_pose` before another start (the next run always performs a fresh 5 cm). |
| Topic | `/scaled_joint_trajectory_controller/joint_trajectory` | Publish a one-shot joint array to “teleport” the UR arm before running the controller. |
| Topic | `/hybrid_force_motion_controller/state` (`HybridForceMotionState`) | Finite-state-machine status, tangential progress, PI band info, pause/fault flags. |
| Topic | `/hybrid_force_motion_controller/direction` (`geometry_msgs/Vector3`, optional) | Live override for tangential direction hint. |
| Topic | `/hybrid_force_motion_controller/twist_cmd` (`geometry_msgs/TwistStamped`) | Base-frame Cartesian twist published by the hybrid node and consumed by `cartesian_velocity_controller`. |
| TF | `contact_frame` | Published each control cycle for RViz verification (normal = Z-axis). |

## What to Expect During a Run
- The controller enforces the configured `search_direction` (default base −Z) during SEEK and DWELL, so the press stays vertical until the `/netft/proc_base` wrench shows 5 N within ±0.2 N and the 1 s dwell timer expires—only then is any tangential motion allowed.
- Tangential velocity is recomputed every cycle inside the contact frame supplied by the friction-normal estimator (see `reference/friction_normal_estimator.md`), so unknown curvature simply tilts the frame while the probe orientation stays aligned.
- `contact_force_min_threshold_N` (default 0.1 N) gates the friction-aware normal estimator; raise it (e.g., to 1–2 N) if you want reorientation to happen only once a firm contact load is present. The estimator also requires a minimum tangential sliding speed, so it ignores dithering along the normal axis.
- `cartesian_velocity_controller` listens to `/hybrid_force_motion_controller/twist_cmd`, re-applies the Cartesian and joint velocity limits once (in base_link), and pushes `/forward_velocity_controller/commands`, so you can inspect the twist topic directly if you need to debug motions.
- Tangential progress is measured from the actual end-effector pose (projected along the live tangential axis), so the logged `tangential_distance` reflects what the tool really did, not just what was commanded.
- Each `start_motion` integrates exactly 0.05 m of tangential distance; `pause_motion`/`resume_motion` finish the remaining distance, while `stop_motion` resets the counter so the next run performs a full 5 cm.
- The PI loop retains the soft-start and anti-windup logic, so steady-state normal-force error remains below the sensor bias even while sliding or when the surface height varies.
- If `(F·n̂)` drops below the disengage threshold for `N` cycles the node zeroes velocity commands, enters `FAULT`, and logs `CONTACT_LOST`. Other FAULT causes include TF lookup failures, Jacobian ill-conditioning, NaNs, or limit saturation; `set_start_pose` is always required to recover.
- TF `contact_frame` should show the Z-axis following the corrected surface normal and the X-axis pointing along the commanded tangential direction—use it (or RViz) to confirm the probe stays aligned during the 5 cm traverse.

## Cross-References
- `plan.md` — full architecture, PI/contact-loss/FAULT logic, and validation plan.
- `reference/friction_normal_estimator.md` — derivation of the friction-aware surface-normal estimator adopted from the cited literature.

This package implements the workflow described above; tune gains and limits in `config/controller.yaml` as needed.
