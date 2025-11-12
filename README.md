# hybrid_force_motion_controller

Hybrid force-motion velocity controller that reuses the `ur_admittance_controller` wrench pipeline, holds a 5 N normal load, and slides 5 cm along a movable 9 cm hemisphere while commanding `/forward_velocity_controller/commands`.

> **Status**: design frozen (see `plan.md` for architecture, `reference/friction_normal_estimator.md` for the surface-normal math). This README focuses on operator workflow.

## Repository Setup
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ajaygunalan/ur_simulation_gz.git
git clone https://github.com/ajaygunalan/ur_admittance_controller.git
git clone <this repo> hybrid_force_motion_controller
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select hybrid_force_motion_controller && source install/setup.bash
```

## Quick Navigation
- **Phase 1 – Teleport Bring-Up:** see `phase_one_test.md`.
- **Phase 2 – Hybrid Force-Motion Loop:** see `phase_two_test.md`.
- **Plan/architecture:** see `plan.md`.
## Operator Interfaces
| Type   | Name | Notes |
|--------|------|-------|
| Service | `/hybrid_force_motion_controller/set_start_pose` (`std_srvs/Trigger`) | Captures current TF pose & wrench, zeros integrators, transitions to READY prior to the contact press. |
| Service | `/hybrid_force_motion_controller/start_motion` | Arms RUNNING once engage force is met. |
| Service | `/hybrid_force_motion_controller/pause_motion` / `/resume_motion` | Hold/continue the tangential slide without losing progress. |
| Service | `/hybrid_force_motion_controller/stop_motion` | Enters ABORTED; requires `set_start_pose` before another run. |
| Topic | `/scaled_joint_trajectory_controller/joint_trajectory` | Publish a one-shot joint array to “teleport” the UR arm before running the controller. |
| Topic | `/hybrid_force_motion_controller/state` | Finite-state-machine status, tangential progress, PI error, fault flags. |
| Topic | `/hybrid_force_motion_controller/direction` (`geometry_msgs/Vector3`, optional) | Live override for tangential direction hint. |
| TF | `contact_frame` | Published each control cycle for RViz verification (normal = Z-axis). |

## What to Expect During a Run
- Normal force is regulated with a PI loop and anti-windup; steady-state error should drop below sensor bias even during high-friction slides.
- A configurable soft-start (`normal.softstart_time_s`, default ~0.7 s) ramps the internal force setpoint from 0→5 N after RUNNING begins, so first contact is quiet even when friction is high.
- If `(F·n̂)` falls below the disengage threshold for `N` cycles, the node zeros commands, enters `FAULT`, and logs `CONTACT_LOST`.
- Additional FAULT causes: TF lookup failures, Jacobian ill-conditioning, NaN inputs, or saturated safety limits. Recovery always starts with `set_start_pose`.
- The live contact frame in TF should align with the hemisphere normal. If it doesn’t, use the teleport tools or revisit calibration before proceeding.

## Cross-References
- `plan.md` — full architecture, PI/contact-loss/FAULT logic, and validation plan.
- `reference/friction_normal_estimator.md` — derivation of the friction-aware surface-normal estimator adopted from the cited literature.

Once you’re comfortable with this workflow, we’ll implement the package, launch files, and helpers exactly as described.
