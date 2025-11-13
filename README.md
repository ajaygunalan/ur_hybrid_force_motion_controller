# hybrid_force_motion_controller

Hybrid force-motion velocity controller that reuses the `ur_admittance_controller` wrench pipeline, holds a **5 N ± 0.2 N** normal load, automatically dwells for ~1 s once the load stabilizes, and then slides **exactly 5 cm per run** in the tangent plane of an otherwise unknown curved surface while commanding `/forward_velocity_controller/commands`.

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
| Service | `/hybrid_force_motion_controller/start_motion` | Runs the entire sequence automatically: +Z approach to 5 N ± 0.2 N, ~1 s dwell, then a 5 cm tangential traverse projected into the live contact frame. |
| Service | `/hybrid_force_motion_controller/pause_motion` / `/resume_motion` | Freeze and continue the current 5 cm segment; remaining distance is preserved while the 5 N load hold stays active. |
| Service | `/hybrid_force_motion_controller/stop_motion` | Aborts the run, zeros the tangential distance accumulator, and requires `set_start_pose` before another start (the next run always performs a fresh 5 cm). |
| Topic | `/scaled_joint_trajectory_controller/joint_trajectory` | Publish a one-shot joint array to “teleport” the UR arm before running the controller. |
| Topic | `/hybrid_force_motion_controller/state` | Finite-state-machine status, tangential progress, PI error, fault flags. |
| Topic | `/hybrid_force_motion_controller/direction` (`geometry_msgs/Vector3`, optional) | Live override for tangential direction hint. |
| TF | `contact_frame` | Published each control cycle for RViz verification (normal = Z-axis). |

## What to Expect During a Run
- The controller drives only +Z velocity until `/netft/proc_probe` reports 5 N within ±0.2 N, then automatically dwells for ~1 s before allowing any XY motion.
- Tangential velocity is recomputed every cycle inside the contact frame supplied by the friction-normal estimator (see `reference/friction_normal_estimator.md`), so unknown curvature simply tilts the frame while the probe orientation stays aligned.
- Each `start_motion` integrates exactly 0.05 m of tangential distance; `pause_motion`/`resume_motion` finish the remaining distance, while `stop_motion` resets the counter so the next run performs a full 5 cm.
- The PI loop retains the soft-start and anti-windup logic, so steady-state normal-force error remains below the sensor bias even while sliding or when the surface height varies.
- If `(F·n̂)` drops below the disengage threshold for `N` cycles the node zeroes velocity commands, enters `FAULT`, and logs `CONTACT_LOST`. Other FAULT causes include TF lookup failures, Jacobian ill-conditioning, NaNs, or limit saturation; `set_start_pose` is always required to recover.
- TF `contact_frame` should show the Z-axis following the corrected surface normal and the X-axis pointing along the commanded tangential direction—use it (or RViz) to confirm the probe stays aligned during the 5 cm traverse.

## Cross-References
- `plan.md` — full architecture, PI/contact-loss/FAULT logic, and validation plan.
- `reference/friction_normal_estimator.md` — derivation of the friction-aware surface-normal estimator adopted from the cited literature.

Once you’re comfortable with this workflow, we’ll implement the package, launch files, and helpers exactly as described.
