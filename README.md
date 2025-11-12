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

## Development Roadmap (build it step by step)

### Phase 1 — Environment & Teleport Bring-Up (Simulation Test 1)
Goal: prove the UR5e + dome world, dome teleporter, and end-effector teleporter all work before touching the force-motion code.

1. Launch Gazebo plus the helper stack (URDF + dome + teleporter nodes) in one shot:
   ```bash
   ros2 launch hybrid_force_motion_controller teleport_bringup.launch.py \
     ur_type:=ur5e
   ```
   The launch file includes `ur_simulation_gz/ur_sim_control.launch.py` with the packaged `worlds/contact_dome.sdf`.
2. Move the red hemispherical dome (initially at `x=0.5 m`) relative to `base_link` from another terminal:
   ```bash
   ros2 run hybrid_force_motion_controller dome_teleporter.py \
     --x 0.55 --y 0.15 --z 0.07 --frame base_link --world ur_world
   ```
   - Use `--waypoints path/to/file.yaml` to replay multiple locations (`[{x:...,y:...,z:...,frame:...}, ...]`).
     Example waypoint file:
     ```yaml
     - {x: 0.55, y: 0.10, z: 0.07, frame: base_link}
     - {x: 0.45, y: -0.05, z: 0.08, frame: base_link}
     ```
   - `--entity-name` lets you target a different Gazebo model if needed.
3. Snap the robot TCP onto the dome using IK:
   ```bash
   ros2 service call /hybrid_force_motion_controller/teleport_tool \
     hybrid_force_motion_controller/srv/TeleportTool \
     "{pose: {position: {x: 0.55, y: 0.15, z: 0.12}, \
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
       base_frame: base_link}"
   ```
4. Test criteria (no controller yet):
   - Dome pose updates immediately in RViz/Gazebo when the teleporter runs.
   - Teleporting the tool puts the TCP exactly where requested (confirm via TF and `ros2 topic echo /joint_states`).
   - You can iterate “teleport dome → teleport TCP” for all planned waypoints without restarting Gazebo.

Only after Test 1 is green do we proceed to Phase 2.

### Phase 2 — Hybrid Force-Motion Loop (Simulation Test 2 + Hardware)
1. Launch the full stack (Gazebo, teleport helpers, wrench pipeline, and controller):
   ```bash
   ros2 launch hybrid_force_motion_controller hybrid_force_motion_sim.launch.py
   ```
   Hardware mirrors the same steps except you jog manually (per `ur_admittance_controller/README.md`) and start the node with:
   ```bash
   ros2 run hybrid_force_motion_controller hybrid_force_motion_node
   ```
2. With the tool in contact (teleported in sim, jogged on hardware), run:
   ```bash
   ros2 service call /hybrid_force_motion_controller/set_start_pose std_srvs/srv/Trigger {}
   ros2 service call /hybrid_force_motion_controller/start_motion std_srvs/srv/Trigger {}
   ```
   The controller soft-starts the normal force (0→5 N over ~0.7 s), then performs the 5 cm tangential slide. `pause_motion`, `resume_motion`, and `stop_motion` remain available at any time.
3. Test criteria:
   - `/hybrid_force_motion_controller/state` shows `RUNNING`, then `COMPLETED` at 5 cm of progress; `/forward_velocity_controller/commands` stays bounded.
   - TF `contact_frame` aligns with the dome normal; `/netft/proc_probe` reports ~5 N steady-state.
   - Forcing `(F·n̂)` below the disengage threshold trips `FAULT` (contact lost), zeros commands, and requires a fresh `set_start_pose`.
   - Services behave identically on hardware once you substitute manual jogging for teleporting.

## Operator Interfaces
| Type   | Name | Notes |
|--------|------|-------|
| Service | `/hybrid_force_motion_controller/set_start_pose` (`std_srvs/Trigger`) | Captures current TF pose & wrench, zeros integrators, transitions to READY (supersedes `init_robot`). |
| Service | `/hybrid_force_motion_controller/start_motion` | Arms RUNNING once engage force is met. |
| Service | `/hybrid_force_motion_controller/pause_motion` / `/resume_motion` | Hold/continue the tangential slide without losing progress. |
| Service | `/hybrid_force_motion_controller/stop_motion` | Enters ABORTED; requires `set_start_pose` before another run. |
| Service | `/hybrid_force_motion_controller/teleport_tool` (sim only) | IK-based snap-to-contact helper used alongside the dome teleporter. |
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
