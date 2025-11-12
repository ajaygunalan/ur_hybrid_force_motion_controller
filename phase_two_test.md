# Phase 2 – Hybrid Force‑Motion Loop

Phase 1 positioned the dome and UR joints above the contact point (no force yet). Phase 2 starts from that hovering pose and runs the hybrid controller so the probe first **moves down along +Z** to make contact and build approximately 5 N of normal load, then **slides tangentially** along the dome for 5 cm while you monitor the wrench feedback coming from `ur_admittance_controller`’s `wrench_node`.

---

## Preconditions
1. Gazebo + controller stack running:  
   `ros2 launch hybrid_force_motion_controller hybrid_force_motion_sim.launch.py ur_type:=ur5e`
2. `ur_admittance_controller` nodes (especially `wrench_node`) are active—the hybrid controller subscribes to `/netft/proc_probe` for real-time force feedback.
3. Dome and UR joints already positioned via the Phase 1 commands:
   ```bash
   # Dome pose
   gz service -s /world/contact_dome/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
     --req 'name: "contact_dome", position: { x: 0.55, y: 0.135, z: 0.00 }, orientation: { x: 0, y: 0, z: 0, w: 1 }}'

   # UR joint preset (edit the array inline for each test)
   ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory \
     trajectory_msgs/msg/JointTrajectory \
     "{joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint', \
                     'wrist_1_joint','wrist_2_joint','wrist_3_joint'], \
       points: [{positions: [0.0, -1.3, 1.7, -1.9, -1.57, 0.0], \
                 time_from_start: {sec: 5, nanosec: 0}}]}"
   ```

---

## Step 1 – Start the Hybrid Controller
```bash
ros2 service call /hybrid_force_motion_controller/set_start_pose std_srvs/srv/Trigger {}
ros2 service call /hybrid_force_motion_controller/start_motion std_srvs/srv/Trigger {}
```
Result:
- `/hybrid_force_motion_controller/state`: `READY → RUNNING`.
- Controller begins its **contact approach**: drives +Z velocity until measured normal force ≥ engage threshold (~5 N).
- Keep an eye on `/netft/proc_probe` (filtered wrench from `wrench_node`) to confirm the load is building smoothly; the PI loop soft‑starts over ~0.7 s so first contact is gentle.

---

## Step 2 – Hold & Verify Contact (optional)
Once `/netft/proc_probe` shows ~5 N and `/hybrid_force_motion_controller/state` indicates a steady-contact substate (e.g., `RUNNING` with `normal_progress ≈ 1`), you can pause to inspect alignment:
```bash
ros2 service call /hybrid_force_motion_controller/pause_motion std_srvs/srv/Trigger {}
```
- Joints hold position; you may jog orientation manually if the probe isn’t square on the dome.
- Resume when you’re satisfied:
  ```bash
  ros2 service call /hybrid_force_motion_controller/resume_motion std_srvs/srv/Trigger {}
  ```

> *This pause/resume checkpoint is where the “user approval” happens—confirm the probe is seated, then continue.*

---

## Step 3 – Tangential Slide
After the pause/resume (or immediately if you skipped it), the controller switches to tangential motion:
- Drives in the commanded surface direction (default 5 cm target) using the same Cartesian velocity loop.
- Maintains the +Z force by regulating the PI loop around the 5 N setpoint; if the dome curvature causes force drop, the controller re-enters the Z-press logic automatically.
- Expected completion: `/hybrid_force_motion_controller/state = COMPLETED` once tangential progress ≥ 0.05 m.

If you need to abort early:
```bash
ros2 service call /hybrid_force_motion_controller/stop_motion std_srvs/srv/Trigger {}
```
- State becomes `ABORTED`; re-run `set_start_pose` before starting again.

---

## Monitoring & Validation
| Signal | Expectation |
|--------|-------------|
| `/netft/proc_probe` | Normal component ramps to ~5 N during contact, stays steady during slide. |
| `/hybrid_force_motion_controller/state` | `READY → RUNNING → COMPLETED` (or `FAULT` if contact lost). |
| `/forward_velocity_controller/commands` | Shows +Z velocity during approach, then tangential velocities. |
| TF `contact_frame` | `Z` axis aligns with dome normal; `X/Y` axes follow tangential motion. |

**Fault Handling:** If contact is lost (e.g., force < threshold for several cycles), the node enters `FAULT`, zeroes commands, and logs `CONTACT_LOST`. To recover: fix pose, call `set_start_pose`, then `start_motion` again.

---

## Key Objectives Recap
1. **Contact Initiation:** Only the +Z channel is active until 5 N is achieved—watch the wrench data to confirm.
2. **User Approval:** Pause (optional) to verify alignment/orientation before sliding along the curved surface.
3. **Tangential Motion:** After approval, the controller advances 5 cm along the dome while maintaining the 5 N normal load.

Repeat the process for each dome pose by re-running the Phase 1 commands, then follow this checklist to validate the full hybrid force-motion loop.
