# Phase 2 – Contact Seek + Tangential Traverse

Phase 2 begins immediately after Phase 1 leaves the probe hovering over the surface. The controller now runs end‑to‑end: it drives straight down along tool +Z until the normal load stabilizes, dwells for a second, then slides exactly 5 cm in the local tangent plane while keeping the same 5 N push.

## Launch & Kickoff
```bash
# Start the sim or hardware stack (choose the launch that matches your setup)
ros2 launch hybrid_force_motion_controller hybrid_force_motion_sim.launch.py ur_type:=ur5e

# From the hover pose captured during Phase 1
ros2 service call /hybrid_force_motion_controller/set_start_pose std_srvs/srv/Trigger {}
ros2 service call /hybrid_force_motion_controller/start_motion std_srvs/srv/Trigger {}
```
`set_start_pose` is still the only manual gate: once `start_motion` is issued the controller takes care of contact, dwell, and tangential motion without further operator intervention.

## Controller Behavior
### Contact seek (automatic)
- Commands only +Z Cartesian velocity until the measured normal force hits **5 N ± 0.2 N** (force data comes from `ur_admittance_controller`’s `/netft/proc_probe` stream).
- The PI loop uses the existing soft-start so the load ramps smoothly; the moment the force enters the ±0.2 N window the controller holds position and times **~1 s** to let the load settle.

### Tangential traverse (automatic)
- After the dwell, the node enables XY velocity. The command is always reprojected into the live tangent plane so it respects whatever curvature exists under the tool.
- Displacement is integrated until **0.05 m** is accumulated. Starting again later always produces another 5 cm chunk; pausing and resuming finishes the remaining distance; stopping discards progress so the next `start_motion` begins a full 5 cm again.

### Unknown-surface adaptation
- Each cycle the friction-normal estimator described in `reference/friction_normal_estimator.md` combines the sensed wrench with the Cartesian velocity to recover the true surface normal.
- That corrected normal defines the contact frame published on TF, keeps the probe orientation aligned, and provides the tangent directions used by the XY controller. Curvature therefore shows up only as a slow rotation of the contact frame, while the Z loop keeps the load in band.

## Operator Controls
- `pause_motion` / `resume_motion`: freezes the tangential distance counter in place and restarts exactly where it left off; the 5 N force hold remains active the entire time.
- `stop_motion`: aborts the run, zeros the distance accumulator, and requires `set_start_pose` before another start.
- Reissuing `start_motion` after a completed run kicks off the next 5 cm segment without extra configuration.

## Observability
- `/hybrid_force_motion_controller/state`: exposes the finite-state machine, tangential distance, current normal force, and whether the auto-dwell is counting.
- `/netft/proc_probe`: confirm the load enters and stays within 5 N ± 0.2 N during both the seek and the slide.
- TF `contact_frame`: its Z-axis should remain aligned with the corrected surface normal; the X-axis points along the instantaneous tangential command so you can see the planned 5 cm direction.

If contact is lost (force drops below the disengage threshold for too long) the node enters `FAULT`, zeros commands, and logs `CONTACT_LOST`. Re-run `set_start_pose` and `start_motion` once the probe is repositioned.
