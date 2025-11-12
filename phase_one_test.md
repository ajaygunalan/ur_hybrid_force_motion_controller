# Phase 1 – Teleport Bring-Up Test

Use this checklist after building the workspace (`colcon build --packages-select hybrid_force_motion_controller && source install/setup.bash`). No controller nodes run during this test.

1. **Launch environment + helpers**
   ```bash
   ros2 launch hybrid_force_motion_controller teleport_bringup.launch.py ur_type:=ur5e
   ```
2. **Move the dome** (repeat for each waypoint as needed):
   ```bash
   ros2 run hybrid_force_motion_controller dome_teleporter.py \
    --x 0.55 --y 0.15 --z 0.07 --frame base_link --world ur_world
   ```
   *Optional waypoints file (`waypoints.yaml`):*
   ```yaml
   - {x: 0.55, y: 0.10, z: 0.07, frame: base_link}
   - {x: 0.45, y: -0.05, z: 0.08, frame: base_link}
   ```
   ```bash
   ros2 run hybrid_force_motion_controller dome_teleporter.py --waypoints waypoints.yaml
   ```
3. **Teleport the TCP via IK** (match the dome pose):
   ```bash
   ros2 service call /hybrid_force_motion_controller/teleport_tool \
     hybrid_force_motion_controller/srv/TeleportTool \
     "{pose: {position: {x: 0.55, y: 0.15, z: 0.12}, \
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
       base_frame: base_link}"
   ```
4. **Validate**
   - RViz/Gazebo show the dome and TCP moving instantly to each target.
   - `ros2 topic echo /joint_states` matches the expected joint configurations.
   - TF `contact_dome` and `tool0` frames align at each waypoint.

Mark **Test 1** as PASS in `agents.md` once the above steps succeed without restarting Gazebo.
