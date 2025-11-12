# Phase 1 – Teleport Bring-Up Test

Use this checklist after building the workspace (`colcon build --packages-select hybrid_force_motion_controller && source install/setup.bash`). No controller nodes run during this test.

1. **Launch environment + helpers**
   ```bash
   ros2 launch hybrid_force_motion_controller teleport_bringup.launch.py ur_type:=ur5e
   ```
2. **Move the dome** (repeat for each pose as needed) using Gazebo’s CLI service:
   ```bash
   gz service -s /world/contact_dome/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: "contact_dome", position: { x: 0.55, y: 0.135, z: 0.00 }, orientation: { x: 0, y: 0, z: 0, w: 1 }'
   ```
   *Tip*: run `gz service -l | grep set_pose` first if you’re unsure about the world name, and only edit the pose coordinates/orientation (they’re expressed in the `world` frame).
3. **Snap the UR arm joints** (edit values inline as needed):
   ```bash
   ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory \
     trajectory_msgs/msg/JointTrajectory \
     "{joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint', \
                     'wrist_1_joint','wrist_2_joint','wrist_3_joint'], \
       points: [{positions: [0.0, -1.3, 1.7, -1.9, -1.57, 0.0], \
                 time_from_start: {sec: 5, nanosec: 0}}]}"
   ```
   *Update the `positions` array each time you want a new pose; the controller immediately drives the joints to those angles.*
4. **Validate**
   - RViz/Gazebo show the dome and TCP moving instantly to each target.
   - `ros2 topic echo /joint_states` matches the joint values you just sent.
   - TF `contact_dome` and `tool0` frames align at each commanded pose.


Mark **Test 1** as PASS in `agents.md` once the above steps succeed without restarting Gazebo.
