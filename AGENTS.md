# AGENTS Log

## Phase Tracker
| Phase | Scope | Key Assets | Test ID | Status | Notes |
|-------|-------|------------|---------|--------|-------|
| 1 | Environment + teleport bring-up (sim only) | `worlds/contact_dome.sdf`, `teleport_bringup.launch.py`, `gz service /world/contact_dome/set_pose`, `/scaled_joint_trajectory_controller/joint_trajectory` CLI | Test 1 | Pending | Follow `phase_one_test.md` to verify dome & joint snap commands before enabling the controller. |
| 2 | Hybrid force-motion controller (sim + hw) | controller node, PI loop, services | Test 2 | Not started | Follow `phase_two_test.md` once Phase 1 is green. |

## Test Notes
- **Test 1 (Phase 1)**: Requires Gazebo + ROS stack running. Steps in `phase_one_test.md`. Not executed in this workspace (simulation-heavy, user will run on target machine).
- **Test 2 (Phase 2)**: Detailed workflow in `phase_two_test.md`.

Use this file to record PASS/FAIL as phases advance; reference `plan.md` for architecture and `README.md` for operator commands.
