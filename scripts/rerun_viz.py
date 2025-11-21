#!/usr/bin/env python3
"""
Rerun visualization bridge for UR hybrid_force_motion_controller (ROS 2 Jazzy).

MERGED BEST OF ALL 4 APPROACHES:
- Uses venv for clean isolation (Option 2 approach)
- TF tree with proper hierarchy (Option 3)
- Complete state logging (Option 1)
- Modern Rerun API (not deprecated)

Prerequisites:
  1. source ~/ros2_ws/rerun_venv/bin/activate
  2. source ~/ros2_ws/install/setup.bash
  3. Run: python3 src/hybrid_force_motion_controller/scripts/rerun_viz.py

Topics subscribed:
  /tf, /tf_static              → Robot kinematics + contact_frame
  /netft/proc_base             → Wrench (5N normal force, friction)
  /hybrid_force_motion_controller/twist_cmd  → Cartesian velocity command
  /hybrid_force_motion_controller/state      → FSM state, force/distance, faults

Timelines:
  - ros_time: seconds (from ROS 2 message stamps)

Rerun root: "world"
"""

from typing import Optional
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import WrenchStamped, TwistStamped, Transform
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException

import rerun as rr

# Import custom message (ensure ~/ros2_ws/install/setup.bash sourced)
try:
    from hybrid_force_motion_controller.msg import HybridForceMotionState
    HAS_STATE_MSG = True
except ImportError:
    HAS_STATE_MSG = False
    print("[WARN] Could not import HybridForceMotionState; state logging disabled.")


def ros_stamp_to_seconds(sec: int, nanosec: int) -> float:
    """Convert ROS builtin_interfaces/Time to float seconds."""
    return float(sec) + float(nanosec) * 1e-9


def quat_to_rerun_format(x: float, y: float, z: float, w: float):
    """
    Convert ROS quaternion (x,y,z,w) to Rerun format.
    
    Rerun's rr.Quaternion() expects xyzw by default.
    """
    return rr.Quaternion(xyzw=[x, y, z, w])


class URHFMCRerunViz(Node):
    """Bridge node: ROS 2 topics → Rerun streaming."""

    def __init__(self) -> None:
        super().__init__("ur_hfmc_rerun_viz")

        # Frame names (match your controller config)
        self.world_frame = "world"
        self.base_link = "base_link"
        self.contact_frame = "contact_frame"
        self.tool_link = "p42v_link1"  # UR tool probe

        # Visualization scaling
        self.force_scale = 0.05      # N -> m for arrow length
        self.twist_scale = 1.0       # (m/s, rad/s) -> m
        self.axis_length = 0.08      # coordinate frame axis length

        # --- TF pipeline ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriptions ---
        self.create_subscription(
            TFMessage,
            "/tf",
            lambda msg: self._on_tf(msg, is_static=False),
            50,
        )
        self.create_subscription(
            TFMessage,
            "/tf_static",
            lambda msg: self._on_tf(msg, is_static=True),
            10,
        )

        self.create_subscription(
            WrenchStamped,
            "/netft/proc_base",
            self._on_wrench,
            10,
        )

        self.create_subscription(
            TwistStamped,
            "/hybrid_force_motion_controller/twist_cmd",
            self._on_twist,
            10,
        )

        if HAS_STATE_MSG:
            self.create_subscription(
                HybridForceMotionState,
                "/hybrid_force_motion_controller/state",
                self._on_state,
                10,
            )

        self.get_logger().info(
            f"URHFMCRerunViz initialized. Frames: base={self.base_link}, "
            f"tool={self.tool_link}, contact={self.contact_frame}"
        )

    # =========================================================================
    # TF: Frames + Kinematics
    # =========================================================================

    def _on_tf(self, msg: TFMessage, is_static: bool) -> None:
        """
        Forward TF transforms to Rerun as Transform3D objects.
        
        Builds hierarchical entity paths so Rerun reconstructs the kinematic tree.
        """
        for t in msg.transforms:
            parent_id = t.header.frame_id.lstrip("/") or self.world_frame
            child_id = t.child_frame_id.lstrip("/")

            # Rerun path hierarchy: world/parent/child
            if parent_id == self.world_frame:
                parent_path = self.world_frame
            else:
                parent_path = f"{self.world_frame}/{parent_id}"
            child_path = f"{parent_path}/{child_id}"

            # Only dynamic TF updates the timeline
            if not is_static:
                ts = ros_stamp_to_seconds(
                    t.header.stamp.sec, t.header.stamp.nanosec
                )
                rr.set_time("ros_time", timestamp=ts)

            # Extract translation & rotation
            tf_data = t.transform
            translation = [
                tf_data.translation.x,
                tf_data.translation.y,
                tf_data.translation.z,
            ]
            rot = tf_data.rotation
            rotation = quat_to_rerun_format(rot.x, rot.y, rot.z, rot.w)

            # Key frames get visible axes; others are just transforms
            show_axes = child_id in (
                self.base_link,
                self.tool_link,
                self.contact_frame,
            )

            rr.log(
                child_path,
                rr.Transform3D(
                    translation=translation,
                    rotation=rotation,
                    axis_length=self.axis_length if show_axes else None,
                ),
                static=is_static,
            )

    # =========================================================================
    # Wrench: /netft/proc_base
    # =========================================================================

    def _on_wrench(self, msg: WrenchStamped) -> None:
        """Log processed base-frame wrench as a 3D arrow + scalars."""
        ts = ros_stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        rr.set_time("ros_time", timestamp=ts)

        force = np.array(
            [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            dtype=np.float32,
        )
        force_mag = float(np.linalg.norm(force))

        # 3D arrow in base_link (scaled for visibility)
        rr.log(
            "world/base_link/wrench",
            rr.Arrows3D(
                origins=np.zeros((1, 3), dtype=np.float32),
                vectors=[force * self.force_scale],
                colors=[[255, 100, 100, 255]],  # Red
                radii=[0.004],
                labels=[f"F: {force_mag:.2f}N"],
            ),
        )

        # Scalars for time-series plot
        rr.log("plots/wrench_fx", rr.Scalars(msg.wrench.force.x))
        rr.log("plots/wrench_fy", rr.Scalars(msg.wrench.force.y))
        rr.log("plots/wrench_fz", rr.Scalars(msg.wrench.force.z))
        rr.log("plots/force_magnitude", rr.Scalars(force_mag))

    # =========================================================================
    # Twist: /hybrid_force_motion_controller/twist_cmd
    # =========================================================================

    def _on_twist(self, msg: TwistStamped) -> None:
        """Log Cartesian velocity command as linear + angular arrows + scalars."""
        ts = ros_stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        rr.set_time("ros_time", timestamp=ts)

        lin_vel = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            dtype=np.float32,
        )
        ang_vel = np.array(
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
            dtype=np.float32,
        )

        lin_mag = float(np.linalg.norm(lin_vel))
        ang_mag = float(np.linalg.norm(ang_vel))

        # Linear velocity arrow (Green)
        rr.log(
            "world/base_link/twist_linear",
            rr.Arrows3D(
                origins=np.zeros((1, 3), dtype=np.float32),
                vectors=[lin_vel * self.twist_scale],
                colors=[[100, 255, 100, 255]],  # Green
                radii=[0.003],
                labels=[f"v_lin: {lin_mag:.4f} m/s"],
            ),
        )

        # Angular velocity arrow (Blue) if non-negligible
        if ang_mag > 1e-6:
            rr.log(
                "world/base_link/twist_angular",
                rr.Arrows3D(
                    origins=np.zeros((1, 3), dtype=np.float32),
                    vectors=[ang_vel * self.twist_scale],
                    colors=[[100, 100, 255, 255]],  # Blue
                    radii=[0.003],
                    labels=[f"ω: {ang_mag:.4f} rad/s"],
                ),
            )

        # Scalars
        rr.log("plots/twist_vx", rr.Scalars(msg.twist.linear.x))
        rr.log("plots/twist_vy", rr.Scalars(msg.twist.linear.y))
        rr.log("plots/twist_vz", rr.Scalars(msg.twist.linear.z))
        rr.log("plots/twist_linear_speed", rr.Scalars(lin_mag))

    # =========================================================================
    # Hybrid Controller State: /hybrid_force_motion_controller/state
    # =========================================================================

    def _on_state(self, msg: HybridForceMotionState) -> None:
        """
        Log all internal hybrid controller state for diagnostics.
        
        - Normal force PI loop (actual, target, error)
        - Tangential progress
        - FSM state & phase
        - Dwell, paused, fault flags
        """
        if not HAS_STATE_MSG:
            return

        ts = ros_stamp_to_seconds(msg.stamp.sec, msg.stamp.nanosec)
        rr.set_time("ros_time", timestamp=ts)

        # --- Normal Force Regulation ---
        rr.log("hfmc/normal_force/actual", rr.Scalars(msg.normal_force))
        rr.log("hfmc/normal_force/target", rr.Scalars(msg.normal_force_target))
        rr.log("hfmc/normal_force/error", rr.Scalars(msg.normal_force_error))

        # --- Tangential Progress (5 cm slide) ---
        rr.log("hfmc/tangential/distance", rr.Scalars(msg.tangential_distance))
        rr.log("hfmc/tangential/target", rr.Scalars(msg.tangential_target))

        # --- FSM State & Phase (as enums → floats) ---
        # state: WAITING_FOR_START=0, READY=1, RUNNING=2, PAUSED=3, COMPLETED=4, ABORTED=5, FAULT=6
        # phase: SEEK=0, DWELL=1, TANGENTIAL=2
        rr.log("hfmc/fsm/state", rr.Scalars(float(msg.state)))
        rr.log("hfmc/fsm/phase", rr.Scalars(float(msg.phase)))

        # --- Flags (boolean → 0/1) ---
        rr.log(
            "hfmc/flags/dwell_active",
            rr.Scalars(1.0 if msg.dwell_active else 0.0),
        )
        rr.log("hfmc/flags/paused", rr.Scalars(1.0 if msg.paused else 0.0))
        rr.log(
            "hfmc/flags/fault_active",
            rr.Scalars(1.0 if msg.fault_active else 0.0),
        )

        # --- Fault Reason (text log if fault active) ---
        if msg.fault_active and msg.fault_reason:
            rr.log("hfmc/fault_reason", rr.TextLog(msg.fault_reason))


def main() -> None:
    """Initialize Rerun, then spin ROS 2 node."""
    # Start Rerun viewer automatically
    rr.init("ur_hybrid_force_motion", spawn=True)

    # Set coordinate convention: Right-handed Z-up (matches ROS/UR)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # Bring up ROS 2
    rclpy.init()
    node = URHFMCRerunViz()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
