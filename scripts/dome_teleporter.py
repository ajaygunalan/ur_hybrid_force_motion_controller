#!/usr/bin/env python3
import argparse
import math
import sys
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from ros_gz_interfaces.srv import SetEntityPose
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.utilities import remove_ros_args
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
import yaml

WORLD_FRAME = "world"


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def build_pose(x: float, y: float, z: float,
               roll: float, pitch: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def parse_waypoint_file(path: Path) -> List[dict]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, list):
        raise ValueError("Waypoint file must contain a list of poses")
    waypoints = []
    for idx, entry in enumerate(data):
        if not isinstance(entry, dict):
            raise ValueError(f"Waypoint {idx} is not a dict")
        wp = {
            'x': entry.get('x', 0.0),
            'y': entry.get('y', 0.0),
            'z': entry.get('z', 0.0),
            'roll': entry.get('roll', 0.0),
            'pitch': entry.get('pitch', 0.0),
            'yaw': entry.get('yaw', 0.0),
            'frame': entry.get('frame', 'base_link'),
        }
        waypoints.append(wp)
    return waypoints


class DomeTeleporter(Node):
    def __init__(self, entity_name: str, world: str):
        super().__init__('dome_teleporter')
        self.entity_name = entity_name
        self.world_name = world
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        service_name = f'/world/{self.world_name}/set_entity_pose'
        self.client = self.create_client(SetEntityPose, service_name)
        self.get_logger().info(f"Waiting for {service_name} (entity '{self.entity_name}')")
        self.client.wait_for_service()

    def _pose_to_world(self, pose: Pose, frame: str) -> Pose:
        if frame == WORLD_FRAME:
            return pose
        stamped = PoseStamped()
        stamped.header.frame_id = frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = pose
        try:
            transform = self.tf_buffer.lookup_transform(
                WORLD_FRAME, frame, rclpy.time.Time(), timeout=Duration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
            raise RuntimeError(f"TF transform from {frame} to {WORLD_FRAME} unavailable: {exc}")
        world_pose = do_transform_pose(stamped, transform)
        return world_pose.pose

    def teleport_once(self, pose: Pose, frame: str) -> bool:
        pose_world = self._pose_to_world(pose, frame)
        req = SetEntityPose.Request()
        req.entity.name = self.entity_name
        req.entity.type = req.entity.MODEL
        req.pose = pose_world
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            return False
        return True


def run_single(node: DomeTeleporter, args: argparse.Namespace) -> bool:
    pose = build_pose(args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
    frame = args.frame
    return node.teleport_once(pose, frame)


def run_waypoints(node: DomeTeleporter, waypoints: List[dict], sleep: float) -> bool:
    for idx, wp in enumerate(waypoints):
        pose = build_pose(wp['x'], wp['y'], wp['z'], wp['roll'], wp['pitch'], wp['yaw'])
        frame = wp.get('frame', 'base_link')
        node.get_logger().info(f"Teleporting waypoint {idx+1}/{len(waypoints)} in frame {frame}")
        if not node.teleport_once(pose, frame):
            return False
        if idx < len(waypoints) - 1 and sleep > 0:
            time.sleep(sleep)
    return True


def create_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Teleport the contact dome in Gazebo.')
    parser.add_argument('--entity-name', default='contact_dome', help='Gazebo entity/model name')
    parser.add_argument('--frame', default='base_link', help='Input pose frame (default base_link). Use world/world_n for absolute.')
    parser.add_argument('--world', default='ur_world', help='Gazebo Sim world name hosting the dome model.')
    parser.add_argument('--x', type=float, default=0.6)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.05)
    parser.add_argument('--roll', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--waypoints', type=str, help='YAML file with an array of waypoint dicts {x,y,z,roll,pitch,yaw,frame}')
    parser.add_argument('--sleep', type=float, default=1.0, help='Seconds between waypoint teleports')
    return parser


def main():
    rclpy.init()
    parser = create_arg_parser()
    clean_args = remove_ros_args(sys.argv)
    parsed_args = parser.parse_args(clean_args[1:])

    node = DomeTeleporter(entity_name=parsed_args.entity_name, world=parsed_args.world)
    success = False
    try:
        if parsed_args.waypoints:
            waypoints = parse_waypoint_file(Path(parsed_args.waypoints))
            success = run_waypoints(node, waypoints, parsed_args.sleep)
        else:
            success = run_single(node, parsed_args)
    except Exception as exc:  # pylint:disable=broad-except
        node.get_logger().error(str(exc))
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
