#!/usr/bin/env python3
import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

from urdf_parser_py.urdf import URDF
import PyKDL as kdl

from hybrid_force_motion_controller.srv import TeleportTool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WORLD_FRAME = 'world'
DEFAULT_BASE_FRAME = 'base_link'
DEFAULT_TOOL_FRAME = 'tool0'
DEFAULT_TRAJECTORY_TOPIC = '/scaled_joint_trajectory_controller/joint_trajectory'


class EndEffectorTeleporter(Node):
    def __init__(self):
        super().__init__('ee_teleporter')
        self.declare_parameter('model_name', 'ur5e')
        self.declare_parameter('urdf_param', 'robot_description')
        self.declare_parameter('robot_state_publisher', 'robot_state_publisher')
        self.declare_parameter('base_frame', DEFAULT_BASE_FRAME)
        self.declare_parameter('tool_frame', DEFAULT_TOOL_FRAME)
        self.declare_parameter('trajectory_topic', DEFAULT_TRAJECTORY_TOPIC)
        self.declare_parameter('tolerance', 1e-3)
        self.declare_parameter('timeout', 5.0)

        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.urdf_param = self.get_parameter('urdf_param').get_parameter_value().string_value
        rsp_name = self.get_parameter('robot_state_publisher').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tool_frame = self.get_parameter('tool_frame').get_parameter_value().string_value
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.chain = None
        self.joint_names: List[str] = []
        self.q_min = None
        self.q_max = None
        self.ik_solver = None
        self.joint_state: Optional[JointState] = None

        self._create_joint_state_sub()
        self._load_kdl_chain(rsp_name)

        world = self.declare_parameter('world', 'ur_world').get_parameter_value().string_value
        self.world_name = world
        service_name = f'/world/{self.world_name}/set_model_configuration'
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.traj_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, qos)
        self.get_logger().info(f'Publishing joint trajectories to {self.trajectory_topic}')

        self.teleport_srv = self.create_service(TeleportTool,
                                               '/hybrid_force_motion_controller/teleport_tool',
                                               self.handle_teleport)
        self.get_logger().info('End-effector teleporter ready')

    def _create_joint_state_sub(self):
        qos = QoSProfile(depth=10)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, qos)

    def _joint_state_cb(self, msg: JointState):
        self.joint_state = msg

    def _load_kdl_chain(self, rsp_name: str):
        from rclpy.parameter_client import AsyncParameterClient
        client = AsyncParameterClient(self, rsp_name)
        self.get_logger().info(f"Waiting for parameter service on {rsp_name}")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('...still waiting for robot_state_publisher')
        future = client.get_parameters([self.urdf_param])
        rclpy.spin_until_future_complete(self, future)
        params = future.result()
        if not params:
            raise RuntimeError(f"Parameter {self.urdf_param} not provided by {rsp_name}")
        robot_description = params[0].string_value
        if not robot_description:
            raise RuntimeError(f"Parameter {self.urdf_param} is empty")
        urdf_model = URDF.from_xml_string(robot_description)
        chain = self._build_chain_from_urdf(urdf_model)
        self.chain = chain
        joint_info = self._extract_joint_info(urdf_model)
        self.joint_names = [name for name, _ in joint_info]
        self.q_min = kdl.JntArray(len(joint_info))
        self.q_max = kdl.JntArray(len(joint_info))
        for idx, (_, limits) in enumerate(joint_info):
            self.q_min[idx] = limits[0]
            self.q_max[idx] = limits[1]
        fk = kdl.ChainFkSolverPos_recursive(self.chain)
        ik_vel = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_NR_JL(self.chain, self.q_min, self.q_max, fk, ik_vel, 200, 1e-6)
        self.get_logger().info(f"Loaded KDL chain with {len(self.joint_names)} joints")

    def _build_chain_from_urdf(self, robot_model: URDF) -> kdl.Chain:
        chain = kdl.Chain()
        joint_names = robot_model.get_chain(self.base_frame, self.tool_frame, links=False, fixed=True)
        for joint_name in joint_names:
            joint = robot_model.joint_map[joint_name]
            origin = joint.origin
            xyz = origin.xyz if origin is not None else [0.0, 0.0, 0.0]
            rpy = origin.rpy if origin is not None else [0.0, 0.0, 0.0]
            frame = kdl.Frame(
                kdl.Rotation.RPY(*rpy),
                kdl.Vector(*xyz),
            )
            axis = joint.axis if joint.axis is not None else [0.0, 0.0, 1.0]
            axis_vec = kdl.Vector(*axis)
            if joint.type in ('revolute', 'continuous'):
                kdl_joint = kdl.Joint(joint.name, kdl.Vector(0.0, 0.0, 0.0), axis_vec, kdl.Joint.RotAxis)
            elif joint.type == 'prismatic':
                kdl_joint = kdl.Joint(joint.name, kdl.Vector(0.0, 0.0, 0.0), axis_vec, kdl.Joint.TransAxis)
            else:
                kdl_joint = kdl.Joint(joint.name, kdl.Joint.Fixed)
            segment = kdl.Segment(joint.child, kdl_joint, frame)
            chain.addSegment(segment)
        return chain

    def _extract_joint_info(self, urdf_model: URDF) -> List[tuple]:
        info = []
        joint_names = urdf_model.get_chain(self.base_frame, self.tool_frame, links=False, fixed=False)
        for name in joint_names:
            joint = urdf_model.joint_map[name]
            limits = (-math.pi, math.pi)
            if joint.limit is not None:
                lower = joint.limit.lower if joint.limit.lower is not None else -math.pi
                upper = joint.limit.upper if joint.limit.upper is not None else math.pi
                limits = (lower, upper)
            info.append((name, limits))
        return info

    def _current_joint_array(self) -> Optional[kdl.JntArray]:
        if self.joint_state is None:
            return None
        name_to_pos = dict(zip(self.joint_state.name, self.joint_state.position))
        q = kdl.JntArray(len(self.joint_names))
        for idx, name in enumerate(self.joint_names):
            if name not in name_to_pos:
                self.get_logger().warning(f"Joint {name} missing from JointState; using 0 seed")
                q[idx] = 0.0
            else:
                q[idx] = name_to_pos[name]
        return q

    def _pose_to_frame(self, pose_msg: PoseStamped, target_frame: str) -> PoseStamped:
        if target_frame == WORLD_FRAME:
            return pose_msg
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, pose_msg.header.frame_id,
                                                        rclpy.time.Time(), timeout=Duration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
            raise RuntimeError(f"TF lookup failed: {exc}")
        return do_transform_pose(pose_msg, transform)

    def handle_teleport(self, request: TeleportTool.Request, response: TeleportTool.Response):
        if self.ik_solver is None:
            response.success = False
            response.message = 'IK solver not initialized'
            return response
        q_seed = self._current_joint_array()
        if q_seed is None:
            response.success = False
            response.message = 'No JointState received yet'
            return response
        base_frame = request.base_frame or self.base_frame
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose = request.pose
        if base_frame != self.base_frame:
            pose_stamped = self._pose_to_frame(pose_stamped, self.base_frame)
        frame = self._pose_to_kdl_frame(pose_stamped.pose)
        q_result = kdl.JntArray(self.ik_solver.getNrOfJoints())
        ret = self.ik_solver.CartToJnt(q_seed, frame, q_result)
        if ret < 0:
            response.success = False
            response.message = f'IK solve failed with error code {ret}'
            return response
        if not self._publish_joint_trajectory(q_result):
            response.success = False
            response.message = 'Failed to publish joint trajectory'
            return response

        if not self._wait_for_convergence(q_result):
            response.success = False
            response.message = 'Timeout waiting for joints to reach target'
            return response

        response.success = True
        response.message = 'Joint trajectory executed to target pose'
        return response

    def _pose_to_kdl_frame(self, pose) -> kdl.Frame:
        rot = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w)
        vec = kdl.Vector(pose.position.x, pose.position.y, pose.position.z)
        return kdl.Frame(rot, vec)

    def _publish_joint_trajectory(self, q: kdl.JntArray) -> bool:
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [q[i] for i in range(q.rows())]
        point.velocities = [0.0] * q.rows()
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        traj.points.append(point)
        if not self.traj_pub.get_subscription_count():
            self.get_logger().warn('Trajectory controller may not be listening; publishing anyway')
        self.traj_pub.publish(traj)
        self.get_logger().info('Published joint trajectory command')
        return True

    def _wait_for_convergence(self, q_target: kdl.JntArray) -> bool:
        if self.joint_state is None:
            self.get_logger().warn('No JointState received yet; skipping convergence check')
            return True
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < self.timeout * 1e9:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is None:
                continue
            name_to_pos = dict(zip(self.joint_state.name, self.joint_state.position))
            max_err = 0.0
            for idx, name in enumerate(self.joint_names):
                current = name_to_pos.get(name)
                if current is None:
                    max_err = float('inf')
                    break
                max_err = max(max_err, abs(current - q_target[idx]))
            if max_err <= self.tolerance:
                self.get_logger().info('Joint targets reached')
                return True
        self.get_logger().error('Timeout waiting for joints to reach target')
        return False


def main():
    rclpy.init()
    node = EndEffectorTeleporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
