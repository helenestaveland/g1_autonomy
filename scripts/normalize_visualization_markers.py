#!/usr/bin/env python3
"""Repair invalid visualization poses from GBPlanner3 main for RViz2."""

from functools import partial

import rclpy
from geometry_msgs.msg import PoseArray, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray


MARKER_TOPICS = (
    '/vis/planning_workspace',
    '/vis/no_gain_zone',
    '/vis/planning_graph',
    '/vis/planning_projected_graph',
    '/vis/planning_failed',
    '/vis/shortest_paths',
    '/vis/robot_state',
    '/vis/sensor_fov',
    '/vis/best_planning_paths',
    '/vis/volumetric_gains',
    '/vis/sampler',
    '/vis/ray_casting',
    '/vis/ref_path',
    '/vis/imp_ref_path',
    '/vis/planning_global_graph',
    '/vis/planning_homing_path',
    '/vis/planning_global_path',
    '/vis/shortest_path_clustering',
    '/vis/state_history',
    '/vis/geofence',
    '/vis/negative_edges',
    '/vis/hyperplanes',
    '/vis/mod_path',
    '/vis/blind_mod_path',
    '/vis/alternate_path',
    '/vis/opening_traversal_path',
    '/vis/graph_vertices',
)
POSE_TOPICS = ('/vis/viewpoints',)


def normalize_zero_orientation(orientation: Quaternion) -> None:
    """Give a marker or pose with an uninitialized quaternion identity rotation."""
    if orientation.x == orientation.y == orientation.z == orientation.w == 0.0:
        orientation.w = 1.0


class MarkerPoseRelay(Node):
    """Forward planner visualization topics with valid pose orientations."""

    def __init__(self) -> None:
        super().__init__('gbplanner_marker_pose_relay')
        qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._subscriptions = []
        self._publishers = []
        for message_type, topics, forward in (
            (MarkerArray, MARKER_TOPICS, self._forward_markers),
            (PoseArray, POSE_TOPICS, self._forward_poses),
        ):
            for source in topics:
                publisher = self.create_publisher(
                    message_type, f'{source}_rviz', qos
                )
                subscription = self.create_subscription(
                    message_type, source, partial(forward, publisher), qos
                )
                self._publishers.append(publisher)
                self._subscriptions.append(subscription)

    def _forward_markers(self, publisher, marker_array: MarkerArray) -> None:
        for marker in marker_array.markers:
            normalize_zero_orientation(marker.pose.orientation)
        publisher.publish(marker_array)

    def _forward_poses(self, publisher, pose_array: PoseArray) -> None:
        for pose in pose_array.poses:
            normalize_zero_orientation(pose.orientation)
        publisher.publish(pose_array)


def main() -> None:
    rclpy.init()
    node = MarkerPoseRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
