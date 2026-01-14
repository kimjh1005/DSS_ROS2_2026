#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster


class Map2OdomPublisher(Node):
    def __init__(self):
        super().__init__('map2odom_publisher')

        # Parameters (use_sim_time is automatically declared when passed via yaml)
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')

        # Get use_sim_time - it's already declared by ROS2 when passed via params file
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.default_map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.default_odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value

        # TF broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Track last simulation time from /clock
        self.last_sim_time = None
        if self.use_sim_time:
            self.clock_sub = self.create_subscription(
                Clock,
                '/clock',
                self.clock_callback,
                10)
            self.get_logger().info('Waiting for /clock topic (use_sim_time enabled)')

        # Subscriber for odom2pub
        self.odom_msg = None
        self.subscription = self.create_subscription(
            TransformStamped,
            '/hdl_graph_slam/odom2pub',
            self.callback,
            10)

        # Timer for publishing TF at 10Hz
        self.timer = self.create_timer(0.1, self.spin)

    def clock_callback(self, msg):
        self.last_sim_time = msg.clock

    def callback(self, odom_msg):
        self.odom_msg = odom_msg

    def get_current_time(self):
        """Get current time for TF publishing"""
        if self.use_sim_time:
            # Use cached simulation time
            if self.last_sim_time is not None:
                return self.last_sim_time
            else:
                return None  # No valid sim time yet
        else:
            return self.get_clock().now().to_msg()

    def spin(self):
        if self.odom_msg is None:
            # Get current time
            current_time = self.get_current_time()
            if current_time is None:
                # No valid time yet (waiting for /clock), skip
                return

            # Publish identity transform while waiting for first odom message
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.default_map_frame_id
            t.child_frame_id = self.default_odom_frame_id
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(t)
            return

        # Publish received transform using original message timestamp
        # This ensures TF uses simulation time when use_sim_time is enabled
        t = TransformStamped()
        t.header.stamp = self.odom_msg.header.stamp  # Use original timestamp!
        t.header.frame_id = self.odom_msg.header.frame_id
        t.child_frame_id = self.odom_msg.child_frame_id
        t.transform = self.odom_msg.transform
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Map2OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
