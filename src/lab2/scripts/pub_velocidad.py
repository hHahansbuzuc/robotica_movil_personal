#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from time import sleep

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        cbg = ReentrantCallbackGroup()

        # Publisher hacia el mux de cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_mux/cmd_vel',
            10,
            callback_group=cbg
        )

        # Subscriber al array [linear, angular]
        self.create_subscription(
            Float64MultiArray,
            'pub_vel',
            self.pubvel_callback,
            10,
            callback_group=cbg
        )

    def pubvel_callback(self, msg: Float64MultiArray):
        linear, angular = msg.data
        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular

        # self.get_logger().info(f'[velocity_publisher] publicando Twist → linear.x={linear:.3f}, angular.z={angular:.3f}')
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
