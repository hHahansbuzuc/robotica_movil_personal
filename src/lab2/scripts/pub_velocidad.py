#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        cbg = ReentrantCallbackGroup()
        self.speed = 0.0
        self.tipo_control = None
        # Publicador de velocidad (Float64) en 'pub_vel'
        self.sub_pid = self.create_subscription(
            Float64MultiArray,
            'pub_vel',
            self.twist_callback,
            10,
            callback_group=cbg
        )

        # Suscripción al Twist de cmd_vel_mux
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_mux/cmd_vel',
            10,
            callback_group=cbg
        )

    def pubvel_callback(self, msg: Twist):
        velocidad = msg.data
        twist = Twist()
        twist.linear.x = velocidad[0]
        twist.angular.z = velocidad[1]


    def twist_callback(self, msg: Float64MultiArray):
        # Obtener la velocidad desde el PID
        self.speed = msg.data[0]
        self.tipo_control = msg.data[1]
        self.get_logger().info(f'[velocity_publisher] velocidad recibida: {self.speed:.3f}')

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
