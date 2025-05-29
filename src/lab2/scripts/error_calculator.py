#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import atan2, sin, cos

def angle_diff(a, b):
    """Devuelve la diferencia entre ángulos a y b normalizada a [-π, π]."""
    diff = a - b
    return atan2(sin(diff), cos(diff))

class ErrorCalculator(Node):
    def __init__(self):
        super().__init__('error_calculator')
        cbg = ReentrantCallbackGroup()

        # Estado interno
        self.carrot_x  = None
        self.carrot_y  = None
        self.curr_x    = None
        self.curr_y    = None
        self.curr_yaw  = None

        # 1) Suscribirse al punto carrot dinámico
        self.create_subscription(
            PoseStamped,
            'carrot_point',
            self.cb_carrot,
            10,
            callback_group=cbg
        )

        # 2) Suscribirse a la odometría
        self.create_subscription(
            Odometry,
            'odom',
            self.cb_odom,
            10,
            callback_group=cbg
        )

        # 3) Publicador del error angular
        self.error_pub = self.create_publisher(
            Float64,
            'carrot_error',
            10,
            callback_group=cbg
        )

    def cb_carrot(self, msg: PoseStamped):
        """Guarda la posición del carrot."""
        self.carrot_x = msg.pose.position.x
        self.carrot_y = msg.pose.position.y
        self.get_logger().debug(f'Carrot actualizado → x={self.carrot_x:.2f}, y={self.carrot_y:.2f}')

    def cb_odom(self, msg: Odometry):
        """Actualiza la pose del robot, calcula y publica el error angular."""
        # 1) Extraer posición
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y

        # 2) Extraer yaw del quaternion
        q = msg.pose.pose.orientation
        self.curr_yaw = atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )

        # 3) Si no hay carrot, salimos
        if self.carrot_x is None or self.curr_yaw is None:
            return

        # 4) Calculamos el ángulo deseado al carrot
        dx = self.carrot_x - self.curr_x
        dy = self.carrot_y - self.curr_y
        desired_yaw = atan2(dy, dx)

        # 5) Error angular normalizado
        error = angle_diff(desired_yaw, self.curr_yaw)

        # 6) Publicamos
        msg_out = Float64()
        msg_out.data = error
        self.error_pub.publish(msg_out)

        # 7) Log informativo
        self.get_logger().info(f'[ErrorCalc] error={error:.3f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = ErrorCalculator()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
