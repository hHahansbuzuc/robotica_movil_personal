#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class CarrotSelector(Node):
    def __init__(self):
        super().__init__('carrot_selector')
        cbg = ReentrantCallbackGroup()


        self.lookahead = 0.3

        # Estado interno
        self.path      = None
        self.curr_x    = None
        self.curr_y    = None

        # Suscripción al Path planificado
        self.create_subscription(
            Path,
            'nav_plan',
            self.on_path,
            10,
            callback_group=cbg
        )
        # Suscripción a la odometría
        self.create_subscription(
            Odometry,
            'odom',
            self.on_odom,
            10,
            callback_group=cbg
        )

        # Publicador del carrot point
        self.carrot_pub = self.create_publisher(
            PoseStamped,
            'carrot_point',
            10,
            callback_group=cbg
        )

    def on_path(self, msg: Path):
        """Guarda la trayectoria recibida."""
        self.path = msg
        self.get_logger().info(f'Path recibido con {len(msg.poses)} puntos')

    def on_odom(self, msg: Odometry):
        """Actualiza la posición actual y publica el carrot."""
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self._select_and_publish_carrot()

    def _select_and_publish_carrot(self):
        """Busca el primer punto a ≥ lookahead y lo publica."""
        if self.path is None or self.curr_x is None:
            return

        la = self.lookahead
        selected = None

        for p in self.path.poses:
            dx = p.pose.position.x - self.curr_x
            dy = p.pose.position.y - self.curr_y
            if math.hypot(dx, dy) >= la:
                selected = p
                break

        # Si ninguno supera la distancia, tomamos el último
        if selected is None and self.path.poses:
            selected = self.path.poses[-1]

        if selected:
            carrot = PoseStamped()
            carrot.header.stamp    = self.get_clock().now().to_msg()
            carrot.header.frame_id = selected.header.frame_id
            carrot.pose            = selected.pose

            self.carrot_pub.publish(carrot)
            self.get_logger().info(
                f'Carrot publicado → x={carrot.pose.position.x:.2f}, '
                f'y={carrot.pose.position.y:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CarrotSelector()

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
