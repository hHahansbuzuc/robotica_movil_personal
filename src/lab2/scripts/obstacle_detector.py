#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import numpy as np
from cv_bridge import CvBridge

class Obstacle_detector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()
        self.occ = Vector3()

        # Usamos un callback group reentrante
        cbg = ReentrantCallbackGroup()
        # Definimos un QoSProfile para depth
        depth_qos = QoSProfile(depth=10)

        # Suscripción al tópico de imagen de profundidad
        self.image_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            depth_qos,
            callback_group=cbg
        )

        # Publicador del estado de ocupación
        self.occupancy_pub = self.create_publisher(
            Vector3,
            '/occupancy_state',
            10,
            callback_group=cbg
        )

        # Timer para publicar occupancy cada 0.05s
        self.create_timer(
            0.05,
            self.publicar_occupancy,
            callback_group=cbg
        )

    def image_callback(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        arr = np.array(depth, copy=False).astype(np.float32)
        h, w = arr.shape
        third = w // 3
        left   = arr[:, :third]
        center = arr[:, third:2*third]
        right  = arr[:, 2*third:]

        for region in (left, center, right):
            region[region == 0] = np.nan

        dl = float(np.nanmin(left))
        dc = float(np.nanmin(center))
        dr = float(np.nanmin(right))

        TH = 0.5
        self.occ.x = 1.0 if (0.0 < dl <= TH) else 0.0
        self.occ.y = 1.0 if (0.0 < dc <= TH) else 0.0
        self.occ.z = 1.0 if (0.0 < dr <= TH) else 0.0

        self.get_logger().info(
            f"Occupancy → Izq:{self.occ.x:.0f} Cent:{self.occ.y:.0f} Der:{self.occ.z:.0f}"
        )

    def publicar_occupancy(self):
        self.occupancy_pub.publish(self.occ)

def main():
    rclpy.init()
    node = Obstacle_detector()

    # Usamos MultiThreadedExecutor para permitir concurrencia interna
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        # parar executor y nodo
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()