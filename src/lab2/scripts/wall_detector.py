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

class WallDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()
        # vector donde guardamos dist_left, dist_right, diff
        self.dists = Vector3()

        # callback group para permitir concurrencia
        cbg = ReentrantCallbackGroup()
        # QoS adecuado para depth images
        depth_qos = QoSProfile(depth=10)

        # 1) Subscribir a la cámara de profundidad
        self.image_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            depth_qos,
            callback_group=cbg
        )

        # 2) Publicador de distancias
        self.dist_pub = self.create_publisher(
            Vector3,
            '/side_distances',
            10,
            callback_group=cbg
        )

        # 3) Timer que publica al ritmo deseado (20 Hz)
        self.create_timer(
            0.05,
            self.publish_distances,
            callback_group=cbg
        )

    def image_callback(self, msg: Image):
        # Convertir ROS Image a array NumPy
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        arr = np.array(depth, copy=False).astype(np.float32)
        h, w = arr.shape

        # Dividimos la imagen en tres regiones verticales
        third    = w // 3
        left     = arr[:, :third]
        center   = arr[:, third:2*third]
        right    = arr[:, 2*third:]

        # Convertir ceros (sin dato) a NaN para ignorarlos
        for region in (left, center, right):
            region[region == 0] = np.nan

        # Extraer la mínima distancia válida en cada región
        dl = float(np.nanmin(left))   if not np.all(np.isnan(left))   else float('inf')
        dc = float(np.nanmin(center)) if not np.all(np.isnan(center)) else float('inf')
        dr = float(np.nanmin(right))  if not np.all(np.isnan(right))  else float('inf')

        # Guardamos solo izq (x) y der (y)
        self.dists.x = dl
        self.dists.y = dr
        # y como z la diferencia absoluta
        self.dists.z = abs(dr - dl)

        # Si quieres loguear:
        self.get_logger().debug(
            f"dl={dl:.2f}m  dr={dr:.2f}m  diff={self.dists.z:.2f}m"
        )

    def publish_distances(self):
        # Publica el Vector3 con (dist_left, dist_right, diff)
        self.dist_pub.publish(self.dists)

def main():
    rclpy.init()
    node = ObstacleDetector()

    # Executor multihilo para callbacks concurrentes
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
