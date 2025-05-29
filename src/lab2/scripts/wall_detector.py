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
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        arr = np.array(depth, copy=False).astype(np.float32)
        h, w = arr.shape

        # 1) Define una franja centrada verticalmente (±5% de la altura)
        mid    = h // 2
        margin = int(h * 0.05)
        stripe = arr[mid-margin:mid+margin, :]

        # 2) Divide en izquierda/derecha
        third = w // 3
        left  = stripe[:, :third].flatten()
        right = stripe[:, 2*third:].flatten()

        # 3) Filtra ceros y NaNs
        left  = left[(left > 0) & ~np.isnan(left)]
        right = right[(right > 0) & ~np.isnan(right)]

        # 4) Toma un percentil bajo (p.ej. 5%) o la mediana
        if left.size:
            dl = np.percentile(left, 5)    # o np.median(left)
        else:
            dl = float('inf')
        if right.size:
            dr = np.percentile(right, 5)   # o np.median(right)
        else:
            dr = float('inf')

        # 5) Publica dist_left/dr y su diferencia
        self.dists.x = dl
        self.dists.y = dr
        self.dists.z = abs(dr - dl)

        self.get_logger().debug(
            f"[ROI] dl={dl:.2f}m  dr={dr:.2f}m  diff={self.dists.z:.2f}m"
        )

    def publish_distances(self):
        # Publica el Vector3 con (dist_left, dist_right, diff)
        self.dist_pub.publish(self.dists)

def main():
    rclpy.init()
    node = WallDetector()

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
