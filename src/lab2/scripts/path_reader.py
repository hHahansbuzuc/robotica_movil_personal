#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point

# esto te da la ruta /…/install/lab2/share/lab2
from ament_index_python.packages import get_package_share_directory

class PathReader(Node):
    def __init__(self):
        super().__init__('path_reader')
        cbg = ReentrantCallbackGroup()

        # 1) Default absoluto a config/path_line.txt
        share_dir = get_package_share_directory('lab2')
        default_path = os.path.join(share_dir, 'config', 'path_sqrt.txt')

        # 2) Declaro parámetro con valor por defecto
        self.declare_parameter('file_path', default_path)
        self.declare_parameter('frame_id',  '/odom')

        file_path = self.get_parameter('file_path').get_parameter_value().string_value
        frame_id  = self.get_parameter('frame_id').get_parameter_value().string_value

        # 3) Publisher
        self.path_pub = self.create_publisher(Path, 'nav_plan', 10, callback_group=cbg)

        # 4) Cargo el path
        self._path_msg = self._load_path(file_path, frame_id)

        # 5) Publico UNA sola vez
        self._timer = self.create_timer(1.0, self._publish_once, callback_group=cbg)

    def _load_path(self, file_path: str, frame_id: str) -> Path:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp    = self.get_clock().now().to_msg()

        if not os.path.isfile(file_path):
            self.get_logger().error(f'PathReader: no existe "{file_path}"')
            return path

        with open(file_path, 'r') as f:
            for idx, line in enumerate(f):
                txt = line.strip()
                if not txt or txt.startswith('#'):
                    continue
                parts = txt.replace(',', ' ').split()
                if len(parts) < 2:
                    continue
                try:
                    x, y = float(parts[0]), float(parts[1])
                except ValueError:
                    self.get_logger().warn(f'Línea inválida ({idx}): "{txt}"')
                    continue

                ps = PoseStamped()
                ps.header.frame_id = frame_id
                ps.header.stamp    = path.header.stamp
                ps.pose            = Pose(position=Point(x=x, y=y, z=0.0))
                path.poses.append(ps)

        self.get_logger().info(f'PathReader: cargados {len(path.poses)} puntos')
        return path

    def _publish_once(self):
        self.path_pub.publish(self._path_msg)
        self.get_logger().info(f'PathReader: publicado nav_plan ({len(self._path_msg.poses)} puntos)')
        self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PathReader()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
