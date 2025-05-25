#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import csv
import os
import atexit

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
        self.real_data = []
        self.odom_data = []
        self.create_subscription(Pose,     '/real_pose', self.cb_real, 10)
        self.create_subscription(Odometry, '/odom',      self.cb_odom, 10)
        # Aseguramos guardar al morir el nodo
        atexit.register(self.save_csvs)

    def cb_real(self, msg: Pose):
        self.real_data.append((msg.position.x, msg.position.y))

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_data.append((p.x, p.y))

    def save_csvs(self):
        wd = os.path.expanduser('~/ros2_ws')
        with open(wd + '/real_tray.csv', 'w', newline='') as f:
            csv.writer(f).writerows(self.real_data)
        with open(wd + '/odom_tray.csv', 'w', newline='') as f:
            csv.writer(f).writerows(self.odom_data)
        self.get_logger().info(f"Guardados {len(self.real_data)} reales y "
                               f"{len(self.odom_data)} odom en CSV")

def main():
    rclpy.init()
    node = DataRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()