#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

class PoseLoader(Node):
    def __init__(self, filepath):
        super().__init__('pose_loader')
        self.pub = self.create_publisher(PoseArray, '/goal_list', 10)
        self.filepath = filepath
        self.published = False
        self.create_timer(1.0, self.publish_poses)

    def publish_poses(self):
        if self.published:
            return
        pa = PoseArray()
        with open(self.filepath, 'r') as f:
            for line in f:
                x, y, theta = map(float, line.split())
                p = Pose()
                p.position.x = x
                p.position.y = y
                p.orientation.z = theta
                pa.poses.append(p)
        self.pub.publish(pa)
        self.get_logger().info(f"Publicado {len(pa.poses)} poses en /goal_list")
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    args = rclpy.utilities.remove_ros_args()[1:]
    node = PoseLoader(args[0])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
