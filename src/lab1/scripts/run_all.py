#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from obstacle_detector import Obstacle_detector
from dead_reckoning_nav import DeadReckoningNav
from pose_loader import PoseLoader
import sys

def main():
    rclpy.init()
    # Instanciamos los nodos
    obs_node   = Obstacle_detector()
    nav_node   = DeadReckoningNav()
    # PoseLoader necesita la ruta al archivo de poses
    # Se pasa como argumento en el launch: args=".../poses.txt"
    loader_node = PoseLoader(sys.argv[1])

    # Creamos el executor multihilo
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(obs_node)
    executor.add_node(nav_node)
    executor.add_node(loader_node)

    try:
        executor.spin()
    finally:
        obs_node.destroy_node()
        nav_node.destroy_node()
        loader_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()