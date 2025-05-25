#!/usr/bin/env python3

import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor

from obstacle_detector import Obstacle_detector
from desplazador import Desplazador
from pose_loader import PoseLoader
from pid import PIDController
from pub_velocidad import VelocityPublisher

def main():
    # Inicializa ROS 2
    rclpy.init()
    
    # Instancia cada uno de los nodos
    obs_node      = Obstacle_detector()
    nav_node      = Desplazador()
    pid_node      = PIDController()
    vel_pub_node  = VelocityPublisher()
    loader_node   = PoseLoader(sys.argv[1])  # Ruta al archivo de poses como argumento

    # Crea un executor multihilo y añade todos los nodos
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (obs_node, nav_node, pid_node, vel_pub_node, loader_node):
        executor.add_node(node)

    try:
        executor.spin()
    finally:
        # Destruye todos los nodos y cierra ROS 2
        for node in (obs_node, nav_node, pid_node, vel_pub_node, loader_node):
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
