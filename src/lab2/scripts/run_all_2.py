#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

# Importa tus nodos
from wall_detector import WallDetector
from pid_walls import PidWalls
from pub_velocidad import VelocityPublisher

def main():
    rclpy.init()

    # Instanciamos los nodos
    obs_node = WallDetector()
    pid_node = PidWalls()
    vel_pub_node  = VelocityPublisher()

    # Executor multihilo para que ambos callbacks puedan correr concurrentemente
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(obs_node)
    executor.add_node(pid_node)
    executor.add_node(vel_pub_node)

    try:
        executor.spin()
    finally:
        # Shutdown limpio
        executor.shutdown()
        obs_node.destroy_node()
        pid_node.destroy_node()
        vel_pub_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()