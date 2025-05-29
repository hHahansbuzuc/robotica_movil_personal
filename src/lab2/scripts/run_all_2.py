#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

# Importa tus nodos
from wall_detector       import WallDetector
from pid_walls           import PidWalls
from pub_velocidad       import VelocityPublisher

def main():
    # 1) Inicializa rclpy
    rclpy.init()

    # 2) Crea el executor con al menos tantos hilos como nodos
    executor = MultiThreadedExecutor(num_threads=3)

    # 3) Instancia tus nodos
    nodes = [
        WallDetector(),
        PidWalls(),
        VelocityPublisher(),
    ]
    for node in nodes:
        executor.add_node(node)

    # 4) Spin hasta Ctrl–C o hasta que ROS2 decida cerrarnos
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Si alguien pulsa Ctrl–C localmente, caemos aquí
        pass
    finally:
        # 5) Shutdown limpio: primero detenemos el executor…
        executor.shutdown()
        # …luego destruimos cada nodo…
        for node in nodes:
            try:
                node.destroy_node()
            except Exception as e:
                node.get_logger().warn(f"Error al destruir nodo: {e}")
        # …y por último cerramos rclpy
        rclpy.shutdown()

if __name__ == '__main__':
    main()
