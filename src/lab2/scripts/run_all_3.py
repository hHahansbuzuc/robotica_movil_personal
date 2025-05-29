#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from path_reader      import PathReader
from carrot_selector  import CarrotSelector
from error_calculator import ErrorCalculator
from pid_carrot       import PidCarrot
from pub_velocidad import VelocityPublisher
from path_plotter import PathPlotter

def main():
    rclpy.init()

    # instancio todos los nodos:
    path_node   = PathReader()
    carrot_node = CarrotSelector()
    error_node  = ErrorCalculator()
    pid_node    = PidCarrot()
    vel_node    = VelocityPublisher()
    
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (path_node, carrot_node, error_node, pid_node, vel_node):
        executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        for node in (path_node, carrot_node, error_node, pid_node, vel_node):
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
