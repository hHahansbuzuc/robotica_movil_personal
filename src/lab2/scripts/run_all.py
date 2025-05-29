#!/usr/bin/env python3

import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor

from desplazador import Desplazador
from pose_loader import PoseLoader
from pid import PIDController
from pub_velocidad import VelocityPublisher
from data_recorder import DataRecorder
from data_recorder_od import DataRecorderOd

def main():
    # Inicializa ROS 2
    rclpy.init()
    
    # Instancia cada uno de los nodos
    nav_node      = Desplazador()
    pid_node      = PIDController()
    vel_pub_node  = VelocityPublisher()
    loader_node   = PoseLoader(sys.argv[1])  # Ruta al archivo de poses como argumento
    data_recorder = DataRecorder()
    data_recorder_od = DataRecorderOd()  # Nodo para grabar trayectorias reales y odometría
    # Crea un executor multihilo y añade todos los nodos
    executor = MultiThreadedExecutor(num_threads=6)
    for node in ( nav_node, pid_node, vel_pub_node, loader_node, data_recorder, data_recorder_od):
        executor.add_node(node)

    try:
        executor.spin()
    finally:
        # Destruye todos los nodos y cierra ROS 2
        for node in ( nav_node, pid_node, vel_pub_node, loader_node, data_recorder, data_recorder_od):
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
