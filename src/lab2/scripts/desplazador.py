#!/usr/bin/env python3
import rclpy

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from geometry_msgs.msg import Vector3, Bool
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math, time


class Desplazador(Node):
    def __init__(self):
        super().__init__('desplazador')
         #  creo el callback group
        cbg = ReentrantCallbackGroup()


        self.logger_sub = self.create_subscription(PoseArray, '/goal_list', self.accion_mover_cb, 10, callback_group=cbg)        
        self.sub_occupancy = self.create_subscription(Vector3, '/occupancy_state', self.cb_occupancy, 10, callback_group=cbg)
        self.sub_pid_ready = self.create_subscription(Bool, 'pid_ready', self.cb_pid_done, 10, callback_group=cbg)
        self.pub_desplazador = self.create_publisher(Float64MultiArray, 'objetivo', 10, callback_group=cbg)

        self.pose = Pose()
        self.v_lin = 0.2
        self.v_ang = 1.0
        self.occupancy = Vector3(x=0.0, y=0.0, z=0.0)
        self.ready = False



    def cb_occupancy(self, msg):
        self.get_logger().info(f"CB_OCC ➜ occupancy_state = ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
        self.occupancy = msg
    
    def cb_pid_done(self, msg):
        self.ready = msg.data

    def mover_robot_a_destino(self, goal_pose):
        # El movimiento va a consistir en 3 pasos, primero apuntar, luego dirigirme y finalmente girarme
        # Descomposición
        # (lineal) + (angular) + (lineal)
        x = goal_pose[0]
        y = goal_pose[1]
        theta = goal_pose[2]

        distancia = math.sqrt(x**2 + y**2)
        rotacion = theta
        desplazamientos = [
            (distancia, 0.0),
            (0.0, rotacion),
            (distancia, 0.0)
        ]

        for d in desplazamientos:
            msg = Float64MultiArray()
            msg.data = [d[0], d[1]]  # (lineal, angular)
            self.ready = False
            self.pub_desplazador.publish(msg)

            self.get_logger().info(f"Desplazamiento enviado: {d}")

            while not self.ready:
                time.sleep(0.1)
            self.get_logger().info("Desplazamiento confirmado por PID")

    def accion_mover_cb(self, msg: PoseArray):
        for goal_pose in msg.poses:
            self.get_logger().info(f"Goal → x={goal_pose.position.x}, y={goal_pose.position.y}")
            self.mover_robot_a_destino(goal_pose)

def main():
    rclpy.init()
    node = Desplazador()

    # creo el executor multihilo
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()       # spin del executor, no rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
