#!/usr/bin/env python3
import rclpy

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import math, time
import csv
import os

class DeadReckoningNav(Node):
    def __init__(self):
        super().__init__('dead_reckoning_nav')
        cbg = ReentrantCallbackGroup()
        
        self.pub = self.create_publisher(Twist, '/cmd_vel_mux/cmd_vel', 10, callback_group=cbg)
        self.logger_sub = self.create_subscription(PoseArray, '/goal_list', self.accion_mover_cb, 10, callback_group=cbg)        
        self.sub_occupancy = self.create_subscription(Vector3, '/occupancy_state', self.cb_occupancy, 10, callback_group=cbg)

        self.pose = Pose()
        self.v_lin = 0.2
        self.v_ang = 1.0
        self.occupancy = Vector3(x=0.0, y=0.0, z=0.0)
        #  creo el callback group
        


        
    def cb_occupancy(self, msg):
        self.get_logger().info(f"CB_OCC ➜ occupancy_state = ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
        self.occupancy = msg


    def aplicar_velocidad(self, cmds):
        
        stop =    self.occupancy.x == 1 or self.occupancy.y == 1 or self.occupancy.z == 1


        for v, w, t in cmds:
            twist = Twist()
            twist.linear.x  = float(v)
            twist.angular.z = float(w)
            # usamos time.time() puro
            start = time.time()
            end   = start + t
            while time.time() < end:
                blocked_start = 0
                if stop:
                    blocked_start = time.time() # Detengo el tiempo
                    self.pub.publish(Twist())  # Me detengo si hay un obstáculo
                    rclpy.spin_once(self, timeout_sec=0.02)
                else:
                    self.pub.publish(twist)
                    rclpy.spin_once(self, timeout_sec=0.02)
                    time.sleep(0.02)
                # Actualizo el tiempo ajustado por la perdida
            blocked_time = time.time() - blocked_start
            end += blocked_time
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        # al terminar todos los cmds, se detiene el robot
        self.pub.publish(Twist())
        
    def normalize_angle(self, a): #normalizo el angulo entre -pi y pi para no rotar infinitamente
        return (a + math.pi) % (2*math.pi) - math.pi

    def mover_robot_a_destino(self, goal_pose):
        # El movimiento va a consistir en 3 pasos, primero apuntar, luego dirigirme y finalmente girarme
        x_obj, y_obj , theta_obj = goal_pose.position.x, goal_pose.position.y, goal_pose.orientation.z
        x, y, theta = self.pose.position.x, self.pose.position.y, self.pose.orientation.z

        # Mediante calculo geometrico obtengo el tiempo que me tarda llegar al x,y objetivo.
        dx = x_obj - x
        dy = y_obj - y
        d = math.sqrt(dx**2 + dy**2)
        # Ya que la trayectoria es recta y en cuadrados yo se que me debo girar siempre 90 grados a la izquierda al final de cada trayectoría
        
        # Calculo el tiempo que me tarda llegar al objetivo
        t_mover = d / self.v_lin
        # Me muevo ese tiempo para llegar al objetivo
        cmds = [(self.v_lin, 0, t_mover)]
        self.aplicar_velocidad(cmds)
        
        # Finalmente calculo el timepo para quedar en el angulo objetivo
        t_girar = self.normalize_angle(theta_obj-theta) / self.v_ang
        t_girar = t_girar*1.115 # 1.1 para compensar el tiempo que me tarda girar
        # Me muevo ese tiempo para quedar en el angulo objetivo
        cmds = [(0, self.v_ang, t_girar)]
        self.aplicar_velocidad(cmds)
        
        # Actualizo la pose del robot
        self.pose.position.x = x_obj
        self.pose.position.y = y_obj
        self.pose.orientation.z = theta_obj

    def accion_mover_cb(self, msg: PoseArray):
        for goal_pose in msg.poses:
            self.get_logger().info(f"Goal → x={goal_pose.position.x}, y={goal_pose.position.y}")
            self.mover_robot_a_destino(goal_pose)

def main():
    rclpy.init()
    node = DeadReckoningNav()

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
