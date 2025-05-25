#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Odometry
from math import sqrt
from numpy import arctan2

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        cbg = ReentrantCallbackGroup()
    	# Se inicializan las variables y coeficientes
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        # Estado del sistema se modificara cada vuelta nueva
        self.r_t = 0.0
        self.y_t = 0.0

        # Errores históricos, se modificaran cada vuelta nueva
        self.e_k = 0.0
        self.e_k1 = 0.0
        self.e_k2 = 0.0

        # Control anterior
        self.u_k1 = 0.0

        # Paso de tiempo
        self.dt = 0.01  # lo fijo a 100 Hz por conveniencia

        # Esto sirve para determinar el tipo de PID que se va a usar
        self.tipo_control = None

        # Suscriptores
        self.create_subscription(Float64MultiArray, 'objetivo', self.callback_setpoint, 10,callback_group=cbg)
        self.create_subscription(Odometry, '/odom', self.callback_state, 10, callback_group=cbg)
        # Publicador
        self.pub_vel = self.create_publisher(Float64MultiArray, 'pub_vel', 10, callback_group=cbg)
        self.pub_pid_ready = self.create_publisher(Bool, 'pid_ready', 10, callback_group=cbg)

        # Es hasta cierto punto nuestra resolucion
        self.create_timer(self.dt, self.control_loop, callback_group=cbg)

    def callback_setpoint(self, msg):
        lineal, angular = msg.data

        if lineal != 0.0:
            self.get_logger().info("Recivi moviemiento lineal")
            self.r_t = lineal
            self.tipo_control = "lineal"
            self.Kp, self.Ki, self.Kd = 0.1, 0.0, 0.0
        elif angular != 0.0:
            self.r_t = angular
            self.tipo_control = "angular"
            self.Kp, self.Ki, self.Kd = 0.01, 0.0, 0.0
        else:
            self.tipo_control = None  # ningún desplazamiento

    def callback_state(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = arctan2( msg.pose.pose.orientation.z,
                     msg.pose.pose.orientation.x )  # si usas quaternion puro, convierte bien

        if self.tipo_control == 'lineal':
            self.y_t = sqrt(x*x + y*y)
        elif self.tipo_control == 'angular':
            self.y_t = yaw

    def control_loop(self):

        if self.tipo_control is None:
            return  # No hay nada que hacer

        # Error actual
        self.e_k = self.r_t - self.y_t

        # Esta es una manera para que yo pueda subdividir el PID que quedaría gigante si simpleemente pusiera todo junto
        a0 = self.Kp + self.Ki * self.dt + self.Kd / self.dt
        a1 = -self.Kp - 2 * (self.Kd / self.dt)
        a2 = self.Kd / self.dt

        # PID
        u_k = self.u_k1 + a0 * self.e_k + a1 * self.e_k1 + a2 * self.e_k2

        # Publicar
        if self.tipo_control == "lineal":
            self.pub_vel.publish(Float64MultiArray(data=[u_k, 0.0]))
        elif self.tipo_control == "angular":
            self.pub_vel.publish(Float64MultiArray(data=[0.0, u_k]))
        else:
            self.pub_vel.publish(Float64MultiArray(data=[0.0, 0.0]))

        # Desplazar historial, error anterior es igual al error actual
        self.e_k2 = self.e_k1
        self.e_k1 = self.e_k
        self.u_k1 = u_k

        # salimos del pid si se alcanza el objetivo del momento
        if abs(self.e_k) < 0.1:
            self.pub_pid_ready.publish(Bool(data=True))
            self.get_logger().info(">>> Movimiento PID terminado")



def main(args=None):
    rclpy.init(args=args)
    pid = PIDController()
    rclpy.spin(pid)
    pid.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
