#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Odometry
from math import sqrt
from time import sleep
import tf_transformations
from math import atan2, sin, cos

from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.executors import MultiThreadedExecutor

from math import atan2, sin, cos

def angle_diff(curr_yaw, start_yaw):
    # diferencia directa
    diff = curr_yaw - start_yaw
    # normalizada a (–π, π)
    return atan2(sin(diff), cos(diff))


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        cbg = ReentrantCallbackGroup()

        # Para movimiento relativo
        self.start_x = 0.0
        self.start_y = 0.0
        self.curr_x  = 0.0
        self.curr_y  = 0.0
        self.start_yaw = 0.0
        self.curr_yaw  = 0.0
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

        self._has_arrived = False

        # Suscriptores
        self.create_subscription(Float64MultiArray, 'objetivo', self.callback_setpoint, 10,callback_group=cbg)
        self.create_subscription(Odometry, '/odom', self.callback_state, 10, callback_group=cbg)
        # Publicador
        self.pub_vel = self.create_publisher(Float64MultiArray, 'pub_vel', 10, callback_group=cbg)
        self.pub_pid_ready = self.create_publisher(Bool, 'pid_ready', 10, callback_group=cbg)

        # Es nuestro loop de control, se ejecuta cada dt segundos
        self.create_timer(self.dt, self.control_loop, callback_group=cbg)

    def callback_setpoint(self, msg):

        lineal, angular = msg.data
        self.e_k1        = 0.0
        self.u_k1        = 0.0
        self.e_k2        = 0.0

        self._has_arrived = False

        if lineal != 0.0:
            self.get_logger().info("Recivi moviemiento lineal")
            self.r_t = lineal
            self.tipo_control = "lineal"
            self.Kp, self.Ki, self.Kd = 0.45, 0.05, 0.0
            self.start_x = self.curr_x
            self.start_y = self.curr_y

        elif angular != 0.0:
            self.get_logger().info("Recivi moviemiento angular")
            self.r_t = angular
            self.tipo_control = "angular"
            self.Kp, self.Ki, self.Kd = 0.35, 0.03, 0.0
            self.start_yaw = self.curr_yaw
        else:
            self.tipo_control = None  # ningún desplazamiento
            self.pub_pid_ready.publish(Bool(data=True))
            self.get_logger().info(">>> Movimiento PID terminado")
            self.tipo_control = None
            self._has_arrived = True


    def callback_state(self, msg: Odometry):
        # actualiza la posición actual
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y


        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.curr_yaw = yaw

        if self.tipo_control == 'angular':
            # para giro, y_t es yaw
            self.y_t = angle_diff(self.curr_yaw, self.start_yaw)

        elif self.tipo_control == 'lineal':
            # para lineal, y_t es la distancia recorrida
            dx = self.curr_x - self.start_x
            dy = self.curr_y - self.start_y
            self.y_t = sqrt(dx**2 + dy**2)



    def control_loop(self):

        if self.tipo_control is None:
            return  # No hay nada que hacer

        # Error actual
        
        self.e_k = self.r_t - self.y_t
        self.get_logger().info(f">>> ek {self.e_k:.9f}")
        # Esta es una manera para que yo pueda subdividir el PID que quedaría gigante si simpleemente pusiera todo junto
        a0 = self.Kp + self.Ki * self.dt + self.Kd / self.dt
        a1 = -self.Kp - 2 * (self.Kd / self.dt)
        a2 = self.Kd / self.dt



        max_u = 0.5
        min_u = -0.5
        
        # PID
        u_k = self.u_k1 + a0 * self.e_k + a1 * self.e_k1 + a2 * self.e_k2
        u_k = max(min(u_k, max_u), min_u)

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

        # Verificar si hemos llegado al objetivo
        arrived = abs(self.e_k) < 0.01
        if arrived and not self._has_arrived:
            self.pub_pid_ready.publish(Bool(data=True))
            self.get_logger().info(">>> Movimiento PID terminado")
            self.tipo_control = None
            self._has_arrived = True
        elif not arrived:
            self._has_arrived = False


def main(args=None):
    rclpy.init(args=args)
    pid = PIDController()
    rclpy.spin(pid)
    pid.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
