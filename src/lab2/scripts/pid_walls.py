#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from rclpy.executors import MultiThreadedExecutor

class PidWalls(Node):
    def __init__(self):
        super().__init__('wall_balancer_pid')
        cbg = ReentrantCallbackGroup()

        # Parámetros PID (P puro)
        self.Kp = 0.2
        self.Ki = 0.0
        self.Kd = 0.0

        # Historial para forma incremental
        self.e_k1 = 0.0
        self.e_k2 = 0.0
        self.u_k1 = 0.0

        # Intervalo de muestreo (coincide con el timer)
        self.dt = 0.05

        # Márgen de error: si la diferencia está por debajo, no giramos
        self.epsilon = 0.001  # [m]

        # Últimos valores recibidos
        self.dist_left  = None
        self.dist_right = None

        # Suscripción a las distancias laterales
        self.create_subscription(
            Vector3,
            '/side_distances',
            self.cb_distances,
            10,
            callback_group=cbg
        )

        # Publicador de velocidades [v_lin, v_ang]
        self.pub_vel = self.create_publisher(
            Float64MultiArray,
            'pub_vel',
            10,
            callback_group=cbg
        )

        # Timer de control a 20 Hz
        self.create_timer(self.dt, self.control_loop, callback_group=cbg)

    def cb_distances(self, msg: Vector3):
        # msg.x = dist_left, msg.y = dist_right
        self.dist_left  = msg.x
        self.dist_right = msg.y

    def control_loop(self):
        # Esperamos a tener ambas distancias
        if self.dist_left is None or self.dist_right is None:
            return

        # Error lateral: + si right > left (estamos más cerca pared izq)
        e_k = self.dist_right - self.dist_left

        # Cálculo incremental PID
        a0 = self.Kp + self.Ki * self.dt + self.Kd / self.dt
        a1 = -self.Kp - 2 * (self.Kd / self.dt)
        a2 = self.Kd / self.dt
        u_k = self.u_k1 + a0 * e_k + a1 * self.e_k1 + a2 * self.e_k2

        # Desplazar historial
        self.e_k2 = self.e_k1
        self.e_k1 = e_k
        self.u_k1 = u_k

        # Usa solo el P para la corrección angular
        u_ang = u_k

        # Aplica margen de error: si la diferencia es pequeña, no girar
        if abs(e_k) < self.epsilon:
            u_ang = 0.0

        # Saturación angular
        max_ang = 1.0  # rad/s
        if u_ang >  max_ang: u_ang =  max_ang
        if u_ang < -max_ang: u_ang = -max_ang

        # Publica velocidad lineal fija 0.2 m/s y corrección angular
        msg = Float64MultiArray(data=[0.2, u_ang])
        self.pub_vel.publish(msg)

        # Log informativo
        self.get_logger().info(
            f"[PID] err={e_k:.3f}  u_ang={u_ang:.3f}  "
            f"left={self.dist_left:.2f} right={self.dist_right:.2f}"
        )

def main():
    rclpy.init()
    node = PidWalls()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
