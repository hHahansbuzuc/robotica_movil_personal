#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class PidCarrot(Node):
    def __init__(self):
        super().__init__('pid_carrot')
        cbg = ReentrantCallbackGroup()

        # —— Parámetros PID fijos —— 
        self.Kp = 0.5       # ganancia proporcional
        self.Ki = 0.0       # ganancia integral
        self.Kd = 0.0       # ganancia derivativa

        # —— Velocidad y saturación fija —— 
        self.lin_speed = 0.1   # m/s
        self.max_ang   = 1.0   # rad/s

        # —— Historial para PID incremental —— 
        self.e_k2 = 0.0
        self.e_k1   = 0.0
        self.u_k1   = 0.0       # último control angular

        # —— Intervalo de control —— 
        self.dt = 0.05   # 20 Hz

        # —— Último error recibido —— 
        self.error = 0.0

        # 1) Suscripción al error angular (desde ErrorCalculator)
        self.create_subscription(
            Float64,
            'carrot_error',
            self._error_cb,
            10,
            callback_group=cbg
        )

        # 2) Publicador de velocidad [v_lin, v_ang]
        self.pub_vel = self.create_publisher(
            Float64MultiArray,
            'pub_vel',
            10,
            callback_group=cbg
        )

        # 3) Timer para cálculo PID periódico
        self.create_timer(self.dt, self._control_loop, callback_group=cbg)

    def _error_cb(self, msg: Float64):
        # Actualiza el error antes de cada ciclo de control
        self.error = msg.data

    def _control_loop(self):
        # Calcula PID
        e_k = self.error
        # Cálculo incremental PID
        a0 = self.Kp + self.Ki * self.dt + self.Kd / self.dt
        a1 = -self.Kp - 2 * (self.Kd / self.dt)
        a2 = self.Kd / self.dt
        u_ang = self.u_k1 + a0 * e_k + a1 * self.e_k1 + a2 * self.e_k2

        # Desplazar historial
        self.e_k2 = self.e_k1
        self.e_k1 = e_k
        self.u_k1 = u_ang

        # Saturación angular
        if u_ang >  self.max_ang: u_ang =  self.max_ang
        if u_ang < -self.max_ang: u_ang = -self.max_ang

        # Publicar [v_lin, v_ang]
        msg = Float64MultiArray(data=[self.lin_speed, u_ang])
        self.pub_vel.publish(msg)

        # Log para depuración
        self.get_logger().info(f"[PidCarrot] e={e_k:.3f}  u_ang={u_ang:.3f}")


def main():
    rclpy.init()
    node = PidCarrot()

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
