#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg      import Float64MultiArray
from nav_msgs.msg      import Odometry
from math              import sqrt, atan2
from time              import time
import csv, os, atexit

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
        # ——— Listas para acumular datos ——————————
        # Cada entrada será una tupla (t, valor1, valor2)
        self.ref_data  = []  # referencia: (t, ref_lin, ref_ang)
        self.act_data  = []  # actuación: (t, act_lin, act_ang)
        self.real_data = []  # salida:   (t, dist, yaw)

        # ——— Suscripciones a tópicos ——————————
        # P_ref(t) viene en 'objetivo' como Float64MultiArray [lin, ang]
        self.create_subscription(
            Float64MultiArray,
            'objetivo',
            self.cb_ref,
            10
        )
        # V_s(t) viene en 'pub_vel' como Float64MultiArray [u_lin, u_ang]
        self.create_subscription(
            Float64MultiArray,
            'pub_vel',
            self.cb_act,
            10
        )
        # P_real(t) lo extraemos de '/odom'
        self.create_subscription(
            Odometry,
            '/odom',
            self.cb_odom,
            10
        )

        # Al cerrarse el proceso (Ctrl-C), llamar a save_csvs()
        atexit.register(self.save_csvs)

    def _now(self):
        """Devuelve un timestamp en segundos (float)."""
        return time()

    def cb_ref(self, msg: Float64MultiArray):
        """Callback para la señal de referencia P_ref(t)."""
        t   = self._now()
        lin = msg.data[0]  # consigna lineal
        ang = msg.data[1]  # consigna angular
        self.ref_data.append((t, lin, ang))

    def cb_act(self, msg: Float64MultiArray):
        """Callback para la señal de actuación V_s(t)."""
        t   = self._now()
        lin = msg.data[0]  # acción lineal
        ang = msg.data[1]  # acción angular
        self.act_data.append((t, lin, ang))

    def cb_odom(self, msg: Odometry):
        """Callback para la señal real P_real(t) desde odometría."""
        t = self._now()
        # 1) Distancia euclidiana al origen
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dist = sqrt(x*x + y*y)

        # 2) Yaw real calculado directamente con atan2
        q = msg.pose.pose.orientation
        yaw = atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )

        self.real_data.append((t, dist, yaw))

    def save_csvs(self):
        """Graba tres CSVs: ref_data.csv, act_data.csv y real_data.csv."""
        # Asegura que el directorio de trabajo exista (no debería fallar)
        os.makedirs('.', exist_ok=True)

        # --- 1) Referencias P_ref(t) ---
        with open('ref_data.csv',  'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t','ref_lin','ref_ang'])
            writer.writerows(self.ref_data)

        # --- 2) Actuaciones V_s(t) ---
        with open('act_data.csv',  'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t','act_lin','act_ang'])
            writer.writerows(self.act_data)

        # --- 3) Salida real P_real(t) ---
        with open('real_data.csv','w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t','dist','yaw'])
            writer.writerows(self.real_data)

        # Log final para confirmar
        self.get_logger().info(
            f"Guardados {len(self.ref_data)} referencias, "
            f"{len(self.act_data)} actuaciones, "
            f"{len(self.real_data)} datos reales en:\n"
            f"  ./ref_data.csv\n"
            f"  ./act_data.csv\n"
            f"  ./real_data.csv"
        )

def main():
    rclpy.init()
    node = DataRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
