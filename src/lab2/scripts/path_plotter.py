#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class PathPlotter(Node):
    def __init__(self):
        super().__init__('path_plotter')
        cbg = ReentrantCallbackGroup()

        # Variables donde guardamos datos
        self.path_x = []
        self.path_y = []
        self.curr_x = None
        self.curr_y = None

        # Preparo la figura y ejes
        self.fig, self.ax = plt.subplots()
        self.path_line, = self.ax.plot([], [], 'b-', label='Plan')
        self.pose_scatter, = self.ax.plot([], [], 'ro', label='Robot')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.legend()
        self.ax.grid(True)

        # Subscríbete al plan y a la odometría
        self.create_subscription(Path,     'nav_plan', self.cb_path, 10, callback_group=cbg)
        self.create_subscription(Odometry, '/odom',     self.cb_odom, 10, callback_group=cbg)

        # Timer ROS para refrescar el plot (solo update de datos, no GUI)
        self.create_timer(0.1, self.update_data, callback_group=cbg)

    def cb_path(self, msg: Path):
        self.path_x = [p.pose.position.x for p in msg.poses]
        self.path_y = [p.pose.position.y for p in msg.poses]
        self.get_logger().info(f'PathPlotter: recibí plan con {len(self.path_x)} puntos')

    def cb_odom(self, msg: Odometry):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y

    def update_data(self):
        # Actualiza los objetos de la figura con los nuevos datos
        if self.path_x:
            self.path_line.set_data(self.path_x, self.path_y)
        if self.curr_x is not None:
            self.pose_scatter.set_data([self.curr_x], [self.curr_y])

        # Reajusta ejes
        self.ax.relim()
        self.ax.autoscale_view()

        # Redibuja el canvas
        self.fig.canvas.draw_idle()

def main(args=None):
    # Inicializa ROS y el nodo
    rclpy.init(args=args)
    node = PathPlotter()

    # Arranca spin de ROS en un hilo aparte
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Ahora lanzamos la ventana matplotlib en el hilo principal
    plt.show()

    # Cuando cierres la ventana, paran todo:
    node.get_logger().info("Ventana cerrada, apagando nodo…")
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
