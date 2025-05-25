#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from kobuki_ros_interfaces.msg import BumperEvent

class TeleOp( Node ):

    def __init__( self ):
        super().__init__( ""'teleop_nav' "")
        #self.max_v = 1.0 # [m/s]
        #self.max_w = 0 # [rad/s]
        #primero para el simulador, segundo para el roboto
        self.cmd_vel_mux_pub = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 ) #le envia al sim
        self.velocidad_tb = self.create_publisher( Twist, "/commands/velocity", 10) #vamos a escribirle a sim y a roboto
        
        self.subscription = self.create_subscription(BumperEvent,"/events/bumper",self.choque_callback,10)
        
        timer_period = 0.05  # [s]
        self.timer = self.create_timer( timer_period, self.my_callback ) #estos son nuestros loops, cool no?
        #self.c = 0   
        self.obstacle_detected = 0 
        cv2.namedWindow("Pantalla de Teleoperacion") #colocarle un color o imagen?


    def my_callback( self ):
        speed = Twist()
        
        #speed.linear.x = 1.0 Esperemos que, tanto para el simulador que para kobuki robot, este sea el formato de vel
        #speed.angular.z = 0.0
        #elif self.c % 3 == 1: #la linea inferior parece q está modificando de a una los componentes de speed, que es de tipo Twist()
        #speed.linear.x = 0.0
        #speed.angular.z = 4.0
        key = cv2.waitKey(1) & 0xFF #se supone que para poco el timer de ros2
        if key == ord("i"): #6 if q largoooo, tanto if q loco amiwo
            speed.linear.x = 0.2
            speed.angular.z = 0.0
        elif key == ord("j"): 
            speed.linear.x = -0.2
            speed.angular.z = 0.0
        elif key == ord("a"): 
            speed.linear.x = 0.0
            speed.angular.z = 1.0
        elif key == ord("s"): 
            speed.linear.x = 0.0
            speed.angular.z = -1.0
        elif key == ord("q"): 
            speed.linear.x = 0.2
            speed.angular.z = 1.0
        elif key == ord("w"): 
            speed.linear.x = 0.2
            speed.angular.z = -1.0
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        #no se que son los (%f, %f)
        self.get_logger().info( 'publishing speed (%f, %f)') #esta está recibiendo, el % no se q es lo q hace la vdd
        
        if self.obstacle_detected != 1: #solo si es 1, no avanzamos
            self.cmd_vel_mux_pub.publish( speed ) #esta esta publicando
            self.velocidad_tb.publish( speed ) #grande los nodos, le escribe a dos y si uno no pesca no esta ni ahi
        #le podemos poner un img show para tener una pantalla, pero creo que se puede correr desde asi no mah
        solid_green = np.zeros((200,300,3),np.uint8)
        solid_green[:,:] = [143,188,143]
        self.get_logger().info(f"valor de las velocidades: {speed.linear.x,speed.linear.y}")
        cv2.imshow("Pantalla de Teleoperacion",solid_green)


    def choque_callback(self,msg): #asumo que está leyendo todo el rato!
        bumperr = BumperEvent() #esperemos funcione asi!
        bumperr.bumper = 1 #el bumper que va a leer!
        self.obstacle_detected = int(bumperr.state) #asi deberia funcionar bien 

def main(args=None):
    rclpy.init(args=args)
    mic = TeleOp() #no c pque el profe le puso de nombre mic
    rclpy.spin( mic )
    mic.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


