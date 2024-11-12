#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from resource.app_scara.main import MainApp

class NexosApp(Node):
    def __init__(self):
        super().__init__('nexos_app')
        # Para cinematica directa
        self.req_motor_ang1_publisher = self.create_publisher(Vector3, 'req_motor_ang1', 10) # 10 mensajes en cola maximo antes de empezar a borrar los mas viejos
        self.req_motor_ang2_publisher = self.create_publisher(Vector3, 'req_motor_ang2', 10)
        # Para cinematica inversa
        self.efector_final = self.create_publisher(Pose, 'efector_final', 10)


def main(args=None):
    rclpy.init(args=args)
    nexos_app = NexosApp()
    main_app = MainApp(req_motor_ang1_publisher= nexos_app.req_motor_ang1_publisher, req_motor_ang2_publisher= nexos_app.req_motor_ang2_publisher, efector_final= nexos_app.efector_final, tipo_pose= Pose, tipo_vector= Vector3)
    try:
        # Iniciar la aplicación Tkinter en un hilo separado
        tkinter_thread = threading.Thread(target= main_app.iniciar_app)
        tkinter_thread.start()
        rclpy.spin(nexos_app)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        print("Se interrumpio desde consola")
    finally:
        if rclpy.ok():
            nexos_app.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()