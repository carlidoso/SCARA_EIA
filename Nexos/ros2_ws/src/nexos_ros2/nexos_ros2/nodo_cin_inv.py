import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
import math

class CinInv(Node):
    def __init__(self):
        super().__init__('cinematica_inversa')
        self.l1 = 21
        self.l2 = 25
        self.ang_mot_1 = Vector3()
        self.ang_mot_2 = Vector3()
        self.req_motor_ang1_publisher = self.create_publisher(Vector3, 'req_motor_ang1', 10)
        self.req_motor_ang2_publisher = self.create_publisher(Vector3, 'req_motor_ang2', 10)
        self.efector_final = self.create_subscription(Pose, 'efector_final', self.efector_callback, 10)

    def efector_callback(self, msg):
        try:
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            or_z = msg.orientation.z
            if 21.6 <= x <= 46 and 29.73**2-x**2 <= y**2 <= 46**2-x**2:
                self.calcular_cin_inv(x, y, z, or_z)
            elif -13.92 <= x < 21.6 and 29.73**2-x**2 <= y**2 <= (math.sqrt(25**2-(x-9.86)**2)+18.54)**2:
                self.calcular_cin_inv(x, y, z, or_z)
            else:
                self.get_logger().warning(f"Punto fuera del workspace")
        except Exception as e:
            self.get_logger().error(f"Se produjo un error en efector_callback: {e}")

    def calcular_cin_inv(self, x, y, z, or_z):
        try:
            r = math.sqrt(y**2+x**2)
            d = (r**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2)

            th_2_1 = math.atan(math.sqrt(1-d**2)/d)
            th_2_2 = -th_2_1

            th_1_1 = math.atan2(y, x) - math.atan(math.sin(th_2_1)*self.l2/(self.l1+math.cos(th_2_1)*self.l2))
            th_1_2 = math.atan2(y, x) - math.atan(math.sin(th_2_2)*self.l2/(self.l1+math.cos(th_2_2)*self.l2))

            self.ang_mot_1.x = math.degrees(th_1_2)
            self.ang_mot_1.y = math.degrees(th_2_2)     
            self.ang_mot_1.z = or_z
            self.req_motor_ang1_publisher.publish(self.ang_mot_1)

            self.ang_mot_2.x = z
            self.req_motor_ang2_publisher.publish(self.ang_mot_2)
        except Exception as e:
            self.get_logger().error(f"Se produjo un error en calcular_cin_inv: {e}")

def main(args=None):
    rclpy.init(args=args)
    cin_inv = CinInv()
    try:
        rclpy.spin(cin_inv)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        cin_inv.get_logger().warning("Se interrumpio desde consola")
    finally:
        if rclpy.ok():
            cin_inv.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
