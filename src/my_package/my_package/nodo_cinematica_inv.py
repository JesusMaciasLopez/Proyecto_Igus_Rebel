import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np
from rclpy.qos import QoSProfile,QoSReliabilityPolicy

class Cine_inversa(Node):
    def __init__(self):                                                     
        super().__init__('nodo_cinematica_inv')
        AnchoBanda = QoSProfile(depth=10)
        AnchoBanda.reliability = QoSReliabilityPolicy.RELIABLE

        self.d1 = 400 # Distancia base al primer eslabon
        self.l2 = 400 # Longitud del segundo eslabon 
        self.l3 = 400 # Longitud del tercer eslabon
        self.d6 = 126 # Distancia efector final(punta tercer eslabon) al ultimo eje

        self.subscripcion = self.create_subscription(
            Pose,'cinematica_directa',self.calc_angulos,AnchoBanda
        )
        self.publicador = self.create_publisher(
            Float64MultiArray,'angulos_teta',AnchoBanda
        )

    def calc_angulos(self,msg):                                              
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        Matriz_R = self.quaterniones_a_matriz_rotacion(qx,qy,qz,qw)

        try:
            Theta = self.calculo_cine_inv(x,y,z,Matriz_R)

            Angulos = Float64MultiArray()
            Angulos.data = Theta
            self.get_logger().info(
                f'Theta1: {Angulos.data[0]}, Theta2: {Angulos.data[1]}, Theta3: {Angulos.data[2]}'
                f'Theta4: {Angulos.data[3]}, Theta5: {Angulos.data[4]}, Theta6: {Angulos.data[5]}'
            )
            self.publicador.publish(Angulos)
        except ValueError as problema:
            self.get_logger().error(f"Error: {problema}")

    def quaterniones_a_matriz_rotacion(self,qx,qy,qz,qw):
        R = np.array([
            [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw),    2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw),   1-2*(qx**2+qz**2),  2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),    1-2*(qx**2+qy**2)]
        ])
        return R
    
    def calculo_cine_inv(self,x,y,z,Matriz_R):

        xw = x - self.d6 * Matriz_R[0,2]
        yw = y - self.d6 * Matriz_R[1,2]
        zw = z - self.d6 * Matriz_R[2,2]

        theta1 = np.arctan2(yw,xw)
        
        r = np.sqrt(xw**2 + yw**2)
        s = zw-self.d1
        D = (r**2+s**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)

        if abs(D) > 1:
            raise ValueError('Fuera de area de trabajo')
        
        theta3 = np.arccos(D)
        theta2 = np.arctan2(s,r)-np.arctan2(self.l3*np.sin(theta3),self.l2+self.l3*np.cos(theta3))

        R_3_6 = np.dot(np.linalg.inv(self.calcular_R_0_3(theta1,theta2,theta3)),Matriz_R)
        theta4, theta5, theta6 = self.rotacion_a_auler(R_3_6)

        return[np.degrees(theta1),np.degrees(theta2),np.degrees(theta3),
               np.degrees(theta4),np.degrees(theta5),np.degrees(theta6)]
    
    def calcular_R_0_3(self,theta1,theta2,theta3):
        theta1 = np.radians(theta1)
        theta2 = np.radians(theta2)
        theta3 = np.radians(theta3)

        R0_1 = np.array([
            [np.cos(theta1),    -np.sin(theta1),0],
            [np.sin(theta1),    np.cos(theta1), 0],
            [0,                 0,              1]
        ])
        R1_2 = np.array([
            [np.cos(theta2),    -np.sin(theta2),0],
            [np.sin(theta2),    np.cos(theta2), 0],
            [0,                 0,              1]
        ])
        R2_3 = np.array([
            [np.cos(theta3),    -np.sin(theta3),0],
            [np.sin(theta3),    np.cos(theta3), 0],
            [0,                 0,              1]
        ])
        R_0_3 = np.dot(np.dot(R0_1,R1_2),R2_3)
        return R_0_3

    def rotacion_a_auler(self,Matriz_R):
        theta4 = np.arctan2(Matriz_R[1,2],Matriz_R[0,2])
        theta5 = np.arccos(Matriz_R[2,2])
        theta6 = np.arctan2(Matriz_R[2,1],-Matriz_R[2,0])
        return theta4, theta5, theta6

def main(args=None):
    try:
        rclpy.init(args=args)
        cinematica_inv = Cine_inversa()
        rclpy.spin(cinematica_inv)
    except KeyboardInterrupt:
        print('...exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
