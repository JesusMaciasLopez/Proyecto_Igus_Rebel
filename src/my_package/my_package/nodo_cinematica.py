import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose,Point,Quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
from std_msgs.msg import Float64MultiArray

class cine_dir(Node):
    def __init__(self):
        super().__init__('nodo')
        AnchoBanda = QoSProfile(depth=10)
        AnchoBanda.reliability = QoSReliabilityPolicy.RELIABLE

        self.subscripcion = self.create_subscription(Twist,'xyz_mando',self.func_mando,AnchoBanda)

        self.publicador = self.create_publisher(Pose, 'cinematica_directa', AnchoBanda)
        self.NombreNodo = self.get_name()                                       
        self.get_logger().info(f'{self.NombreNodo} inicializado')               

        # Últimos valores recibidos del mando
        self.ultimo_mando = np.zeros(6)

        # Publicar periódicamente
        self.timer = self.create_timer(0.1, self.cinematica_dir_resultado)

    def matriz_DH(self, a, alpha, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),   np.sin(theta)*np.sin(alpha),    a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha),    -np.cos(theta)*np.sin(alpha),   a*np.sin(theta)],
            [0,             np.sin(alpha),                  np.cos(alpha),                  d],
            [0,             0,                              0,                              1],
        ])

    
    def func_mando(self, msg):
        escalado = 1
        self.ultimo_mando = np.array([
            msg.linear.x * escalado,
            msg.linear.y * escalado,
            msg.linear.z * escalado,
            msg.angular.x * escalado,
            msg.angular.y * escalado,
            msg.angular.z * escalado
        ])
    
    def cinematica_dir(self, angulo):
        param_tabla_DH = [
#            [0,     np.pi/2,    252,    angulo[0]+np.pi],
 #           [237,   0,          0,      angulo[1]+np.pi/2],
  #          [0,     np.pi/2,   0,      angulo[2]+np.pi/2],
   #         [0,     -np.pi/2,    297,    angulo[3]],
    #        [0,     np.pi/2,   0,      angulo[4]],
     #       [0,     0,          126,    angulo[5]]
            [0,     -np.pi/2,    252,    angulo[0]],
            [237,   0,          0,      angulo[1]-np.pi/2],
            [0,     np.pi/2,   0,      angulo[2]+np.pi/2],
            [0,     -np.pi/2,    297,    angulo[3]],
            [0,     np.pi/2,   0,      angulo[4]],
            [0,     0,          126,    angulo[5]]
        ]

        T = np.eye(4)
        for articulaciones in param_tabla_DH:
            a, alpha, d, theta = articulaciones
            T_i = self.matriz_DH(a, alpha, d, theta)
            T = np.dot(T, T_i)

        posicion_xyz = T[:3, 3]
        Rotacion = T[:3, :3]
        yaw = np.arctan2(Rotacion[1, 0], Rotacion[0, 0])
        pitch = np.arctan2(-Rotacion[2, 0], np.sqrt(Rotacion[0, 0]**2 + Rotacion[1, 0]**2))
        roll = np.arctan2(Rotacion[2, 1], Rotacion[2, 2])

        grados_yaw = np.degrees(yaw)
        grados_pitch = np.degrees(pitch)
        grados_roll = np.degrees(roll)

        return posicion_xyz, (grados_roll, grados_pitch, grados_yaw)
    
    def funcionQuaterniones(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return qx, qy, qz, qw

    def cinematica_dir_resultado(self):
        angulo = Float64MultiArray()
        angulo.data = [
            np.radians(0), 
            np.radians(0), 
            np.radians(0), 
            np.radians(0), 
            np.radians(90), 
            np.radians(0)
            ]

        posicion, rotacion = self.cinematica_dir(angulo.data)

        # Redondear valores cercanos a 0 para evitar errores de precisión
        epsilon = 1e-10  # Umbral para considerar un número como 0
        posicion = np.where(np.abs(posicion) < epsilon, 0, posicion)
        rotacion = [0 if abs(r) < epsilon else r for r in rotacion]
        
        quaternion = cine_dir.funcionQuaterniones(
            rotacion[0] + self.ultimo_mando[3],
            rotacion[1] + self.ultimo_mando[4],
            rotacion[2] + self.ultimo_mando[5]
            )
        self.get_logger().info(
            f'x: {rotacion[0]}, y: {rotacion[1]}, z: {rotacion[2]}, ')

        msg_control = Pose()
        msg_control.position = Point(
            x=posicion[0] + self.ultimo_mando[0],
            y=posicion[1] + self.ultimo_mando[1],
            z=posicion[2] + self.ultimo_mando[2]
        )
        msg_control.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        self.get_logger().info(
            f'x: {msg_control.position.x}, y: {msg_control.position.y}, z: {msg_control.position.z}, '
            f'qx: {msg_control.orientation.x}, qy: {msg_control.orientation.y}, '
            f'qz: {msg_control.orientation.z}, qw: {msg_control.orientation.w}'
        )
        self.publicador.publish(msg_control)

def main(args=None):
    try:
        rclpy.init(args=args)
        cinematica = cine_dir()
        rclpy.spin(cinematica)
    except KeyboardInterrupt:
        print('Finalizado')
    except Exception as error:
        print(error)

if __name__ == '__main__':
    main()
