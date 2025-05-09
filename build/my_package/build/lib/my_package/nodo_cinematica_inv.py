import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class CineInversa(Node):
    def __init__(self):                                                     
        super().__init__('nodo_cinematica_inv')
        AnchoBanda = QoSProfile(depth=10)
        AnchoBanda.reliability = QoSReliabilityPolicy.RELIABLE

        # Tabla DH ajustada
        self.param_tabla_DH = [
            [0, -np.pi/2, 252, 0],
            [237, 0, 0, -np.pi/2],
            [0, np.pi/2, 0, np.pi/2],
            [0, -np.pi/2, 297, 0],
            [0, np.pi/2, 0, 0],
            [0, 0, 126, 0]
        ]

        self.subscripcion = self.create_subscription(
            Pose, 'cinematica_directa', self.calc_angulos, AnchoBanda
        )
        self.publicador = self.create_publisher(
            Float64MultiArray, 'angulos_teta', AnchoBanda
        )

    def calc_angulos(self, msg):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        Matriz_R = self.quaterniones_a_matriz_rotacion(qx, qy, qz, qw)

        try:
            Theta = self.calculo_cine_inv(x, y, z, Matriz_R)

            Theta = [round(angle, 2) for angle in Theta]

            Angulos = Float64MultiArray()
            Angulos.data = Theta

            self.get_logger().info(
                f"Theta1: {Theta[0]}, Theta2: {Theta[1]}, Theta3: {Theta[2]}, "
                f"Theta4: {Theta[3]}, Theta5: {Theta[4]}, Theta6: {Theta[5]}"
            )

            self.publicador.publish(Angulos)
        except ValueError as problema:
            self.get_logger().error(f"Error: {problema}")

    def quaterniones_a_matriz_rotacion(self, qx, qy, qz, qw):
        R = np.array([
            [1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
            [2 * (qx*qy + qz*qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qx*qw)],
            [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx**2 + qy**2)]
        ])

        if not np.allclose(np.dot(R.T, R), np.eye(3), atol=1e-6):
            raise ValueError("La matriz de rotación no es ortonormal")

        return R

    def calculo_cine_inv(self, x, y, z, Matriz_R):
        d1 = self.param_tabla_DH[0][2]
        a2 = self.param_tabla_DH[1][0]
        d4 = self.param_tabla_DH[3][2]
        d6 = self.param_tabla_DH[5][2]

        xw = x - d6 * Matriz_R[0, 2]
        yw = y - d6 * Matriz_R[1, 2]
        zw = z - d6 * Matriz_R[2, 2]
        self.get_logger().info(f"xw: {xw}, yw: {yw}, zw: {zw}")

        D = (xw**2 + yw**2 + (zw - d1)**2 - a2**2 - d4**2) / (2 * a2 * d4)
        if abs(D) > 1:
            raise ValueError("D está fuera del rango permitido para arccos")

        theta3 = np.arccos(D)

        theta2 = (
            np.arctan2(zw - d1, np.sqrt(xw**2 + yw**2)) -
            np.arctan2(d4 * np.sqrt(1 - D**2), a2 + d4 * D)
        )
        var1 = zw - d1
        var2 = np.sqrt(xw**2 + yw**2)
        var3 = d4 * np.sqrt(1 - D**2)
        var4 = a2 + d4 * D
        self.get_logger().info(f"var1: {var1}, var2: {var2}, var3: {var3}, var4: {var4}")
        
        theta1 = np.arctan2(yw, xw)

        R_0_3 = self.calcular_R_0_3(theta1, theta2, theta3)
        self.get_logger().info(f"matriz: {R_0_3}")
        R_3_6 = np.dot(np.linalg.inv(R_0_3), Matriz_R)

        if np.isclose(R_3_6[2, 2], 1):
            theta4 = 0
            theta5 = 0
            theta6 = np.arctan2(R_3_6[0, 1], R_3_6[0, 0])
        elif np.isclose(R_3_6[2, 2], -1):
            theta4 = 0
            theta5 = np.pi
            theta6 = np.arctan2(R_3_6[0, 1], R_3_6[0, 0])
        else:
            theta5 = np.arccos(R_3_6[2, 2])
            theta4 = np.arctan2(R_3_6[1, 2], R_3_6[0, 2])
            theta6 = np.arctan2(R_3_6[2, 1], -R_3_6[2, 0])

        angulos = [
            self.normalizar_angulo(np.degrees(theta1)),
            self.normalizar_angulo(np.degrees(theta2-np.pi/2)),
            self.normalizar_angulo(np.degrees(theta3)),
            self.normalizar_angulo(np.degrees(theta4)),
            self.normalizar_angulo(np.degrees(theta5)),
            self.normalizar_angulo(np.degrees(theta6)),
        ]

        return angulos

    def calcular_R_0_3(self, theta1, theta2, theta3):
        theta1, theta2, theta3 = map(np.radians, [theta1, theta2, theta3])

        dh_params = self.param_tabla_DH[:3]
        T_0_3 = np.eye(4)
        for a, alpha, d, theta in dh_params:
            T_i = np.array([
                [np.cos(theta + theta1), -np.sin(theta + theta1) * np.cos(alpha), np.sin(theta + theta1) * np.sin(alpha), a * np.cos(theta + theta1)],
                [np.sin(theta + theta1), np.cos(theta + theta1) * np.cos(alpha), -np.cos(theta + theta1) * np.sin(alpha), a * np.sin(theta + theta1)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            T_0_3 = np.dot(T_0_3, T_i)

        return T_0_3[:3, :3]

    def normalizar_angulo(self, angulo):
        while angulo > 180:
            angulo -= 360
        while angulo < -180:
            angulo += 360
        return angulo

def main(args=None):
    try:
        rclpy.init(args=args)
        cinematica_inv = CineInversa()
        rclpy.spin(cinematica_inv)
    except KeyboardInterrupt:
        print('...exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()

