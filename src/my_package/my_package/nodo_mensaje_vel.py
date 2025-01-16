import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import subprocess

class Velocidad(Node):
    def __init__(self):
        super().__init__('nodo_cinematica_inv')
        AnchoBanda = QoSProfile(depth=10)
        AnchoBanda.reliability = QoSReliabilityPolicy.RELIABLE

        self.subscripcion2 = self.create_subscription(
            Float64MultiArray, 'angulinguis', self.comparar, AnchoBanda
        )
        self.subscripcion = self.create_subscription(
            Float64MultiArray, 'angulos_teta', self.mensaje, AnchoBanda
        )

    def mensaje(self, msg):
        if len(msg.data) != 6:
            self.get_logger().error("Faltan o sobran elementos")
            return

        angulo1 = self.ang(round(msg.data[0]))
        angulo2 = self.ang(round(msg.data[1]))
        angulo3 = self.ang(round(msg.data[2]))
        angulo4 = self.ang(round(msg.data[3]))
        angulo5 = self.ang(round(msg.data[4]))
        angulo6 = self.ang(round(msg.data[5]))

        self.get_logger().info(
            f'angulo 1: {angulo1}, angulo 2: {angulo2}, angulo 3: {angulo3}, '
            f'angulo 4: {angulo4}, angulo 5: {angulo5}, angulo 6: {angulo6}'
        )
        velocidad = self.comparar(round(msg.data))

        mensaje1 = f'cansend can0 010#25 00 {velocidad[0]}'
        mensaje2 = f'cansend can0 010#25 00 {velocidad[1]}'
        mensaje3 = f'cansend can0 010#25 00 {velocidad[2]}'
        mensaje4 = f'cansend can0 010#25 00 {velocidad[3]}'
        mensaje5 = f'cansend can0 010#25 00 {velocidad[4]}'
        mensaje6 = f'cansend can0 010#25 00 {velocidad[5]}'
        subprocess(mensaje1)
        subprocess(mensaje2)
        subprocess(mensaje3)
        subprocess(mensaje4)
        subprocess(mensaje5)
        subprocess(mensaje6)

    def decimal_a_binario(self, num):
        if num == 0:
            return "0"
        binario = ""
        while num > 0:
            residuo = num % 2
            binario = str(residuo) + binario
            num //= 2
        return binario

    def complento_a_2_binario(self, binario):
        if binario[0] == "0":  # Número positivo
            return binario
        # Manejo de números negativos
        num = int(binario, 2)
        bits = len(binario)
        rango = 1 << bits
        complemento = rango - num
        complemento_binario = f"{complemento:0{bits}b}"
        return complemento_binario

    def binario_a_hexadecimal(self, binario):
        if not all(bit in '01' for bit in binario):
            raise ValueError("El número ingresado no es un binario")
        decimal = int(binario, 2)
        hexadecimal = hex(decimal)[2:].upper()
        return hexadecimal

    def ang(self, num):
        binario = self.decimal_a_binario(abs(num))  # Usamos valor absoluto
        if num < 0:
            binario = self.complento_a_2_binario(binario)
        hexadecimal = self.binario_a_hexadecimal(binario)
        return hexadecimal
    def comparar (self,msg,angulo):
        num1 = 32 if  angulo[0] < msg.data[0] else () if angulo[0] > msg.data[0] else 0
        num2 = 32 if  angulo[1] < msg.data[1] else () if angulo[1] > msg.data[1] else 0
        num3 = 32 if  angulo[2] < msg.data[2] else () if angulo[2] > msg.data[2] else 0
        num4 = 32 if  angulo[3] < msg.data[3] else () if angulo[3] > msg.data[3] else 0
        num5 = 32 if  angulo[4] < msg.data[4] else () if angulo[4] > msg.data[4] else 0
        num6 = 32 if  angulo[5] < msg.data[5] else () if angulo[5] > msg.data[5] else 0

        lista = [
            num1,
            num2,
            num3,
            num4,
            num5,
            num6
        ]
        return lista
        

def main(args=None):
    try:
        rclpy.init(args=args)
        mensajes_can = Velocidad()
        rclpy.spin(mensajes_can)
    except KeyboardInterrupt:
        print('...exit node')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
