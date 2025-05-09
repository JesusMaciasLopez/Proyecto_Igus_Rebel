import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import subprocess

class ReceptorFiltro(Node):
    def __init__(self):
        super().__init__('receptor_filtro')
        AnchoBanda = QoSProfile(depth=10)
        AnchoBanda.reliability = QoSReliabilityPolicy.RELIABLE

        # Variables para almacenar las posiciones
        self.posiciones = [0] * 6

        # Publicador al tópico
        self.publicador = self.create_publisher(Float64MultiArray, 'angulinguis', AnchoBanda)

        self.get_logger().info('ReceptorFiltro inicializado')

    def procesar_mensaje(self, mensaje):
        try:
            partes_mensaje = mensaje.split()
            index = 1 if partes_mensaje[1] == '012' else 2 if partes_mensaje[1] == '022' else 3 if partes_mensaje[1] == '032' else 4 if partes_mensaje[1] == '042' else 5 if partes_mensaje[1] == '052' else 6 if partes_mensaje[1] == '062' else 0
            if partes_mensaje[3] == 'EF' and partes_mensaje[6] =='7E':
                numero = int(partes_mensaje[-2]+partes_mensaje[-1],16)
                self.posiciones[index] = numero

            # Publicar las posiciones
            angulitos = Float64MultiArray()
            angulitos.data = self.posiciones
            self.publicador.publish(angulitos)

            self.get_logger().info(f'Posiciones actualizadas: {self.posiciones}')

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Error al procesar el mensaje: {e}')

def main(args=None):
    rclpy.init(args=args)
    receptor_filtro = ReceptorFiltro()
    try:
        proceso = subprocess.Popen(['candump', 'can0'], stdout=subprocess.PIPE, text=True)
        for linea in proceso.stdout:
            receptor_filtro.procesar_mensaje(linea.strip())
    except KeyboardInterrupt:
        print('Nodo receptor finalizado')
    except Exception as error:
        print(f'Error: {error}')
    finally:
        receptor_filtro.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

