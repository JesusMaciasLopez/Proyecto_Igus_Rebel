import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt16

import can
import threading

class CanIONode(Node):
    def __init__(self):
        super().__init__('can_io_node')
        self.bus = can.interface.Bus(channel='can1', bustype='socketcan')

        # Pines
        self.digital_inputs = [6, 7, 8, 9]
        self.analog_inputs = [16, 17]  # A2 = 16, A3 = 17
        self.digital_outputs = [2, 3, 4, 5]
        self.analog_outputs = [14, 15]  # A0 = 14, A1 = 15

        # Publicadores de entradas
        self.input_publishers = {
            pin: self.create_publisher(Bool, f'entrada_digital_{pin}', 10)
            for pin in self.digital_inputs
        }
        self.analog_publishers = {
            pin: self.create_publisher(UInt16, f'entrada_analogica_A{pin - 14}', 10)
            for pin in self.analog_inputs
        }

        # Suscripciones para salidas
        for pin in self.digital_outputs:
            self.create_subscription(
                Bool, f'salida_digital_{pin}', lambda msg, p=pin: self.set_digital_output(p, msg.data), 10
            )

        for pin in self.analog_outputs:
            self.create_subscription(
                UInt16, f'salida_analogica_A{pin - 14}', lambda msg, p=pin: self.set_analog_output(p, msg.data), 10
            )

        # Temporizador para pedir estado cada 1s
        self.timer = self.create_timer(2.0, self.poll_inputs)

        # Hilo para recibir mensajes
        self.listener_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.listener_thread.start()

    def poll_inputs(self):
        for pin in self.digital_inputs:
            self.send_can_cmd(0x01, pin)  # Leer entrada digital

        for pin in self.analog_inputs:
            self.send_can_cmd(0x02, pin)  # Leer entrada analógica

    def set_digital_output(self, pin, value):
        self.get_logger().info(f"Escribiendo salida digital {pin}: {'HIGH' if value else 'LOW'}")
        self.send_can_cmd(0x03, pin, 1 if value else 0)

    def set_analog_output(self, pin, value):
        value = max(0, min(255, value))  # Limitar a 8 bits
        self.get_logger().info(f"Escribiendo salida analógica A{pin - 14}: {value}")
        self.send_can_cmd(0x07, pin, value)

    def send_can_cmd(self, cmd, pin, value=0):
        data = [cmd, pin, value] + [0] * 5
        msg = can.Message(arbitration_id=0x100, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(f"Error al enviar CAN: {e}")

    def receive_loop(self):
        while rclpy.ok():
            msg = self.bus.recv(timeout=1.0)
            if msg is None or len(msg.data) < 3:
                continue

            cmd, pin, val = msg.data[0], msg.data[1], msg.data[2]

            # Entrada digital
            if cmd == 0x04 and pin in self.input_publishers:
                msg_out = Bool()
                msg_out.data = bool(val)
                self.input_publishers[pin].publish(msg_out)

            # Entrada analógica
            elif cmd == 0x05 and pin in self.analog_publishers:
                value = (val << 8) | msg.data[3]  # 10-bit value (MSB first)
                msg_out = UInt16()
                msg_out.data = value
                self.analog_publishers[pin].publish(msg_out)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CanIONode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Finalizado')
    except Exception as error:
        print(error)

if __name__ == '__main__':
    main()
