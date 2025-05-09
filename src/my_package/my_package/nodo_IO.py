#Estas librerias las necessitamos para poder trabajar con ros2 y la interfaz can
#Para trabajar en esta interfaz can lo que necesitamos es crear un ciclo que envie i escuche los mensajes y tambien necesitamos definir el tipo de mensajes que vamos a enviar

import rclpy                            #libreria para comunicacion con ros2
from rclpy.node import Node             #libreria para trabajar con nodos de ros2
from std_msgs.msg import Bool, UInt16   #librerias para trabajar con IO digitales (bool) y IO analogicas (UInt16)
import can                              #libreria para interactuar con el can
import threading                        #libreria para multitarea(enviar y esperar mensajes can)

#Creamos un objeto o clase donde lo inicializamos o construimos y lo conectamos al can

class CanIONode(Node):                                                      #Creamos la clase del nodo de entradas y salidas
    def __init__(self):                                                     #Lo definimos para que si inicie con otros nodos
        super().__init__('can_io_node')                                     #Ejecutamos el nodo
        self.bus = can.interface.Bus(channel='can1', bustype='socketcan')   #Definimos el canal y el socket que utilizara(este depende en donde trabajes, en linux es socketcan)

#Difinimos como queremos que sean los pines del arduino (14,15,16 y 17 son respectivamente A0,A1,A2 y A3)
#Los definimos con estos numeros para no liarnos con los numeros del arduino(se podrian nombrar DI0,DI1,...)
        
        self.digital_inputs = [6, 7, 8, 9]  #Definimos las entradas digitales con sus respectivos pines
        self.analog_inputs = [16, 17]       #Definimos las entradas analogicas con sus respectivos pines
        self.digital_outputs = [2, 3, 4, 5] #Definimos las salidas digitales con sus respectivos pines
        self.analog_outputs = [14, 15]      #Definimos las salidas analogicas con sus respectivos pines

#Publicamos las entradas ya que este nos informa el estado de las entradas
#Nos subscribimos a las salidas ya que esperaremos a que otro nodo nos diga cuando las queremos activar
#Hemos creado un topico por cada salida y entrada de forma que tendremos 12 topicos donde nos subcribiremos y publicaremos en cada uno
#Lo hacemos asi para facilitar la creacion de rutinas ja que nos facilita activar multiples IO ja que si sobreescribimos nos activa una i nos desactiva otra o problemas al compilar al trabajar con un vector(se puede con un vector pero no tenemos tiempo para solucionar problemas)
#Se publica com entrada_digital_x donde x es del 6 al 9 como true, lo mismo para las salidas digitales y las analogicas como salida_analogica_Ax donde x es del 0 al 1 lo mismo para las entradas 

        self.input_publishers = {                                                                                   #Publicamos las entradas digitales
            pin: self.create_publisher(Bool, f'entrada_digital_{pin}', 10)                                          #Lo publicamos como una variable de 0 o 1 en el numero de pin cada cierto tiempo
            for pin in self.digital_inputs                                                                          #Recorremos cada pin
        }
        self.analog_publishers = {                                                                                  #Publicamos las entradas analogicas
            pin: self.create_publisher(UInt16, f'entrada_analogica_A{pin - 14}', 10)                                #Lo publicaremos como una variable de 0 a 65535 en el numero de pin cada cierto tiempo(al no estar escalado en el arduino va del 0 al 255)
            for pin in self.analog_inputs                                                                           #Recorremos cada pin
        }
        for pin in self.digital_outputs:                                                                            #Recorremos cada salida
            self.create_subscription(                                                                               #Nos subcribimos a las salidas digitales
                Bool, f'salida_digital_{pin}', lambda msg, p=pin: self.set_digital_output(p, msg.data), 10          #Definimos como una varible booleana con su respectivo topico y lo ejcutamos cuando llega el topico(lambda msg) con el pin que estamos recorriendo cada cierto tiempo
            )
        for pin in self.analog_outputs:                                                                             #Recorremos cada salida analogica
            self.create_subscription(                                                                               #Nos subscribimos a las salidas analogicas
                UInt16, f'salida_analogica_A{pin - 14}', lambda msg, p=pin: self.set_analog_output(p, msg.data), 10 #Definimos como un Uint16(aunque nos llegan valores del 0 al 255 por falta de escalado) con su respectivo topico i cuando llega este con el pin que estamos recorriendo cada cierto tiempo
            )

#Temporizador de raspberry que se encarga de enviar i pedir los valores cada cierto tiempo
        
        self.timer = self.create_timer(1.0, self.poll_inputs)

#Recibimos los mensajes sin interrumpir los mensajes

        self.listener_thread = threading.Thread(target=self.receive_loop, daemon=True)  #Definimos como trabaja
        self.listener_thread.start()                                                    #Ejecutamos

#Hacemos una funcion con unos comandos para pedir la informacion del arduino (si envias estos comandos el arduino respondera con sus valores actuales)

    def poll_inputs(self):                  #Funcion para hacer una poll o llamada de las entradas
        for pin in self.digital_inputs:     #Recorremos cada pin
            self.send_can_cmd(0x01, pin)    #Leemos entrada digital
        for pin in self.analog_inputs:      #Recorremos cada pin
            self.send_can_cmd(0x02, pin)    #Leemos entrada analógica

#Hacemos una funcion que envie un comando para escribir en cada salida 

    def set_digital_output(self, pin, value):                                                       #Funcion para setear las salidas digitales
        self.get_logger().info(f"Escribiendo salida digital {pin}: {'HIGH' if value else 'LOW'}")   #Informamos por terminal a que estado vamos a poner la salida
        self.send_can_cmd(0x03, pin, 1 if value else 0)                                             #Comando para escribir el el pin deseado

#Hacemos una funcion que envie el comando para escribir las salidas analogicas

    def set_analog_output(self, pin, value):                                            #Funcion para setear las salidas analogicas
        value = max(0, min(255, value))                                                 #Limitamos a 255 por que no hemos escalado en el arduino
        self.get_logger().info(f"Escribiendo salida analógica A{pin - 14}: {value}")    #Informamos por terminal el valor que seteamos
        self.send_can_cmd(0x07, pin, value)                                             #Comando para setear los pines

#Funcion para enviar los valores por el can

    def send_can_cmd(self, cmd, pin, value=0):                                  #funcion que llamamos cuando queremos enviar algo al arduino donde ponemos comando + pin + el valor que enviamos
        data = [cmd, pin, value] + [0] * 5                                      #Creamos el mensaje que enviaremos por can
        msg = can.Message(arbitration_id=0x100, data=data, is_extended_id=False)#Formamos mensaje can con ID 100, el ID del arduino es 200, en caso de añadir mas dispositivos iremos poniendole otros IDs
        try:
            self.bus.send(msg)                                                  #Enviamos el mensaje can al arduino
        except can.CanError as e:
            self.get_logger().error(f"Error al enviar CAN: {e}")                #En caso de error de procesamiento de los mensajes nos devueve un error para que no se detenga el programa

#Funcion para recibir los valores por el can

    def receive_loop(self):                                         #Funcion que se encarga de recibir los mensajes can
        while rclpy.ok():                                           #Bucle para descartar los mensajes mientras el nodo esta funcionando
            msg = self.bus.recv(timeout=1.0)                        #Esperar el mensaje cada segundo
            if msg is None or len(msg.data) < 3:                    #En caso de que el mensaje no llegue o sea diferente a la largada de los valores salimos de la funcion
                continue
            cmd, pin, val = msg.data[0], msg.data[1], msg.data[2]   #Guardamos la informacion en variables del mensaje que nos llega
            if cmd == 0x04 and pin in self.input_publishers:        #En caso que que el codigo del mensaje sea 0x04 entrar para definir la entrada
                msg_out = Bool()                                    #Definimos la variable como un booleano
                msg_out.data = bool(val)                            #Guardamos la informacion que nos proporcian el mensaje en la variable
                self.input_publishers[pin].publish(msg_out)         #Publicamos el valor en el que se encuentra la entrada
            elif cmd == 0x05 and pin in self.analog_publishers:     #Si el comando que recibimos es 0x05 entramos para definir entradas analogicas
                value = (val << 8) | msg.data[3]                    #Formamos el valor como 2 bytes de 8 aunque solo usamos 10, Usamos Big Endian(el mas significativo primero)
                msg_out = UInt16()                                  #Definimos una variable como una variable Uint16
                msg_out.data = value                                #Guardamos el valor analogico dentro de la variable creada
                self.analog_publishers[pin].publish(msg_out)        #Publicamos el valor del pin en el topico de salida analogica

#Funcion principal del programa, estructura que usamos en Ros2 por el canal de Robotica con python

def main(args=None):
    try:                        #funcion para ejecutar lo que esta dentro asta que except lo interrumpa
        rclpy.init(args=args)   #inicializamos la comunicacion con los nodos
        node = CanIONode()      #creamos una instancia que se vincula a la clase
        rclpy.spin(node)        #hacemos el bucle de la instancia de la clase                                                                                                                                                                                   
    except KeyboardInterrupt:   #interrupcion del programa por (Ctrl + C)
        print('Finalizado')     #mostramos en terminal que a acabado el programa
    except Exception as error:  #en caso de que ocurra un error por Ros2 lo guardamos como una variable
        print(error)            #nos pone el error que a ocurrido en Ros2 por la terminal

#Aqui ejecutaremos el programa, de forma que llamamos a la funcion main i la main trabaja con la clase del principio del programa

if __name__ == '__main__':  #Estructura de python para que sea ejecutable
    main()                  #Principio del programa

#-----LISTA DE COMANDO DE RASPBERRY A ARDUINO-----#
#--0x01--Leer entrada digital
#--0x02--Leer entrada analogica
#--0x03--Escribir salida digital
#--0x07--Escirbir salida analogica
#-----LISTA DE COMANDO DE ARDUINO A RASPBERRY-----#
#--0x04--Estado entrada digital
#--0x05--Valor entrada analogica
#--0x06--Confirmacion salida digital
#--0x08--Confirmacion salida analogica