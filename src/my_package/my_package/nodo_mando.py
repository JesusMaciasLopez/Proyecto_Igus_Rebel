#Necesitamos estas libreria para poder tener una comunicacion con Ros i con el mando
#Las librerias de calidad las necesitamos para que no tengamos perdida de paquetes entre medio del proceso
#La varible de twist la usamos por que desde una variable podemos enviar todo lo que necesitamos, podriamos enviar tambien una matriz

import rclpy                                            #libreria para comunicacion de ros2
from rclpy.node import Node                             #libreria para trabajar con nodos de ros2
from geometry_msgs.msg import Twist                     #libreria para definir variable a valores angulares i liniales
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  #librerias que aseguran que se envie el mensaje cada cierto tiempo
from sensor_msgs.msg import Joy                         #libreria que se encarga de leer el mando

#Hacemos una clase para trabajar con las caracteristicas del objeto,en este caso el mando
#Las linias que siguen son un requisito para poder trabajar en el entorno de ros, por eso lo definmos como nodo i que se publique cada cierto tiempo

class PublicadorMando(Node):                                    #creamos una clase del publicador del mando
    def __init__(self):                                         #definimos __init__ para que se ejecute con varios nodos
        super().__init__('publicador_mando')                    #ejecutamos el nodo que va a trabajar
        AnchoBanda = QoSProfile(depth=10)                       #definimos variable que determine que se a ejecutado la accion cada tiempo especificado
        AnchoBanda.reliability = QoSReliabilityPolicy.RELIABLE  #definimos que la informacion no pierda paquetes en el camino

#Esta variable la inicializamos para que cuando se ejecute ya tenga un valor de los 2 que puede tener, ya que si no tiene un valor el programa no ara nada asta que pulsemos las teclas que veremos despues en el programa

        self.modo = 1       #inicializamos una variable que define si esta en modo linial o angular(la inicializamos en linial)

#En esta ocasion nos hace falta subscribirnos al mando, ja que si no trabajamos las funciones la llamar a los botones i potenciometros no funcionarian

        self.subscriptor = self.create_subscription(    #creamos la subscirpcion
            Joy, 'joy', self.mando_teclas, AnchoBanda)  #definimos que se subscriba al topico joy del nodo Joy,i que a partir de este ejecute la funcion mando teclas con el tiempo especificado de la variable

#Publicaremos el resultado que tenemos porque necesitamos trabajar con los valores que obtendremos de este programa para mover el robot

        self.publicador =  self.create_publisher(Twist, 'xyz_mando', AnchoBanda) #creamos un publicador que publica el topico xyz mando como variable twist cada tiempo especificado de la variable
        self.NombreNodo = self.get_name()                                       #obtenemos en nombre del nodo como una variable
        self.get_logger().info(f'{self.NombreNodo} inicializado')               #mensaje del loger en consola para que sepamos que a iniciado correctamente

#Esta es la funcion principal donde trabajaremos, i en caso de que el mando no sea de las mismas caracteristicas se tendra que cambiar las primeras linias

    def mando_teclas(self, msg):                                #creamos la funcion mando_teclas para ejecutar los comandos del mando
        if len(msg.axes) < 8 or len(msg.buttons) < 11:          #comprovamos que el mando tenga como maximo una cantidad de botones
            self.get_logger().error("Mensaje Joy incompleto.")  #si el mando no es compatible por la cantidad de botones el loger muestra un error en consola
            return                                              #nos saca del programa en caso de error

#Estos son los botones que usamos para el programa, por lo tanto solo emos definido estos
#Antes de definir se tienen que comprobar que coincidan los botones con el mensaje
#En este caso es un mando de Xbox 360(7 axes i 10 botones) pero solo emos definido 2 botones i 4 axes

        boton_LB = msg.buttons[4]       #definimos botones        
        boton_RB = msg.buttons[5]       #definimos botones
        joystick_esq_x = msg.axes[0]    #definimos potenciometros
        joystick_esq_y = msg.axes[1]    #definimos potenciometros
        joystick_dre_y = msg.axes[4]    #definimos potenciometros
        gatillo_rt = msg.axes[5]        #definimos potenciometros

#esto lo hacemos para definir el modo, lo emos puesto asi ja que si ponemos todo el programa dentro tendriamos que mantener pulsados estos botones para funcionar
#solo funciona con un clic i la variable se queda guardada con el valor

        if boton_LB:        #en caso de que pulsemos el boton lb entraremos en el modo linial
            self.modo = 1   #ponemos la variable al modo lineal
        elif boton_RB:      #en caso de que pulsemos el boton rb entraremos en el modo angular
            self.modo = 2   #ponemos la variable al modo angular

#Comprovamos que los joystick tengan un valor suficiente como para definir para que lado se mueven

        valor_x = 1 if joystick_esq_x > 0.5 else -1 if joystick_esq_x < -0.5 else 0     #definimos una variable para saber en que direccion nos movemos en x
        valor_y = 1 if joystick_esq_y > 0.5 else -1 if joystick_esq_y < -0.5 else 0     #definimos una variable para saber en que direccion nos movemos en y
        valor_z = 1 if joystick_dre_y > 0.5 else -1 if joystick_dre_y < -0.5 else 0     #definimos una variable para saber en que direccion nos movemos en z

#Creamos una variable que se ira actualizando en cada bucle para evitar que se quede una variable guardada del otro modo de funcionamiento
#La definimos como twist ja que esta sera la variable que usaremos para enviar al publicador, por eso le emos puesto el mismo nombre que el topico para no crear confusiones

        xyz_mando = Twist()     #definimos que la variable con la que vamos a trabajar es del tipo twist(variable para vectores liniales i angulares)

#Asignamos los valores de cada componente que forma la variable twist(6 valores -> 3 liniales(x,y,z) i 3 angulares(x,y,z))
#Los variables valor_(x,y,z) son la direccion i el gatillo es el multiplicador que se encarga de definir a cuanto en (x,y,z)se va a mover
#los calculos que hacemos en el gatillo son para pasar de los valores originales del gatillo que van des del -1 al 1 para que funcione del 0 al 1
#En caso de querer ajustar las velocidades puede poner un multiplicador despues de cada linia

        if self.modo == 1:                                          #en caso de estar en modo linial trabajaremos con los valores liniales
            xyz_mando.linear.x = valor_x * (2-(1+gatillo_rt))/2     #definimos cuanto nos movemos en el eje x
            xyz_mando.linear.y = valor_y * (2-(1+gatillo_rt))/2     #definimos cuanto nos movemos en el eje y
            xyz_mando.linear.z = valor_z * (2-(1+gatillo_rt))/2     #definimos cuanto nos movemos en el eje z
        elif self.modo == 2:                                        #en caso de estar en modo angular trabajaremos con los valores angulares
            xyz_mando.angular.x = valor_x * (2-(1+gatillo_rt))/2    #definimos cuanto nos movemos en el angulo rx
            xyz_mando.angular.y = valor_y * (2-(1+gatillo_rt))/2    #definimos cuanto nos movemos en el angulo ry
            xyz_mando.angular.z = valor_z * (2-(1+gatillo_rt))/2    #definimos cuanto nos movemos en el angulo rz

#Esto lo hacemos para comprovar que los valores que tenemos de la variable sean correctos
#Ahora mandamos la variable asta el publicador de arriba del programa para que lo envie como un topico, luego este topico lo cojera otro nodo para trabajar con este

        self.get_logger().info(                                                                 #el loger nos manda informacion en la terminal
            f'x: {xyz_mando.linear.x}, y: {xyz_mando.linear.y}, z: {xyz_mando.linear.z}, '      #hacemos que nos mande los valores de x,y,z
            f'rx: {xyz_mando.angular.x}, ry: {xyz_mando.angular.y}, rz: {xyz_mando.angular.z}'  #hacemos que nos mande los valores de rx,ry,rz
        )
        self.publicador.publish(xyz_mando)  #publicamos la variable xyz_mando para que lo publique en el topico con el mismo nombre

#Este sera una estructura generica de un nodo en python de ros, este def main sera casi siempre igual por como hemos estructurado los nodos
#Solo comunicamos con Ros i hacemos el bucle para que se ejecute ciclicamente i definos alguna forma para detener el programa

def main(args=None):
    rclpy.init(args=args)           #inicializamos la comunicacion con los nodos
    try:                            #funcion para ejecutar lo que esta dentro asta que except lo interrumpa
        mando = PublicadorMando()   #creamos una instancia que se vincula a la clase
        rclpy.spin(mando)           #hacemos el bucle de la instancia de la clase
    except KeyboardInterrupt:       #interrupcion del programa por (Ctrl + C)
        print('finalizado')         #mostramos en terminal que a acabado el programa
    except Exception as error:      #en caso de que ocurra un error por Ros2 lo guardamos como una variable
        print(error)                #nos pone el error que a ocurrido en Ros2 por la terminal
    finally:                        #en caso de que finalice el programa
        rclpy.shutdown()            #acaba la comunicacion con los nodos de Ros2

#Aqui ejecutaremos el programa, de forma que llamamos a la funcion main i la main trabaja con la clase del principio del programa

if __name__ == '__main__':  #Estructura de python para que sea ejecutable
    main()                  #Principio del programa

#-----TENER-EN-CUENTA-----#
#para que este nodo funcione tienes que arrancar el nodo del mando(puede ser antes o despues)

#-----PASOS-PARA-ARRANCAR-NODO-JOY-----#

#--1.abrir una terminal (Ctrl + Alt + T)-------------------------------------#
#--2.Entrar en el espacio de trabajo de las carpetas donde estes trabajando--#
#------cd <work space>-------------------------------------------------------#
#--3.Arrancar ros para que el terminal trabaje con el------------------------#
#------source install/setup.bash---------------------------------------------#
#--4.Ejecutar el nodo del mando----------------------------------------------#
#------ros2 run joy joy_node-------------------------------------------------#
#--5.abrir otra terminal (Ctrl + Alt + T)------------------------------------#
#--6.Entrar en el espacio de trabajo de las carpetas donde estes trabajando--#
#------cd <work space>-------------------------------------------------------#
#--7.Arrancar ros para que el terminal trabaje con el------------------------#
#------source install/setup.bash---------------------------------------------#
#--8.Ejecutar este nodo------------------------------------------------------#
#------ros2 run my_package nodo_mando----------------------------------------#