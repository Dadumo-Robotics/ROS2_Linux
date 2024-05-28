import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist


class Ros2OpenCVImageConverter(Node):

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_publisher = self.create_publisher(Image, 'camera/image_processed', 10)  # Añadido para publicar imágenes
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # esto es posible que sea necesario para el robot REAL, image_raw es para SIMULACION
        #self.image_sub = self.create_subscription(Image,'/camera/image',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def camera_callback(self,data):
        #print("Llamando al callback!")
        
        # Publicar la imagen en el tópico camera/image
        try:
            image_message = self.bridge_object.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_publisher.publish(image_message)
        except CvBridgeError as e:
            print(e)


        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Aqui ya deberiamos ser capaces de trabajar con la imagen, y trabajar en la detección de objetos y la respuesta del robot a ellos
        self.centrar_camara(cv_image)
        cv2.imshow("Imagen capturada por el robot", cv_image)
        cv2.waitKey(1)

    # esta funcion debe ser llamada cuando se ha CONFIRMADO un objeto, no cuando detecta cualquier cosa
    def centrar_camara(self,imagen_obstaculo):
        # cambiar centro_imagen por un RANGO, para que no sea un solo pixel, es decir, cambiarlo a algo como [180, 230]
        # algo asi
        #centro_imagen1 = ancho_imagen/1.9
        #centro_imagen2 = ancho_imagen/2.1

        # dejo un print para ver si funciona
        #print("Esto funciona!")

        ancho_imagen = imagen_obstaculo.shape[1]
        centro_imagen = ancho_imagen/2
        msg = Twist()
        objeto_a_la_derecha = False
        objeto_a_la_izquierda = False
        if(objeto_a_la_derecha):
            msg.angular.z = -0.5
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando hacia la derecha')
        if(objeto_a_la_izquierda):
            msg.angular.z = 0.5
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando hacia la derecha')

        # de momento pilla bien el ancho de la imagen...
        # el objetivo ahora es que calculemos el centro del objeto de interés, y con ese centro, que intente girar al robot para que centro_imagen
        # y el centro de interés coincidan. Llamaremos al centro de interés "centro_interes"
        

        

def main(args=None):
    #print("Print 1 !!!")
    rclpy.init(args=args)
    img_converter_object = Ros2OpenCVImageConverter()
    #print("PRINT 2 !!!")
    try:
        rclpy.spin(img_converter_object)
        #print(" PRINT 3 !!!!!")
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")

    cv2.destroyAllWindows()




if __name__ == '__main__':
    main()