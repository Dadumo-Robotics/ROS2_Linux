import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
import cv2
import numpy as np

# En cuanto al procesamiento con el robot real damos prioridad a la deteccion por color ya que con una imagen real resulta muy complicado
# detectar la forma y aumenta severamente la imprecision, por lo tanto con el color detectamos el objeto sin problemas y posteriormente
# obtenemos el centro de este
# Realiza el proceso perfecto pero tiene problemas para girar (en capture_mov.py esta el movimiento solo sin openCV que funciona)
class Ros2OpenCVImageConverter(Node):

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')

        self.img = Image()



        print("Nodo inicializado")
        self.image_publisher = self.create_publisher(Image, 'camera/image_processed', 10)  # Añadido para publicar imágenes
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/image',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # el topic image_raw es para SIMULACION
        #self.image_sub = self.create_subscription(Image,'/camera/image',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        

    def camera_callback(self,data):
        print("Llamando al callback!")
        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Publicar imagen a la web
        try:
            image_message = self.bridge_object.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_publisher.publish(image_message)
        except CvBridgeError as e:
            print(e)

        self.img = cv_image
        # Obtener las dimensiones de la imagen
        height, width, channels = self.img.shape
        self.height2 = height/2
        self.width2 = width/2

        print("Height: " + str(height))
        print("W: " + str(width))
        # Aqui ya deberiamos ser capaces de trabajar con la imagen, y trabajar en la detección de objetos y la respuesta del robot a ellos
        self.conjuntopro()
        #self.centrar_camara(cv_image)
        cv2.imshow("Imagen capturada por el robot", cv_image)
        cv2.waitKey(1)

    # esta funcion debe ser llamada cuando se ha CONFIRMADO un objeto, no cuando detecta cualquier cosa
    def centrar_camara(self, xCI, yCI):
        # cambiar centro_imagen por un RANGO, para que no sea un solo pixel, es decir, cambiarlo a algo como [180, 230]
        # algo asi
        #centro_imagen1 = ancho_imagen/1.9
        #centro_imagen2 = ancho_imagen/2.1

        # dejo un print para ver si funciona
        print("Esto funciona!")

        #self.img = imagen_obstaculo
        msg = Twist()
        objeto_a_la_derecha = False
        objeto_a_la_izquierda = False

        if(xCI > self.width2):
            msg.angular.z = -0.5
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando hacia la derecha')
            #while(True):
            #    self.get_logger().info(str(msg.angular.z))
            # Este log nos muestra que el mensage angular z siempre es -0.5 y no cambia, el robot no gira por otra cosa
            
        elif(xCI < self.width2):
            msg.angular.z = 0.5
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando hacia la izquierda')
        else:
            self.get_logger().info('Conseguido')

    def conjuntopro(self):
        img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        # Umbralizamos la imagen
        _, threshold = cv2.threshold(img_gray, 240, 255, cv2.THRESH_BINARY)
        
        # Buscamos los contornos
        contornos, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        cont_bueno = contornos[0]
        # Iteramos sobre los contornos
        for contorno in contornos:

            poligonoAproximado = cv2.approxPolyDP(contorno, 0.01* cv2.arcLength(contorno, True), True)
            cv2.drawContours(self.img, [poligonoAproximado], 0, (0,0,0), 5)

            # Buscamos las coordenadas donde queremos escribir el nombre de la forma
            x = poligonoAproximado.ravel()[0]
            y = poligonoAproximado.ravel()[1] - 5

            # Basandonos en el núnmero de curvas poligonales determinamos de qué
            # forma geométrica se trata en cada caso
            numeroCurvas = len(poligonoAproximado)

            if numeroCurvas == 10:
                #cv2.putText(img, 'Estrella', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
                cX2 = 0
                cY2 = 0
                M2 = cv2.moments(contorno)
                if M2["m00"] != 0:
                    # Calcula la coordenada x y y del centro del contorno
                    cX2 = int(M2["m10"] / M2["m00"])
                    cY2 = int(M2["m01"] / M2["m00"])
                    print(cX2)
                    print(cY2)
                else:
                    # Si el área es cero, evita la división por cero
                    cX2, cY2 = 0, 0
                if( self.width2-20 <= cX2 <= self.width2+20):
                    print("Bien dentro")
                    cont_bueno = contorno
                    cv2.putText(self.img, 'Estrella', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
                else:
                    print("Mal dentro")
                print("LLego")
            elif 5 < numeroCurvas < 10:
                cv2.putText(self.img, 'Lata', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
            else:
                cv2.putText(self.img, 'Circulo', (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
        # Encuentra el momento del contorno
        M = cv2.moments(cont_bueno)

        cX = 0
        cY = 0
        if M["m00"] != 0:
            # Calcula la coordenada x y y del centro del contorno
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            # Si el área es cero, evita la división por cero
            cX, cY = 0, 0

        print(cX)
        print(cY)
        
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # Definir rangos de color rojo en HSV
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Crear máscaras para los rangos de color rojo
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Aplicar la máscara a la imagen original
        #res = cv2.bitwise_and(img, img, mask=mask)
        res = cv2.bitwise_not(cv2.bitwise_or(mask1, mask2))
        cv2.imshow('res',res)

        contours, _ = cv2.findContours(cv2.bitwise_or(mask1, mask2), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Suponemos que el contorno más grande es el objeto que buscamos
            largest_contour = max(contours, key=cv2.contourArea)

            # Calcular el centroide del contorno más grande
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cXR = int(M["m10"] / M["m00"])
                cYR = int(M["m01"] / M["m00"])
            else:
                cXR, cYR = 0, 0

            # Dibujar el contorno y el centroide en la imagen
            cv2.drawContours(self.img, [largest_contour], -1, (0, 255, 0), 2)
            cv2.circle(self.img, (cXR, cYR), 7, (255, 0, 0), -1)
            cv2.putText(self.img, "centro", (cXR - 20, cYR - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            self.centrar_camara(cXR, cYR)

            print(f"Centro del objeto: ({cXR}, {cYR})")
        else:
            print("No se encontraron objetos rojos")

        cv2.imshow('Imagen con centro del objeto', self.img)
        cv2.imshow('Mascara roja', res)

        # Mostrar las imágenes finales
        cv2.imshow('Imagen con detecciones', self.img)
        cv2.imshow('Imagen con máscara', res)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        

        

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