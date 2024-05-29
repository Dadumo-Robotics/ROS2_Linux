import cv2
import numpy as np

# Este es el archivo base donde se ha desarrollado el tratamiento de la imagen con OpenCV
# Esta incluido con un ejemplo aunque posteriormente se implemente el codigo con el robot, pero aqui se puede probar con una imagen clara

img = cv2.imread('coca_cola_normal.jpg')

# Obtener las dimensiones de la imagen
height, width, channels = img.shape
# Mitad de las dimensiones de la imagen
height2 = height/2
width2 = width/2

# Aqui hacemos la detección de la imagen en base al color rojo y utilizando una mascara invertida, esto con la intencion de detectar todo menos el color de fondo
# Aunque en el ejemplo esta preparado para la deteccion de la coca cola para que sea mas facil de probar
# Posteriormente al filtro del color se sacan los contornos del objeto detectado para obtener el centro
def color_detection():
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    #lower_red = np.array([50,50,0])
    #upper_red = np.array([255,255,255])
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    lower_red2 = np.array([170,50,50])
    upper_red2 = np.array([180,255,255])

    mask = cv2.inRange(hsv, lower_red, upper_red)

    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    #res = cv2.bitwise_and(img, img, mask= mask)
    res = cv2.bitwise_not(cv2.bitwise_or(mask, mask2))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Suponemos que el contorno más grande es el objeto que buscamos
        largest_contour = max(contours, key=cv2.contourArea)

        # Calcular el centroide del contorno más grande
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # Dibujar el contorno y el centroide en la imagen
        cv2.drawContours(img, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1)
        cv2.putText(img, "centro", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        print(f"Centro del objeto: ({cX}, {cY})")
    else:
        print("No se encontraron objetos rojos")

    cv2.imshow('Imagen con centro del objeto', img)
    cv2.imshow('Mascara roja', res)
    cv2.waitKey(0) #aprieta una tecla
    cv2.destroyAllWindows()

# Deteccion de bordes con el algoritmo de Canny, descartada por el momento debido a que no es tan precisa pero se deja el codigo por si se necesita
# para futuros casos, ya que podria ser util en otras situaciones
def canny_detection():
    imagen_desenfocada = cv2.GaussianBlur(img,(5,5),0)

    imagen_canny = cv2.Canny(imagen_desenfocada, 50, 50)

    contours, _ = cv2.findContours(imagen_canny.copy(), mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(image=img,contours=contours,contourIdx=-1,color=(0,255,0),thickness=2, lineType=cv2.LINE_AA)

    cv2.imshow("Contornos", img)
    cv2.imshow('Imagen desenfocada', imagen_desenfocada)
    cv2.imshow('Imagen tras Canny', imagen_canny)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Deteccion de figuras geometricas, en este caso se ha implementado un algoritmo que detecta figuras geometricas basicas, estableciendo una
# figura lata
def figure_detection():
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Umbralizamos la imagen
    _, threshold = cv2.threshold(img_gray, 240, 255, cv2.THRESH_BINARY)

    # Buscamos los contornos
    contornos, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Iteramos sobre los contornos
    for contorno in contornos:

        poligonoAproximado = cv2.approxPolyDP(contorno, 0.01* cv2.arcLength(contorno, True), True)
        cv2.drawContours(img, [poligonoAproximado], 0, (0,0,0), 5)

        # Buscamos las coordenadas donde queremos escribir el nombre de la forma
        x = poligonoAproximado.ravel()[0]
        y = poligonoAproximado.ravel()[1] - 5

        # Basandonos en el núnmero de curvas poligonales determinamos de qué
        # forma geométrica se trata en cada caso
        numeroCurvas = len(poligonoAproximado)

        if numeroCurvas == 3:
            cv2.putText(img, 'Triangulo', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
        elif numeroCurvas == 4:
            x1, y1, w, h = cv2.boundingRect(poligonoAproximado)
            relacionDeAspecto = float(w)/h
            if (relacionDeAspecto > 0.95 and relacionDeAspecto <= 1.05):
                cv2.putText(img, 'Cuadrado', (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
            else:
                cv2.putText(img, 'Rectangulo', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
        elif numeroCurvas == 5:
            cv2.putText(img, 'Pentagono', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
        elif numeroCurvas == 10:
            cv2.putText(img, 'Estrella', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
        elif 5 < numeroCurvas < 10:
            cv2.putText(img, 'Lata', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
        else:
            cv2.putText(img, 'Circulo', (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))

    cv2.imshow('Formas geometricas', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Funcion que junta todos los metodos y trata de complementarlos entre ellos
# Primero detectamos la figura geometrica, luego el color y finalmente se busca el centro del objeto por cada uno de los metodos
def conjunto():
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Umbralizamos la imagen
    _, threshold = cv2.threshold(img_gray, 240, 255, cv2.THRESH_BINARY)
    
    # Buscamos los contornos
    contornos, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    cont_bueno = contornos[0]
    # Iteramos sobre los contornos
    for contorno in contornos:

        poligonoAproximado = cv2.approxPolyDP(contorno, 0.01* cv2.arcLength(contorno, True), True)
        cv2.drawContours(img, [poligonoAproximado], 0, (0,0,0), 5)

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
            if( width2-20 <= cX2 <= width2+20):
                print("Bien dentro")
                cont_bueno = contorno
                cv2.putText(img, 'Estrella', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
            else:
                print("Mal dentro")
            print("LLego")
        elif 5 < numeroCurvas < 10:
            #cv2.putText(img, 'Lata', (x,y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
            f = 6
            #cont_bueno = contorno
            #print("LLego")
    # Encuentra el momento del contorno
    M = cv2.moments(cont_bueno)

    cX = 0
    cY = 0
    if M["m00"] != 0:
        # Calcula la coordenada x y y del centro del contorno
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        #print(cX)
        #print(cY)
    else:
        # Si el área es cero, evita la división por cero
        cX, cY = 0, 0

    print(cX)
    print(cY)
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cXR = 0
    cYR = 0
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
        cv2.drawContours(img, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(img, (cXR, cYR), 7, (255, 0, 0), -1)
        cv2.putText(img, "centro", (cXR - 20, cYR - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        print(f"Centro del objeto: ({cXR}, {cYR})")
    else:
        print("No se encontraron objetos rojos")

    # Comprobamos que los centros coinciden (aproximadamente) asi sabemos que coinciden los centros y funcionan los metodos
    # Solo de la x porque es lo que nos interesa de cara a futuro, con y seria lo mismo y nos seguiria dando buen resultado
    if( cXR-20 <= cX <= cXR+20 ):
        print("Confirmamos objeto encontrado")
    else:
        print("No confirmamos objeto encontrado")

    # Mostrar las imágenes finales
    cv2.imshow('Imagen con detecciones', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

conjunto()