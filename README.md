# ROS_Linux

## Descripción del Proyecto

Este repositorio forma parte del proyecto de Navegación ROS (Robot Operating System). Aquí se proporcionan instrucciones detalladas sobre cómo utilizar ROS para controlar un robot simulado y real, así como para controlar el robot desde la web.

## Estructura del Proyecto

El proyecto está organizado en varias carpetas para facilitar la comprensión y el funcionamiento del código. A continuación se detalla la estructura del proyecto:

- **my_nav2_system**: Contiene los archivos relacionados con la navegación del robot utilizando ROS Navigation 2 (Nav2). Aquí se encuentran los archivos de configuración y lanzamiento para controlar el robot simulado y real, así como para habilitar el control del robot desde la web.

- **my_first_service**: Contiene los archivos necesarios para proporcionar un servicio web para controlar el movimiento del robot utilizando comandos de flechas desde una interfaz web.

## Instrucciones de Uso

### Simulación del Robot

Para simular el movimiento del robot y ejecutar una trayectoria, sigue estos pasos:

1. Actualiza el archivo `my_nax2_params.yaml` con la ruta correcta al archivo de configuración de navegación.
2. Ejecuta `colcon build --packages-select my_nav2_system` para compilar el paquete.
3. Fuente `source install/setup.bash`.
4. En tres terminales diferentes, ejecuta los siguientes comandos:

   - Terminal 1:
     ```bash
     ros2 launch my_nav2_system my_nav2_waypoints_follower.launch.py
     ```

   - Terminal 2:
     ```bash
     ros2 run my_nav2_system my_waypoint_follower_sim
     ```

   - Terminal 3:
     ```bash
     ros2 service call /start_waypoint_following std_srvs/srv/Trigger "{}"
     ```

### Control del Robot desde la Web (Simulación)

Para controlar el robot simulado desde la web, sigue estos pasos:

1. En un terminal, ejecuta:
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml

2. En otro terminal, ejecuta:
   ```bash
   ros2 launch my_nav2_system my_nav2_waypoints_follower.launch.py

3. En otro terminal, ejecuta:
   ```bash
   ros2 run my_nav2_system my_waypoint_follower_sim

4. En otro terminal, ejecuta:
   ```bash
   ros2 launch my_first_service movement_server_launch2.launch.py

4. Lanza un servidor HTTP Python en el directorio del proyecto:
   ```bash
   python -m http.server 8000

### Control del Robot Real desde la Web

Este apartado describe cómo controlar un robot real utilizando ROS desde una interfaz web.

### Pasos a Seguir

Para controlar el robot real desde la web, sigue estos pasos:

1. **Conexión al Robot**: Conéctate al robot a través de SSH utilizando el siguiente comando en un terminal:

   ```bash
   ssh usuario@192.168.0.62
   
Reemplaza usuario por el nombre de usuario del robot y 192.168.0.62 por la dirección IP del robot. 

2. Inicio del Sistema del Robot: En un terminal, ejecuta el siguiente comando para iniciar el sistema del robot:

   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   
Este comando iniciará todos los nodos necesarios para el funcionamiento del robot.

3. Conexión al Servidor ROS Bridge: En otro terminal, ejecuta el siguiente comando para conectar el servidor ROS Bridge:
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   
Este comando establecerá una conexión WebSocket para comunicarse con el servidor ROS desde la interfaz web.

4. Inicio del Sistema de Navegación: En otro terminal, ejecuta el siguiente comando para iniciar el sistema de navegación del robot:

   ```bash
   ros2 launch my_nav2_system my_nav2_waypoints_follower.launch.py
   
Este comando iniciará el sistema de navegación que permite al robot moverse hacia puntos de referencia especificados.

5.Inicio del Control desde la Web: En otro terminal, ejecuta el siguiente comando para iniciar el nodo de control desde la web:

   ```bash
   ros2 run my_nav2_system my_waypoint_follower
   ```
Este comando habilitará la comunicación entre el servidor web y el robot para enviar comandos de control.

6. Inicio del Servidor HTTP: En otro terminal, lanza un servidor HTTP Python en el directorio del proyecto:

   ```bash
   python -m http.server 8000

Este comando iniciará un servidor web en el puerto 8000 para la interfaz de control desde la web.

7. Control del Robot desde la Web: Abre un navegador web e ingresa la dirección URL del servidor HTTP (por lo general, http://localhost:8000). Utiliza la interfaz web proporcionada para enviar comandos de control al robot.


