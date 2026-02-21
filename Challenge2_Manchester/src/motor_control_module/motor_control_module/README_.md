Iniciar sistema: ros2 service call /init custom_interfaces/srv/Initiate "{command: {data: 'resume'}}"
Parar sistema: ros2 service call /init custom_interfaces/srv/Initiate "{command: {data: 'stop'}}"

Los nodos de dc_motor.py y set_point.py tienen un subscriptor a /init_system que apaga y prende el sistema. server.py publica la información por ese tópico. controller.py deberá de tener esa misma suscripción
