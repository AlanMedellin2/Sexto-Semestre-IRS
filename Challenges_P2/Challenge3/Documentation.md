# Challenge3 Documentation

Los archivos:
* closed_loop_ctrl.py
* error.py

Están adaptados para usarse con los tópicos de la simulación. A continuación se explicará cómo debería de estar aplicado en la Rubik con los tópicos de la hackerboard:

## node_odometry.py
Este nodo se suscribe a los tópicos /VelocityEncR y /VelocityEncl (velocidad del encoder derecho e izquierdo respectivamente), y publica la odometría a través del tópico /encoder_odometry. 

El modelo cinemático del robot diferencial es:

La ecuación es $E = mc^2$.
