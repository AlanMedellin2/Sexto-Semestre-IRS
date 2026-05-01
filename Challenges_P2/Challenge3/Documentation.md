# Challenge3 Documentation

Los archivos:
* closed_loop_ctrl.py
* error.py

Están adaptados para usarse con los tópicos de la simulación. A continuación se explicará cómo debería de estar aplicado en la Rubik con los tópicos de la hackerboard:

## node_odometry.py
Este nodo se suscribe a los tópicos /VelocityEncR y /VelocityEncl (velocidad del encoder derecho e izquierdo respectivamente), y publica la odometría a través del tópico /encoder_odometry. 

En el modelo cinemático del robot diferencial cada rueda aporta la mitad de la velocidad

$$
V_R = rw_R
$$
$$
V_L = rw_L
$$        
        
La velocidad lineal es:
        
$$
V = \frac{V_R + V_L}{2}
$$

Y la velocidad angular es:

$$
W = \frac{V_R - V_L}{L}
$$

* $L$ es la distancia entre ambas ruedas
* $w_R$ es la velocidad angular de la rueda derecha ($w_R = \frac{d{\theta}_R}{dt}$)
* $w_L$ es la velocidad angular de la rueda izquierda ($w_L = \frac{d{\theta}_L}{dt}$)

Si solo conoces la posición de la rueda, no sabes si el robot en realidad se está moviendo. La velocidad angular (derivada de la posición del ángulo) te dice cuánto gira por segundo. Al multiplicar la velocidad angular por el radio, obtienes la velocidad lineal real. 

Una vez que obtenemos la velocidad lineal y angular del robot, vamos a predecir su posición.

1) El robot es no holonómico, por lo que solo puede avanzar en la dirección en la que apunta $\theta$:
* Eje x: hacia adelante o hacia atrás
* Eje y: lateral (no se puede)
Por lo tanto:
$V_x = V , V_y = 0$

2)  Queremos encontrar $(x,y)$ en el marco de referencia global. Para ello hay que rotar ese vector transformándolo del marco local al marco global tomando en cuenta su no holonomía:

$$
\begin{aligned}
\begin{bmatrix}
\dot{x} \\
\dot{y}
\end{bmatrix}
&=
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix}
v \\
0
\end{bmatrix}
\end{aligned}
$$

3) Al multiplicar obtenemos:
   
$$
\dot{x} = v\cos\theta
$$
$$
\dot{y} = v\sin\theta 
$$

5) El razonamiento base para obtener la posición es integrar la velocidad y sumársela a la condición inicial:

$$
x(t) = x_0 + \int \dot{x}(t)\, dt
$$

$$
y(t) = y_0 + \int \dot{y}(t)\, dt
$$

$$
\theta(t) = \theta_0 + \int \dot{\theta}(t)\, dt
$$

Pero no podemos realizar una integral continua en programación. Para ello, utilizamos la aproximación de Euler considerando que por un cierto intervalo de tiempo la velocidad es constante:

$$
x_{k+1} = x_k + \dot{x}_k \, \Delta t
$$

$$
y_{k+1} = y_k + \dot{y}_k \, \Delta t
$$

$$
\theta_{k+1} = \theta_k + \dot{\theta}_k \, \Delta t
$$

Sustituyendo el modelo cinemático:

$$
x_{k+1} = x_k + v_k \cos(\theta_k)\, \Delta t
$$

$$
y_{k+1} = y_k + v_k \sin(\theta_k)\, \Delta t
$$

$$
\theta_{k+1} = \theta_k + \omega_k\, \Delta t
$$

En dónde $v_k$ y $\omega_k$ son las velocidades lineale y angular previamente calculadas 

## error.py

El nodo de error se suscribe al tópico de odometría de los ecoders (/ground_truth en simulación) para tomar esa pose como la pose actual del robot. También recibe los puntos (goals) que el robot tiene que alcanzar en el mapa. Este nodo publica:
* /error_distance: error en m entre el punto actual y el punto goal
* /error_theta: error en radianes de alineación entre el heading actual y el deseado
* /estado: es solo un tópico que envía 1 o 0 en caso de que llegue al último punto de la secuencia

Los puntos (goals) se almacenan uno por uno en una cola llamada "buffer_metas". Mientras haya puntos de meta almacenados en la cola se van a calcular y publicar dos errores:

#### Error de distancia:
Se calcula mediante la fórmula de distancia entre dos puntos:

$$
d = \sqrt{(x_g - x_r)^2 + (y_g - y_r)^2}
$$

En dónde:
* $x_g$ $y_g$ son los componentes del punto de meta
* $x_r$ $y_r$ son los componentes del punto actual

#### Error de ángulo:

La orientación actual del robot ($\theta_{actual}$) es la conversión de la orientación en cuaterniones que nos entrega el tópico de odometría en ángulo de Euler (yaw). Esta conversión se hace mediante una función llamada "euler_from_quaternion()" que calcula el yaw con:

$$
\theta = \operatorname{atan2}(2(q_w * q_z + q_x * q_y)),(1 - 2({q_y}² + {q_x}²))
$$

donde:
* $q = (x,y,z,w)$
* $x² + y² + z² + w² = 1$

$atan2$ es una función trigonométrica que calcula el ángulo entre el eje x positivo del sistema de referencia (marco global) y el punto definido por las coordenadas catesianas (x,y). Básicamente calcula qué ángulo tiene la línea que conecta el robot con la meta respecto al eje x global:

<img width="1119" height="683" alt="image" src="https://github.com/user-attachments/assets/27afe202-228c-459f-9d3e-7836c37136f8" />

Por ejemplo, el ángulo deseado se calcula con la siguiente función de arcotangente2:

desired_yaw = math.atan2(y_g - y_r, x_g - x_r)

Primero hay que visualizar el vector que se forma de la resta de ambos puntos:
$$
(\Delta{x}, \Delta{y}) = (x_g - x_r, y_g - y_r)
$$

en donde: 
* $\Delta{x}$: cuánto hay que moverse en X para ir del robot a la meta
* $\Delta{y}$: cuánto hay que moverse en Y para ir del robot a la meta

Esto forma un vector con una componente x,y. Para obtener el ángulo de ese vector respecto al eje X global se utiliza la función "atan2()".

Por último, el error entre ambos ángulos se calcula con:

$$
\theta_{error} = \atan2{(\sin(\Delta{\theta})),((\cos(\Delta{\theta}))}
$$

en donde:

$$
\Delta{\theta} = \theta_{deseada} - \theta_{actual}
$$

Se aplican $\sin()$ y $\cos()$ para representar ese ángulo como un punto dentro del círculo unitario. De esta manera atan2() siempre devuelve un ángulo entre $(-\pi, \pi]$





 
