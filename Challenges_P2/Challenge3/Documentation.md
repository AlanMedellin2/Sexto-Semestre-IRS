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
\begin{bmatrix}
\dot{x} \\
\dot{y}
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix}
v \\
0
\end{bmatrix}
$$

3) Al multiplicar obtenemos:
$$
\dot{x} = v\cos\theta \\
\dot{y} = v\sin\theta 
$$

4) El razonamiento base para obtener la posición es integrar la velocidad y sumársela a la condición inicial:

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


 
