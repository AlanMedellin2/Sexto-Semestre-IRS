# Odometría con IMU

Para calcular la odometría, primero se calculó el bias (promedio de desviación) de 500 muestras del imu con el código "bias_imu.py". 

Después se utilizó integración por Euler para integrar la velocidad angular en z del IMU y pasarla a ángulo de rotación. Para ello, decidí aplicar un filtro de promedio movil para suavizar los cambios del IMU:

```python
self.w = msg.angular_velocity.z - self.bias

self.w_window.append(self.w)
w_filtered = sum(self.w_window) / len(self.w_window) #Filtro de promedio móvil
```

Luego establezco un umbral para que el ruido natural del sensor no haga que el robot piense que se está moviendo en estado de reposo:

```python
if abs(w_filtered) < 0.03: w_filtered = 0.0
```

Se aplica la integración de Euler. El razonamiento base para obtener la posición es integrar la velocidad y sumársela a la condición inicial:

$$
\theta(t) = \theta_0 + \int \dot{\theta}(t)\, dt
$$

Pero no podemos realizar una integral continua en programación. Para ello, utilizamos la aproximación de Euler considerando que por un cierto intervalo de tiempo la velocidad es constante:

$$
\theta_{k+1} = \theta_k + \dot{\theta}_k \, \Delta t
$$

Sustituyendo el modelo cinemático:


$$
\theta_{k+1} = \theta_k + \omega_k\, \Delta t
$$

En dónde $\omega_k$ es la velocidad angular que se obtiene del tópico del IMU:

```python
self.theta = self.wrap_to_pi(self.theta + w_filtered * dt)
```

Y se publican los cuaterniones:

```python
odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
```

## Métricas: Obtuve un 5% de error entre el ground truth y la odometría del IMU. La idea es reducirlo utilizando un Extended Kalman Filter para fusionar la odometría del IMU con la de los encoders.


<img width="719" height="122" alt="image" src="https://github.com/user-attachments/assets/30ce1071-8da2-4468-a119-e60152a7ed2b" />

Izquierda es Ground_truth y derecha es odometría del IMU. Se aprecia una diferencia de 19° aprox (5%) de 360°
