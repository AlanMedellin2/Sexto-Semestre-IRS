# La Rubik tiene el siguiente PINOUT:

<img width="584" height="383" alt="image" src="https://github.com/user-attachments/assets/87f9db85-c0bb-4bac-87a5-92728bd961fd" />

<img width="609" height="315" alt="image" src="https://github.com/user-attachments/assets/96720c60-7a10-4373-8633-5d56bbc43fd0" />

# El acelerómetro tiene los siguiente ejes de sensibilidad:
<img width="667" height="286" alt="image" src="https://github.com/user-attachments/assets/0f7384d5-8328-401d-af2a-e418649fd15b" />

## Características:

### Gyroscope Features
The triple-axis MEMS gyroscope in the MPU-60X0 includes a wide range of features:
* Digital-output X-, Y-, and Z-Axis angular rate sensors (gyroscopes) with a user-programmable fullscale range of ±250, ±500, ±1000, and ±2000°/sec: Nos da la velocidad de rotación en grados por segundo. El rango programable ajusta la resolución.
* External sync signal connected to the FSYNC pin supports image, video and GPS synchronization: útil si hacemos Visual Odometry con una cámara
* Integrated 16-bit ADCs enable simultaneous sampling of gyros
* Improved low-frequency noise performance: Ayuda a que las vibraciones del robot no afecten las mediciones
* Digitally-programmable low-pass filter
* Gyroscope operating current: 3.6mA
* Standby current: 5µA

### Accelerometer Features
The triple-axis MEMS accelerometer in MPU-60X0 includes a wide range of features:
* Digital-output triple-axis accelerometer with a programmable full scale range of ±2g, ±4g, ±8g and
±16g: Solo sirve para detectar si el robot está inclinado
* Integrated 16-bit ADCs enable simultaneous sampling of accelerometers while requiring no external
multiplexer: útil para implementar Filtro de Kalman
* Accelerometer normal operating current: 500µA

### Objetivo: El acelerómetro es perfecto para calcular el heading ($\theta$) de nuestro robot. Los encoders se encargarán de calcular el desplazamiento. 

### Voltaje de funcionamiento: 3.3 V a 5 V.

### Sus pines son:
<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/872e4994-f423-46b5-8594-9560ea3d1701" />

* INT: Blanco
* ADO: Morado
* XCL: Azul
* XDA: Verde
* SDA: Amarillo
* SCL: Naranja
* GND: Rojo
* VCC: Cafe

Solo se usan:
* INT: Blanco
* SCL: Naranja
* SDA: Amarillo
* GND: Rojo
* VCC: Cafe

Y se conectan a la rubik en:
* INT -> 7 (jetson)
* SCL -> 5 (jetson)
* GND -> GND
* SDA -> 3 (jetson)
* Vcc --> 5 v (2 --> jetson)


# Odometría medinate velocidades del IMU 

Para obtener la posición a partir del IMU, necesitas realizar una doble integración para la traslación y una integración simple para la rotación.
* Aceleración $\xrightarrow{\int}$ Velocidad $\xrightarrow{\int}$ Posición
* Velocidad Angular $\xrightarrow{\int}$ Orientación (Ángulo)

Debido al ruido en las mediciones de aceleración no es recomendable utilizar el IMU para calcular el desplazamiento del robot. Lo mejor es solamente calcular su orientación de la siguiente manera:

$$\theta(t) = \theta_0 + \int_{0}^{t} \omega(t) \,dt$$

En donde:
* $\omega(t)$ es la velocidad angular en el instante $t$
* $\theta_0$ es la condición inicial

Para si implementación en código, se necesita discretizar la integral mediante integración de Euler:

$$\theta_{k} = \theta_{k-1} + (\omega_z \cdot \Delta t)$$

En dónde:
* $\theta_{k-1}$ es la orientación anterior a cada iteración
* $\Delta t$ es un intervalo de tiempo (usualmente el tiempo entre envío de mensajes del IMU)
* $\omega_z$ es la velocidad angular que vamos a considerar constante durante ese intervalo de tiempo $\Delta t$

Se utiliza el eje de rotacion $Z$ porque siguiendo la regla de la mano derecha, el eje $Z$ apunta hacia arriba del robot. Al girar ese eje, el robot cambia su orientación (yaw) y define su dirección de avance. 

```python
#self.theta = 0.0  
#self.last_time = None 

def imu_callback(self, msg):
    current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    if self.last_time is not None:
        dt = current_time - self.last_time
        
        omega_z = msg.angular_velocity.z
        
        self.theta = self.theta + omega_z * dt #Integración de Euler
        
    self.last_time = current_time
```

¿Por qué current_time es la sumatoria de dos campos?
El mensaje de tipo stamp dentro de algunas interfaces de ROS2 es un objetco con dos campos enteros:

header:

  stamp:
  
    sec: 53
    
    nanosec: 460000000

En donde:
* sec: segundos transcurridos
* nanosec: fracción de segundo restante del tiempo completo que no llega a ser un segundo entero ($10^{-9}$) segundos

El tiempo actual es la suma de esos dos campos convertidos en segundos. Si el IMU no proporciona el stamp con el tiempo por defecto, se le debe de agregar al código de la siguiente manera:

```python
from rclpy.clock import Clock

msg.header.stamp = self.get_clock().now().to_msg()
```


