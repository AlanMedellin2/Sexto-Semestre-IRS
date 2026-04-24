# La Rubik tiene el siguiente PINOUT:

<img width="1168" height="769" alt="image" src="https://github.com/user-attachments/assets/87f9db85-c0bb-4bac-87a5-92728bd961fd" />

<img width="1218" height="631" alt="image" src="https://github.com/user-attachments/assets/96720c60-7a10-4373-8633-5d56bbc43fd0" />

# El acelerómetro tiene los siguiente ejes de sensibilidad:
<img width="667" height="286" alt="image" src="https://github.com/user-attachments/assets/0f7384d5-8328-401d-af2a-e418649fd15b" />

## Características:

### Gyroscope Features
The triple-axis MEMS gyroscope in the MPU-60X0 includes a wide range of features:
 Digital-output X-, Y-, and Z-Axis angular rate sensors (gyroscopes) with a user-programmable fullscale range of ±250, ±500, ±1000, and ±2000°/sec: Nos da la velocidad de rotación en grados por segundo. EL rango programable ajusta la resolución.
 External sync signal connected to the FSYNC pin supports image, video and GPS synchronization: útil si hacemos Visual Odometry con una cámara
 Integrated 16-bit ADCs enable simultaneous sampling of gyros
 Improved low-frequency noise performance: Ayuda a que las vibraciones del robot no afecten las mediciones
 Digitally-programmable low-pass filter
 Gyroscope operating current: 3.6mA
 Standby current: 5µA

### Accelerometer Features
The triple-axis MEMS accelerometer in MPU-60X0 includes a wide range of features:
 Digital-output triple-axis accelerometer with a programmable full scale range of ±2g, ±4g, ±8g and
±16g: Solo sirve para detectar si el robot está inclinado
 Integrated 16-bit ADCs enable simultaneous sampling of accelerometers while requiring no external
multiplexer: útil para implementar Filtro de Kalman
 Accelerometer normal operating current: 500µA

### Objetivo: El acelerómetro es perfecto para calcular el heading ($\theta$) de nuestro robot. Los encoders se encargarán de calcular el desplazamiento. 

### Voltaje de funcionamiento: 3.3 V a 5 V.

### Sus pines son:
<img width="800" height="800" alt="image" src="https://github.com/user-attachments/assets/872e4994-f423-46b5-8594-9560ea3d1701" />



