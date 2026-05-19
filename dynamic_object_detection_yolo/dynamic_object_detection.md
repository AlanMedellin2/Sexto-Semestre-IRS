# Dynamic Object Detection documentation

El objetivo es implementar un algortimo de detección de objetos dinámicos en un video en movimeinto utilizando YOLO y que devuelva una imagen segmentada que elimine dichos objetos de la misma.

Se utilizan las librerías:

```python
import cv2     #OpenCV
import numpy as np     #manejo de matrices
import torch     #motor que corre la red neuronal
from ultralitycs import YOLO     #librería que contiene los modelos de YOLO ya entrenados
```

Se optó por usar un modelo de YOLO ya entrenado (YOLO v8) con su versión Nano de segmentación. Mas que nada por el tiempo que se tarda en procesar el modelo de clasificación en cada iteración del algortimo.
De igual manera, la versión Nano de YOLO v8 detecta clases básicas con bastante precisión. La mayoría de objetos dinámicos que se va a encontrar el SDV en el campus serán personas, entonces es una gran opción para empezar a experimentar. la función ".to(device)" solo le indica a mi GPU que se va a encargar de correr el modelo:

```python
model = YOLO('yolov8n-seg.pt').to(device)
```

Se utilizó un dataset llamado Common Objects in COntext (COCO), el cual contiene cerca de 80 categorías de objetos ya entrenadas. Sus índices principales son:
* 0: Personas
* 1: Bicicletas
* 2: Autos
* 3: Motocicletas
* 4: Aviones
* 5: Autobuses
* 6: Trenes
* 7: Camiones

```python
DYNAMIC_CLASSES = [0, 1, 2, 3, 5, 7]
```

El modelo tiene dos funciones disponibles:
1) "model.predict()": Detecta una clase, pero no sabe distinguir entre dos objetos de la misma clase.
2) "model.track()": Le asigna un ID único a cada objeto incluyendo los de la misma clase

Se le pasan los parámetros del frame a analizar ("frame"), si deseamos mantener el rastro ("persist"), el umbral de confianza como filtro de seguridad (40%). Nos devuelve una lista ("results") que contiene las coordenadas de cada caja detectada, polígonos de las máscaras, y números de clases.

```python
results = model.track(frame, persist=True, device=0, conf=0.4, verbose=False)
```

Para acceder a ellos se hace mediante el índice [0] ya que es el único frame en ese momento que está siendo analizado. Luego se obtienen las coordenadas de los puntos de los cuadros que rodean un objeto y se guardan en una lista de arreglos llamada "mask":

```python
masks = results[0].masks.xy
```

Luego convertimos la lista de ID's asignados por el modelo a cad objeto en una lista de python con el siguiente comando:

```python
clss = results[0].boxes.cls.cpu().int().tolist()
```

Se recorre un conjunto de datos (los boxes, los ID) cada uno matcheado con su par mediante la función "zip()" en donde si el ID corresponde a una clase a buscar, se rellena su caja de color blanco (255):

```python
for mask_poly, cls in zip(masks, clss):

  if cls in DYNAMIC_CLASSES:
  
    # Rellenar la silueta en la máscara con blanco
    
    points = np.int32([mask_poly])
    
    cv2.fillPoly(slam_mask, points, 255)
```

Posteriormente, se calcula el centroide del objeto utilizando los momentos aplicados en esa caja.

Los momentos de una imagen son promedios ponderados de las intenisdades de los pixeles. Para una imagen binarizada (píxeles 0 y 1), el momento de orden $(i, j)$ se define como:

$$M_{ij} = \sum_{x,y} x^i y^j I(x,y)$$

Esta fórmula es la base de la Estadística Espacial en imágenes ya que describe la distribución de los píxeles blancos en un plano cartesiano. Sus componentes son:
* $\sum_{x,y}$: Significa que vamos a recorrer cada píxel de la imagen (o del ROI), fila por fila y columna por columna.
* $x^i y^j$: Son las coordenadas del píxel elevadas a una potencia. Estas potencias ($i$ y $j$) determinan qué característica estamos calculando (el "orden" del momento).
* $I(x,y)$: Es la Intensidad del píxel en esa posición.

¿Qué significan los órdenes del momento?

1) Si ponemos $i=0$ y $j=0$, obtenemos el área:

$$M_{00} = \sum_{x,y} x^0 y^0 I(x,y) = \sum_{x,y} 1 \cdot 1 \cdot I(x,y)$$

2) Si ponemos $i=01$ y $j=0$ o viceversa (momentos de primer orden), obtenemos la posición en cada coordenada:

* $M_{10}$ ($i=1, j=0$): Suma las coordenadas $x$ de todos los píxeles blancos ($\sum x \cdot I(x,y)$).
* $M_{01}$ ($i=0, j=1$): Suma las coordenadas $y$ de todos los píxeles blancos ($\sum y \cdot I(x,y)$).

A partir de aquí se puede calcular el centroide (centro de masa) con los momentos de grado 0 y 1 con las siguiente fórmulas:

$$\bar{x} = \frac{M_{10}}{M_{00}} \quad , \quad \bar{y} = \frac{M_{01}}{M_{00}}$$

Cada una describe la posición del centroide en su eje respectivo. 

```python
M = cv2.moments(points)
if M['m00'] != 0:
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    cv2.circle(frame, (cx, cy), 12, (0, 255, 255), 2) 
    cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1) 
```

El siguiente paso es dilatar los bordes de la silueta con un filtro morfológico para no dejar pasar detalles en la figura. Primero se crea un kernel. Luego, se aplica una función de dilatación"cv.dilate()" el cual, si todos los píxeles que están dentro del kernel son negros, el píxel central se queda negro. Si alguno es blanco, el centro se vuelve blanco. 

```python
kernel = np.ones((10, 10), np.uint8)
slam_mask = cv2.dilate(slam_mask, kernel, iterations=1)
```

Finalmente invertimos la máscara binaria para que la silueta se transforme de color negro y aplicamos una máscara AND con el frame original para eliminar las siluetas dinámicas de nuestra imagen:

```python
# Invertimos máscara y aplicamos AND
inv_mask = cv2.bitwise_not(slam_mask)
frame_for_slam = cv2.bitwise_and(frame, frame, mask=inv_mask)
```

Resultados:



https://github.com/user-attachments/assets/bb815a1a-37c8-4782-9954-731527cdf86e




https://github.com/user-attachments/assets/3f420a29-5108-4739-86eb-537afd04fa18




https://github.com/user-attachments/assets/3d94c6c6-360d-4c5a-a1ff-42792c134cf0




https://github.com/user-attachments/assets/2a45bc3c-b91e-4c94-abcf-9b10c8c420f5




