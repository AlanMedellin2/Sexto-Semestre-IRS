# ColorDetectorNode


## Explicación del código:

La librería que hace posible analizar las imágenes de la grabación es:

```python
import cv2
```
Para abrir la cámara web (ínidce 0). Para abrir la cámara Logi de la Rubik (índice 2):

```python
self.cap = cv2.VideoCapture(0)
```
Para configurar la resolución de la imagen, se utiliza el método "cap", el cuál cambia una propiedad de captura de video. Se utilizan los argumentos "CAP_PROP_FRAME_WIDTH, #number_size" para definir el ancho o alto de la imagen en píxeles. A menor reolución, mayor rapidez de procesamiento de datos: 

```python
self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```
Se establecen rangos mínimos y máximos para los tres colores que buscamos (verde, rojo, amarillo) en formato HSV. Sus siglas significan:
* H (Hue/Matiz): Es el valor del color en sí. Se mide en grados de un círculo (0° - 360°). ¿Cómo se obtiene?
* S (Saturación): Define qué tan puro o gris es un color (o es gris, 255 es el color vibrante). ¿Cómo se obtiene?
* V (Valor): El brillo (0 es negro total y 255 es mucha luz). ¿Cómo se obtiene?

La razón por la que conviene usar HSV en vez de RGB es porque si la luz cambia, los valores de RGB cambian drásticamente. En HSV, si la luz cambia, mayormente solo afceta a V, mientras que H se mantiene estable. 

En el código se crea un diccionario llamado "self.color_ranges" en dónde cada color tiene una lista de tuplas. Cada tupla es un máximo o mínimo que juntas para establecer el rango en el que ciertos valores de HSV se consideran un color u otro. EL rojo tiene dos rangos debido a que se encuentra en el corte de los 0° y 360°. Esto provoca que clasifique al rojo en rangos que tiran hacia el naranja y hacia el púrpura:

```python
self.color_ranges = {
    "Amarillo": [(np.array([20, 100, 150]), np.array([35, 255, 255]))],
    "Verde": [(np.array([40, 70, 70]), np.array([90, 255, 255]))],
    "Rojo": [
        (np.array([0, 150, 100]), np.array([10, 255, 255])),
        (np.array([160, 150, 100]), np.array([180, 255, 255]))
    ]
}
```
En la función main, el primer paso es es capturar una imagen de la cámara en ese momento. El método "cap.read" devuelve una tupla (ret, frame) en donde "ret" es un valor booleano que en caso de ser True, indica que la cámara está funcionando correctamente y se pudo leer un cuadro de video sin problemas. "frame" es una matriz de toipo "numpy.ndarray" que contiene los píxeles de la imagen en formato BGR (Blue, Green, Red):

```python
ret, frame = self.cap.read()
```

Después, se convierte el formato BGR del frame a HSV con la función "cv2.cvtColor(<frame>,<formato>)". Esta función aplica una serie de fóromulas matemáticas a cada pixeld e la matriz para transformar las coordenadas de un espacio de color a otro. Los pasos son:
1) Normalización: Divide cada elemento B, G, y R entre 255 para obtener una escala al rango [0,1]:

   $$R' = R/255, \quad G' = G/255, \quad B' = B/255$$
   
2) Cáculo de V: es el máximo de entre los tres valores previamente normalizados
3) Cáculo de $\Delta$: Es la diferencia entre el máximo y el mínimo
4) Cálculo de S: $S = \frac{\Delta}{max}$ si $max$ es diferente de 0. Si $max = 0$, $S = 0$
5) Cáculo de H:
   * Si $\max = R'$, entonces $H = 60^\circ \times \left( \frac{G' - B'}{\Delta} \pmod 6 \right)$
   * Si $\max = G'$, entonces $H = 60^\circ \times \left( \frac{B' - R'}{\Delta} + 2 \right)$
   * Si $\max = B'$, entonces $H = 60^\circ \times \left( \frac{R' - G'}{\Delta} + 4 \right)$
6) Ajustes finales:
   * H: El resultado en grados se divide entre 2 (0° - 179°) para mantener el límite dentro de un byte
   * S y V: Se multiplican por 255 para volver al rango 0 - 255

El resultado es una variable "hsv" que es una matriz de alto x ancho en dónde cada celda contiene los tres valores HSV de cada píxel:

```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
```

Una imagen a color tiene tres dimensiones (alto, ancho, canales). Nuestro frame, al ser una matriz de numpy, tiene un método para obtener esas carcaterísticas:

```python
frame_h, frame_w = frame.shape[:2]
```

El siguiente paso es recorrer nuestro diccionario en dónde en cada iteración se crea una máscara vacía (rellenada con ceros) con la resolución correspondeinte:

```python
mask = np.zeros((frame_h, frame_w), dtype=np.uint8)
```
Después, se recorre cada rango de cada color, Se escanea cada toda la matriz hsv y si un pixel está dentro de los límites "lower" y "uper", se le aisgna un valor de 255 (blanco). Si no, se le asigna un 0 (negro). El resultado es una máscara parcial que resalta las zonas que coinciden con ese color. Se hace mediante la función:

```python
partial_mask = cv2.inRange(hsv, lower, upper)
```

Luego luego se hace una operación OR entre la máscara completamente negra que definimos al inicio de la iteración con la máscara parcial para actualizarla. EN el caso de los colores amarillo y verde, como solo tienen un rango, no tiene mucho sentido hacer eso. SOlo habría que tomar la máscara parcial y ya. Pero como el rojo tiene dos rangos, esta operación se hace dos veces para actualizar el valor entre los dos rangos que tiene:

```python
mask = cv2.bitwise_or(mask, partial_mask)
```
Ahora, hay que limpiar el ruido de la imagen que se genera naturalmente mediante un kernel (ventana) llena de unos:

```python
mask = cv2.bitwise_or(mask, partial_mask)
```

 
