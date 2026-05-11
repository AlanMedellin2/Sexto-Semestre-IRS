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
Ahora, hay que limpiar el ruido de la imagen que se genera naturalmente mediante un kernel (ventana) llena de unos. La función "cv2.morphologyEX" hace que el kernel recorra la imagen y vaya "encogiendo" los píxeles blancos. Si tras el recorrido algunas manchas sobreviven, regresan a su tamaño original. Sirve para eliminar ruido de brillo y para suavizar bordes.

```python
kernel = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
```

Después de limpiar la imagen con la morfología, ya no tenemos píxeles sueltos, sino "manchas" blancas sólidas. Ahora hay que identificar individualmente esas manchas y descartar aquellas que no tengan el tamaño suficiente para ser un semáforo. 

La función "cv2.findContours" "dibuja" el borde de cada mancha blanca que encuentra en la máscara mask. Se usasn parámetros para decirle a OpenCV que solo busque contornos externos (si hubiera una mancha blanca con un agujero negro en medio, ignoraría el agujero y solo tomaría el borde de afuera). Al final, la variable "contours" contiene una lista de todos los contornos encontrados. El guion bajo _ es una forma de ignorar un segundo valor que devuelve la función (la jerarquía), que no necesitamos aquí. Luego, por cada controno se calcula cuántos pixeles mide la superficie de esa mancha y se descartan con base en un threshold:

```python
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for cnt in contours:

area = cv2.contourArea(cnt)

if area < 800: # Ignorar ruidos pequeños

    continue
```

En este punto ya sabemos que el objeto tiene el color correcto y el tamaño adecuado, pero no sabemos si es un círculo que represente las luces de un semáforo. Para ello primero se calcula el permímetro con la función "cv2.arcLength" en donde el True indica que es un controno cerrado. Posteriormente, se calcula la circularidad:

$$C = \frac{4\pi \cdot A}{P^2}$$

En donde:
* $A$ es el área.
* $P$ es el perímetro.

```python
# --- FILTRO 1: CIRCULARIDAD ---

# Un círculo perfecto tiene circularidad = 1

perimeter = cv2.arcLength(cnt, True)

if perimeter == 0: continue

circularity = 4 * np.pi * (area / (perimeter * perimeter))
```

Ahora vamos a corroborar qué tan "estirado está ese cícrulo. La función "cv2.boundingRect" encierra el controno del círculo en un rectángulo imaginario en donde:
* x, y: Son las coordenadas de la esquina superior izquierda del rectángulo.
* w (width): El ancho del rectángulo en píxeles.
* h (height): El alto del rectángulo en píxeles.

Después se calcula la relación de aspecto dividiendo el ancho entre el alto. Si $w = h$ el resultado es 1.0 (un cuadrado perfecto). La circularidad debe de tener un valor mayor a 0.6 y la relación de aspecto mayor a 0.7. Si esas dos condiciones se cumplen, entonces hemos detectado una luz de semáforo. 

```python
# --- FILTRO 2: RELACIÓN DE ASPECTO ---

x, y, w, h = cv2.boundingRect(cnt)

aspect_ratio = float(w)/h

if 0.6 < circularity < 1.2 and 0.7 < aspect_ratio < 1.3:

    total_objects += 1
```

Para mostrar la detección en pantalla se utilizan las siguientes funciones:
* cv2.rectangle(): dibuja un cuadro alrededor del objeto detectado Usando las coordenadas de la equina superiror izquierda y la inferior derecha. EL número 2 es el grosor de las líneas
* cv2.putText(): esta función escribe un texto encima del cuadro para indicar qué color se está detectando.

```python
# Dibujar detección

cv2.rectangle(frame, (x, y), (x + w, y + h), self.draw_colors[color_name], 2)

cv2.putText(frame, f"Semaforo: {color_name}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.draw_colors[color_name], 2)
```

Finalmente, se publica un número por el tópico /color dependiendo del color detectado:
* Amarillo: 1.0
* Verde: 2.0
* Rojo: 3.0
* Otro: 0.0


