# Detección de línea

El objetivo de este nodo es detectar la línea central del carril del mapa del puzzlebot con el fin de establecer un path de seguimiento. Para ello se utilizan dos librerías:
* cv2: Librería de OpenCV
* numpy: Librería para manejo de datos y matrices

### Extracción de la imagen

El primer paso es establecer el puerto por el que se va a leer la información que recopile la cámara. De igual manera, se crea un objeto "cap" de la clase cv2.VideoCapture que como tal no almacena imágenes fijas, si no que es un objeto que tiene acceso a métodos de la clase para obtener datos y realizar acciones con la cámara. Entre esos métodos se pueden tomar fotos, verificar conexión con la cámara, etc... :

```python
video_path = "/dev/video0"

cap = cv.VideoCapture(video_path)
```

Después se establecen rangos de área en los que una máscara binarizada es capaz de decidir si las manchas con cierta área dentor de este umbral se considera una línea o no:

```python
Area_min = 200   #Píxeles
Area_max = 20000 
```
La función "cap.read()" devuelve una tupla con:
* ret: un valor booleano que en caso de ser True, significa que la cámara está funcionando bien y logró capturar una imagen con éxito.
* originalFr: matriz numérica (es un arreglo de NumPy) que contiene los colores de cada píxel en formato BGR de la imagen en ese momento.

```python
ret, originalFr = cap.read()  
```

Se extrae la altura "h" y el ancho "w" de la imagen capturada con la función ".shape[:2]" de NumPy. Esta función, sin el parámetro ":2" devuelve tres valores (alto, ancho, canales). el "[:2]" hace un corte para que solo devuelva los primeros dos valores y conocer la dimensión de la imagen:

```python
h, w = originalFr.shape[:2] 
```

### Cálculo de ROI

ROI significa Region of Interest (Región de Interés). En lugar de procesar toda la imagen, te quedas solo con una parte en la que sabes que está tu objeto de interés. EN este caso, sabemos que el suelo es en dónde se va a encontrar la línea a detectar.

En nuetsro código simplemente se corta la imagen asiganndo una variable "roi" que contenga los píxeles de la imagen original pero empezando desde píxel que represente el 60% de arriba hacia abajo de la altura total y que recorra todas la filas restantes:

```python
roi = originalFr[int(h*0.6):h, :] #60% de altura, : --> todas las columnas
```

Volvemos a obtener la altura y ancho pero de ese nuevo corte:

```python
roi_h, roi_w = roi.shape[:2]
```

### Escala de grises

Se utiliza la función "cv.cvtColor(imagen de entrada, tipo de color)" para transformar la imagen en formato BGR a tonos de grises. 

¿Cómo se convierte una imagen a tonos de grises?

Para hacerlo, se debe de calcular la luminancia:

$$Y = 0.299 \cdot R + 0.587 \cdot G + 0.114 \cdot B$$

Los coeficientes que multiplicana a cada color del formato BGR, sumados, nos dan 1. Básicamente son un porcentaje de sensibilidad que tiene cada uno en el ojo del ser humano:
* Verde ($58.7\%$): Es el que más aporta. El ojo humano es mucho más sensible a los tonos verdes; podemos distinguir más sombras de verde que de cualquier otro color.
* Rojo ($29.9\%$): Tiene un aporte medio.
* Azul ($11.4\%$): Es el que menos "brilla" para nosotros. Por eso tiene el coeficiente más bajo.

De esta manera, nuestro corte pasa de ser una matriz en donde cada celda tiene 3 valores BGR a una matriz en donde cada celda tiene un valor de luminancia. Por ello, el brillo afecta estos valores:

```python
#convertir de BGR a gris
gris_image = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
```

### Filtro Gaussiano

Su objetivo es eliminar el ruido (esos granitos de sal y pimienta que aparecen en la cámara) para que, cuando busquemos la línea, el algoritmo no se confunda con pequeños puntos brillantes u oscuros en el suelo.

Se utiliza un kenerl (ventana de nxn píxeles) que recorre toda la imagen y en cada iteración realiza:
1) Superposición: El centro del kernel se coloca sobre un píxel.
2) Multiplicación: Se multiplica el valor de cada píxel de la imagen por el peso que le corresponde en el kernel gaussiano. Para aclcular los pesos se utiliza la siguiente fórmula:

$$G(x, y) = \frac{1}{2\pi\sigma^2} e^{-\frac{x^2 + y^2}{2\sigma^2}}$$

Donde:
* $(x, y)$: Es la distancia desde el centro del kernel. El centro es $(0,0)$.
* $\sigma$ (Sigma): Es la desviación estándar. Controla qué tan "ancho" es el filtro y se elige como parámetro.
* $e$: Es la constante de Euler ($\approx 2.718$).

Para un kernel de $3 \times 3$, las coordenadas de cada celda respecto al centro $(0,0)$ son:

$$\begin{bmatrix}
(-1, 1) & (0, 1) & (1, 1) \\
(-1, 0) & \mathbf{(0, 0)} & (1, 0) \\
(-1, -1) & (0, -1) & (1, -1)
\end{bmatrix}$$

Calculamos los pesos de cada coordenada reemplazando los valores de $x,y$ en la ecuación considerando $\sigma = 1$:

* $$G(0,0) = \frac{1}{2\pi(1)^2} e^{-\frac{0^2 + 0^2}{2(1)^2}} \approx 0.1591$$
* $$G(0,1) = \frac{1}{2\pi} e^{-\frac{0^2 + 1^2}{2}}  \approx 0.0965$$
* $$G(0,-1) = \frac{1}{2\pi} e^{-\frac{0^2 + (-1)^2}{2}} \approx 0.0965$$
* $$G(-1,1) = \frac{1}{2\pi} e^{-\frac{(-1)^2 + 1^2}{2}} \approx 0.0585$$
* $$G(-1,0) = \frac{1}{2\pi} e^{-\frac{(-1)^2 + 0^2}{2}} \approx 0.0965$$
* $$G(-1,-1) = \frac{1}{2\pi} e^{-\frac{(-1)^2 + (-1)^2}{2}} \approx 0.0585$$
* $$G(1,1) = \frac{1}{2\pi} e^{-\frac{1^2 + 1^2}{2}} \approx 0.0585$$
* $$G(1,0) = \frac{1}{2\pi} e^{-\frac{1^2 + 0^2}{2}}  \approx 0.0965$$
* $$G(1,-1) = \frac{1}{2\pi} e^{-\frac{1^2 + (-1)^2}{2}} \approx 0.0585$$

Lo que nos da:

$$W = \begin{bmatrix} 
0.0585 & 0.0965 & 0.0585 \\ 
0.0965 & \mathbf{0.1591} & 0.0965 \\ 
0.0585 & 0.0965 & 0.0585 
\end{bmatrix}$$

Ahora vamos a normalizar esa matriz. Para ello, se deben de sumar los nxn valores (en nuetsro caso 9). Esto nos da: $0.7791$ y se divide cada valor de la matriz entre ese número. Cömo resultado obtenemos:

$$W = \begin{bmatrix} 
0.075 & 0.124 & 0.075 \\ 
0.124 & 0.204 & 0.124 \\ 
0.075 & 0.124 & 0.075 
\end{bmatrix}$$

Y multiplicamos la ventana de Luminancia $L$ por la ventana de pesos $W$:

Suponiendo que L:

$$L = \begin{bmatrix} 
100 & 102 & 100 \\ 
98 & 200 & 105 \\ 
101 & 103 & 99 
\end{bmatrix}$$

La operación es:

$$\begin{bmatrix} 
100 \cdot 0.075 & 102 \cdot 0.124 & 100 \cdot 0.075 \\ 
98 \cdot 0.124 & 200 \cdot 0.204 & 105 \cdot 0.124 \\ 
101 \cdot 0.075 & 103 \cdot 0.124 & 99 \cdot 0.075 
\end{bmatrix} = \begin{bmatrix} 
7.5 & 12.64 & 7.5 \\ 
12.15 & 40.8 & 13.02 \\ 
7.57 & 12.77 & 7.42 
\end{bmatrix}$$

3) Ahora sumamos todos esos valores $= 121.37 = 121$
4) Reemplazamos el píxel central por ese nuevo valor. Antes era $200$ y ahora con el filtro es $121$

Este filtro se realiza con la siguiente función en dónde se indica el tamaño del kernel y la desviación estándar:

```python
#Suavizamos y usamos gaussiano para reducir el ruido antes de analizar la imagen
blurred = cv.GaussianBlur(gris_image, (5, 5), 0)
```

### Binarización

El objetivo es separar la línea a seguir del suelo den la imagen. En una imagen binaria solo existen dos estados: 0 (negro) y 255 (blanco).
La función "cv.threshold(imagen de entrada, punto de corte, valor máximo de intensidad)" devuelve el umbral usado (en este caso se coloca un _ para indicar que no nos interesa que lo devuelva) y también devuelve una matriz que solo contiene píxeles binarizados. En este caso, "80" es el punto de corte. SIgnifica que si un píxel tiene un valor de luminancia menor a 80, se csonidera oscuro y su valor pasará a ser de 255 (blanco) para resaltar los colores negros:

```python
_, binary = cv.threshold(blurred,80, 255, cv.THRESH_BINARY_INV)
```

### Máscara trapezoidal

El carril a detectar, por la perspectiva de la cámara, se ve como un trapecio geométricamente. Para facilitar su detección, se añade un filtro geométrico que distinga carriles y facilite el seguimiento de la línea. Primero se definen el ancho de la base superior del trapecio (60%) de nuestro corte ROI y se define dónde empieza el trapecio desde arriba (30%):

```python
top_width = int(roi_w * 0.6)
top_y = int(roi_h * 0.3)
```

Luego, se crea un arreglo con las 4 esquinas que forman la figura cada uno como una coordenada (x,y):

```python
trapezoid = np.array([[

  ((roi_w - top_width) // 2, top_y),

  ((roi_w + top_width) // 2, top_y),

  (roi_w, roi_h),

  (0, roi_h)

  ]], dtype=np.int32)
```

Después se crea una máscara completamente negra (llena de ceros) y con la función "cv.fillPoly" se dibuja ese trapecio en la máscara rellenado de color blanco:

```python
mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
cv.fillPoly(mask, trapezoid, 255)
```

Por último, se compara la imagen binarizada con la máscara del trapecio píxel por pixel con una operación AND en dónde sólo los pixeles que coincidan dentro del trapesio se pasan tal cual y como venían en la imagen. Los demás se quedan en negro. Se pasa "binary" dos veces como parámetro de la función "cv.bitwise_and()" solo por convención de OpenCV:

```python
binary_masked = cv.bitwise_and(binary, binary, mask=mask)
```

### Limpieza morfológica

Limpia detalles finales en la imagen del trapecio. Primero se crea un kernel. Luego, se aplica una función de erosión "cv.erode()" el cual, si todos los píxeles que están dentro del kernel son blancos, el píxel central se queda blanco. SI alguno es negro, el centro se vuelve negro. Sirve para lijar los bordes. Luego haces el proceso opuesto con dilatación "cv.dilate()" en dónde si un píxel dentro de la ventana es blanco, el centro será blanco.

Se hace primero la erosión para eliminar ruido y la dilatación va después para regresar los bordes al estado original pero con el ruido eliminado. 

```python
#Morphological operations
kernel = np.ones((3,3), np.uint8)

morph = cv.erode(binary_masked, kernel, iterations=1)

morph = cv.dilate(morph, kernel, iterations=1)
```

