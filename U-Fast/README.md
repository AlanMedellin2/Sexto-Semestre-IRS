# Ultra Fast Structure-aware Deep Lane Detection

Los detectores de carriles han tenido un gran auge en los últimos años, especialmente por su importancia en sistemas de asistencia al conductor, etc. El paper menciona que existen dos enfoques principales para resolver este problema: los métodos tradicionales de procesamiento de imágenes y los métodos basados en aprendizaje profundo, especialmente aquellos que utilizan segmentación semántica.

Los métodos tradicionales suelen apoyarse en características visuales como color, bordes, contraste o transformadas geométricas. Por otro lado, los métodos de segmentación profunda han ganado mayor relevancia porque tienen una mejor capacidad para aprender representaciones complejas de la escena. 

Esta tecnología se utiliza constantemente en aplicaciones donde el tiempo de respuesta es crítico. Por eso, la detección de carriles necesita trabajar con bajo costo computacional, sobre todo cuando se procesan imágenes provenientes de una o varias cámaras en tiempo real. En este contexto, un pipeline rápido es esencial para lane detection.

Además de la velocidad, también se debe considerar el problema conocido como **no-visual-clue**. Este ocurre cuando no existen pistas visuales claras del carril, por ejemplo, cuando la línea está tapada por un vehículo, hay sombras, iluminación extrema, desgaste o condiciones de baja visibilidad. En estos casos, el modelo no puede depender únicamente de la información local de los pixeles, sino que necesita utilizar el contexto completo de la imagen.

U-Fast compite con otros modelos como SAD (Self Attention Distillation). SAD busca mejorar la velocidad mediante destilación de atención, pero sigue estando basado en una formulación de segmentación. Por esa razón, aunque es más eficiente que otros métodos previos, todavía puede mantener un costo computacional considerable en comparación con U-Fast.

Otro punto importante es que muchos modelos basados en segmentación representan los carriles como mapas binarios de pixeles, pero no aprovechan directamente propiedades estructurales de los carriles, como su continuidad, suavidad y forma. 

**U-Fast propone seleccionar puntos concretos de los carriles en filas predefinidas de la imagen utilizando características globales, en lugar de segmentar pixel por pixel basándose únicamente en información local.**

Básicamente, el método cambia la forma de abordar el problema. En lugar de analizar pixel x pixel, utiliza información global de la imagen y selecciona la ubicación del carril en filas horizontales específicas. Esto permite que el modelo tenga una perspectiva más completa de la escena y pueda enfrentar mejor situaciones difíciles, como sombras, oclusiones o líneas parcialmente invisibles.

<img width="730" height="203" alt="image" src="https://github.com/user-attachments/assets/76803215-d4c6-4deb-9161-74728413b41f" />

Fig. 1. Ilustración de la selección en los carriles izquierdo y derecho. En la parte derecha se muestra con detalle la selección dentro de una fila. Los **row anchors** son ubicaciones horizontales predefinidas en la imagen, y la formulación del método consiste en seleccionar una posición horizontal sobre cada una de esas filas. A la derecha de la imagen se introduce una celda adicional de fondo, utilizada para indicar que no existe carril en esa fila.

![Demo animada](https://miro.medium.com/v2/resize\:fit:720/format\:webp/1*CcUhYgdigwMkJ8ZsycolMA.gif)

El primer paso del método consiste en dividir la imagen usando una cuadrícula. Para cada row anchor, la ubicación horizontal se divide en varias celdas. De esta manera, la detección de carriles se puede describir como un problema de selección: el modelo debe elegir qué celda corresponde a la posición del carril en cada fila predefinida.

El proceso general puede resumirse de la siguiente manera:

1. Se definen varias filas horizontales sobre la imagen, llamadas **row anchors**.
2. En cada fila, el ancho de la imagen se divide en celdas horizontales.
3. Para cada carril, el modelo selecciona en qué celda se encuentra la línea.
4. Si no existe carril en una fila determinada, el modelo puede seleccionar la celda de fondo.

**Row-based selecting** significa seleccionar una posición horizontal dentro de cada fila. En lugar de generar una máscara completa de pixeles, el modelo produce una serie de posiciones que, al conectarse, representan la forma del carril.

<img width="539" height="334" alt="Screenshot from 2026-05-31 02-14-33" src="https://github.com/user-attachments/assets/228c57b3-6208-41fd-a9b0-4e3c4d08ad70" />

## Notación principal

* **H** — Altura de la imagen.
* **W** — Ancho de la imagen.
* **h** — Número de row anchors.
* **w** — Número de gridding cells.
* **C** — Número máximo de carriles.
* **X** — Características globales de la imagen.
* **f** — Clasificador utilizado para seleccionar las ubicaciones de los carriles.
* **P ∈ R^(C×h×(w+1))** — Predicciones del modelo.
* **T ∈ R^(C×h×(w+1))** — Etiquetas objetivo.
* **Prob ∈ R^(C×h×w)** — Probabilidad de cada ubicación.
* **Loc ∈ R^(C×h)** — Ubicaciones estimadas de los carriles.

<img width="355" height="41" alt="image" src="https://github.com/user-attachments/assets/37b8ca42-376d-4dda-86c6-d15401f8596f" />

La fórmula anterior indica que, para el carril **i** y la fila **j**, el clasificador **f** toma como entrada las características globales de la imagen, representadas por **X**, y genera una predicción **P**. Esta predicción es un vector que contiene la probabilidad de que el carril esté en cada celda horizontal de esa fila.

En otras palabras, el modelo responde la siguiente pregunta: “para este carril y esta fila, ¿en qué posición horizontal se encuentra la línea?”. Si la fila se divide en varias celdas, el modelo asigna una probabilidad a cada una de ellas y selecciona la más probable.

## Función de pérdida de clasificación

<img width="352" height="69" alt="image" src="https://github.com/user-attachments/assets/5979ae4f-82e2-442e-85a7-affe6d36ef3d" />

Esta fórmula representa la pérdida de clasificación. El modelo compara su predicción con la etiqueta correcta para cada carril y para cada fila. La función **LCE** representa la **cross entropy loss**, que mide qué tan diferente es la predicción del modelo respecto a la respuesta real.

En esta formulación, **T** representa la etiqueta correcta. Por ejemplo, si el carril realmente se encuentra en una celda específica, esa celda será marcada como la respuesta verdadera. Si el modelo asigna alta probabilidad a esa celda, la pérdida será baja. Si asigna mayor probabilidad a una celda incorrecta, la pérdida será más alta.

<img width="671" height="233" alt="image" src="https://github.com/user-attachments/assets/e387dcc7-c8b5-4c66-baf4-a0dccce8be24" />

## Comparación con segmentación tradicional

En la segmentación tradicional, el modelo clasifica cada pixel de la imagen. Si una imagen tiene altura **H** y ancho **W**, el costo computacional está relacionado con **H × W × (C + 1)**, porque se analiza cada pixel y se decide si pertenece a algún carril o al fondo.

En cambio, en U-Fast el costo está relacionado con **C × h × (w + 1)**, porque solo se hacen predicciones para un número limitado de carriles, filas y celdas. Esto reduce considerablemente el número de clasificaciones necesarias.

La razón por la que este método es más eficiente es que normalmente el número de row anchors **h** es mucho menor que la altura total **H** de la imagen, y el número de gridding cells **w** también es menor que el ancho total **W**. Por eso, U-Fast puede ser mucho más rápido que una segmentación completa.

El paper muestra que, usando configuraciones comunes del dataset CULane, la segmentación tradicional requiere aproximadamente **1.15 × 10^6** cálculos, mientras que la formulación propuesta requiere aproximadamente **1.7 × 10^4** cálculos. Esta diferencia explica por qué el método puede alcanzar velocidades muy altas.

## Importancia del número de celdas

El número de gridding cells debe elegirse cuidadosamente. Usar pocas celdas facilita la clasificación porque el modelo tiene menos opciones para elegir, pero reduce la precisión espacial, ya que cada celda cubre una región más grande de la imagen.

Por otro lado, usar demasiadas celdas mejora la resolución espacial, pero hace que la clasificación sea más difícil, porque el modelo debe escoger entre muchas más posiciones posibles. Por eso, existe un equilibrio entre precisión y dificultad de aprendizaje.

En el paper, los autores prueban diferentes cantidades de celdas y concluyen que **100 celdas** funcionan bien para el dataset TuSimple, ya que ofrecen un buen balance entre precisión de localización y facilidad de clasificación.

## Relación con el problema no-visual-clue

Una ventaja importante de U-Fast es que utiliza características globales de la imagen. Esto significa que el modelo no depende únicamente de la información visual localizada justo encima del carril. En situaciones donde una línea está oculta o degradada, el modelo puede utilizar otras pistas de la escena, como la dirección del camino, la posición de otros carriles o la geometría general de la carretera.

Esto ayuda a resolver el problema de **no-visual-clue**, ya que el modelo puede inferir la posición del carril aunque no todas sus partes sean visibles.
