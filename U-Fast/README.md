Los detectores de línea han tenido un gran auge en esttos últimos años, el paper menciona que existen dos métodos principales, donde tenemos el procesamiento de imagen tradicional y métodos de segmentación profunda. Este último teniendo más peso en la actualidad por su gran representación y habilidad para aprender.

Esta tecnología que contantemente es usada en la industría, necesita estar de la mano de un bajo costo computacional, también considerando que en estos sistemas se requiere de datos de entrada de la cámara o cámaras que pueden llegar a ser pesados en especal si se busca analizar el panorama en tiempo real, por lo que un pipeline rápido es escencial en lane detection. también se debe considerar el porblema de no-visual-clue, donde no hay pistas visuales claras del carril, lo que puede dificultar la lectura de las líneas.

U-Fast puede competir contra otros modélos como SAD, el cuál, trabaja como un método que busca veocidad pero al estar basado en segmentación, puede aumentar su costo computacional.

Tmabién algunos modelos no consideran la rigides y la suavidad de los carriles (investigar bien eso)

**U-Fast propone seleccionar puntos concretos de los carriles en filas predefinidas de la imagen utilizando características globales en ves de segmentar pixel x pixel de los carriles basándose en un campo receptivo local**

Basicamente lo que tratamos aqui es en ves de la vision tradicional que suele se local, nos pasamos a la global ayudando a tener una perspectiva más copleja de toda la imagen donde puede aprender a utilizar todos los porblemas y utilizarlos, resolvendo no-visual-clue.

<img width="730" height="203" alt="image" src="https://github.com/user-attachments/assets/76803215-d4c6-4deb-9161-74728413b41f" />

Fig. 1. Ilustración de la selección en los carriles izquierdo y derecho. En la parte derecha se muestra en detalle la selección de una fila. Los puntos de anclaje de fila son las ubicaciones predefinidas de las filas, y nuestra formulación se define como la selección horizontal en cada uno de los puntos de anclaje de fila. A la derecha de la imagen, se introduce una celda de cuadrícula de fondo para indicar que no hay ningún carril en esta fila

![Demo animada](https://miro.medium.com/v2/resize:fit:720/format:webp/1*CcUhYgdigwMkJ8ZsycolMA.gif)

Se menciona que el primer paso es la división de cuadriculas, en cada punto de referencia de fila, la ubicación se divide en varias celdas. De este modo, la detección de carriles puede describirse como la selección de determinadas celdas sobre puntos de referencia de fila predefinidos.

Entonces contamos con este orden:
1. define varias filas horizontales sobre la imagen (row anchors)
2. En cada fila, divide la imagen en celdas horizontales
3. Para cada carril, el modelo escoge en qué celda está

Row-based selecting significa seleccionar una posición horizontal dentro de cada fila

<img width="539" height="334" alt="Screenshot from 2026-05-31 02-14-33" src="https://github.com/user-attachments/assets/228c57b3-6208-41fd-a9b0-4e3c4d08ad70" />

- **H** — Altura de la imagen
- **W** — Ancho de la imagen
- **h** — Número de row anchors
- **w** — Número de gridding cells
- **C** — Número de carriles
- **X** — Características globales de la imagen
- **f** — Clasificador para seleccionar ubicaciones de carriles
- **P ∈ R^(C×h×(w+1))** — Predicciones
- **T ∈ R^(C×h×(w+1))** — Etiquetas objetivo
- **Prob ∈ R^(C×h×w)** — Probabilidad de cada ubicación
- **Loc ∈ R^(C×h)** — Ubicaciones de los carriles

<img width="355" height="41" alt="image" src="https://github.com/user-attachments/assets/37b8ca42-376d-4dda-86c6-d15401f8596f" />

para el carril i y la fila j, el clasificador f toma como entrada las características globales de la imagen, representadas por X, y genera una predicción P. Esa predicción es un vector que contiene la probabilidad de que el carril esté en cada celda horizontal de esa fila, esto responde al donde esta respecto a la linea horizontal.

**Funcón de perdida de clasificación:**
<img width="352" height="69" alt="image" src="https://github.com/user-attachments/assets/5979ae4f-82e2-442e-85a7-affe6d36ef3d" />

esta fórmula significa que el modelo compara su predicción con la respuesta correcta para cada carril y para cada fila. La función LCE representa la cross entropy loss, que sirve para medir qué tan equivocada está la predicción del modelo respecto a la etiqueta real. Donde T representa la etiqueta correcta 

<img width="671" height="233" alt="image" src="https://github.com/user-attachments/assets/e387dcc7-c8b5-4c66-baf4-a0dccce8be24" />

Si la imagen tiene altura H y ancho W, entonces el costo está relacionado con H por W por C + 1, porque se analiza cada pixel y se decide si pertenece a algún carril o al fondo. En cambio, en el método propuesto, el costo está relacionado con C por h por w + 1, porque solo se hacen predicciones para ciertos carriles, ciertas filas y ciertas celdas.

Esto es mucho más eficiente porque normalmente el número de row anchors h es mucho menor que la altura total H de la imagen, y el número de celdas w también es menor que el ancho total W. Por eso, el método puede ser mucho más rápido que una segmentación completa.
