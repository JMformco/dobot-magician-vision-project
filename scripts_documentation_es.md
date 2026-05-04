# Documentación de Scripts del Proyecto Dobot

Este documento proporciona una explicación paso a paso del propósito y el flujo de ejecución de cada script de Python en el directorio del proyecto.

---

### `pick_and_place_cubes_RGBY_node.py`
**Propósito:** Automatiza el proceso de detección de cubos de colores (Rojo, Azul, Verde, Amarillo) y su clasificación utilizando un brazo robótico con un riel lineal, mientras transmite simultáneamente el feed de video a un panel web.
**Paso a paso:**
1. Inicializa un servidor Flask en un hilo de fondo separado para transmitir el feed de video vía HTTP.
2. Se conecta al brazo Dobot, ejecuta su secuencia de homing (regreso a inicio) y posiciona el riel lineal en un estado predeterminado.
3. Carga una matriz de calibración (para traducir coordenadas de píxeles a coordenadas del mundo real) y opcionalmente carga un polígono de máscara de visión.
4. Inicializa la cámara Hikrobot a través de su SDK y comienza a capturar fotogramas.
5. En el bucle principal, captura un fotograma, lo convierte a HSV y aplica máscaras de color para detectar contornos coloreados.
6. Filtra las detecciones por tamaño de área, forma (aproximando 4 vértices) y lógica de contacto con los bordes para aislar cubos individuales.
7. Requiere que una detección permanezca estable en el mismo punto durante 2 segundos antes de desencadenar una acción.
8. Una vez estable, transforma las coordenadas centrales de la cámara a coordenadas físicas del robot y asigna una posición de depósito en el riel basada en el color detectado.
9. Encola y ejecuta comandos del Dobot para moverse al cubo, activar la ventosa, levantar el cubo, moverse a lo largo del riel, colocar el cubo en la posición de depósito y regresar a la posición predeterminada.
10. Reanuda la detección de visión una vez que el movimiento físico se completa.

---

### `calibrate_camera_dobot.py`
**Propósito:** Crea una matriz de transformación de perspectiva (`calibration_matrix.npy`) para mapear matemáticamente las coordenadas de píxeles de la cámara a coordenadas físicas del robot.
**Paso a paso:**
1. Se conecta al brazo Dobot y a la cámara Hikrobot.
2. Muestra el feed de la cámara en vivo y espera la interacción del usuario mediante dobles clics del ratón.
3. Cuando el usuario hace doble clic en el feed, el script registra la coordenada del píxel clicado y simultáneamente lee la posición física actual del efector final del brazo Dobot.
4. Una vez que se registran 4 puntos distintos, utiliza OpenCV (`cv2.getPerspectiveTransform`) para calcular la matriz de transformación de perspectiva y la guarda en el disco.
5. Cambia a un modo de "Prueba" donde hacer doble clic en cualquier punto del feed de la cámara ordena automáticamente al Dobot moverse a esa ubicación física para verificar la precisión.

---

### `create_vision_mask.py`
**Propósito:** Permite al usuario dibujar manualmente un polígono en el feed de la cámara para definir una región de interés (ROI), que se utiliza para ignorar cualquier cosa fuera del espacio de trabajo durante el procesamiento de visión.
**Paso a paso:**
1. Se conecta a la cámara Hikrobot y muestra su feed en vivo.
2. Registra clics del ratón para crear una lista de puntos que forman un polígono.
3. Dibuja líneas que conectan los puntos seleccionados en la pantalla para construir visualmente la máscara.
4. Espera entrada del teclado: 'c' para borrar puntos, 's' para guardar los puntos de la máscara en `vision_mask.npy`, o 'q' para salir.

---

### `detect_shapes_colors_mvs.py`
**Propósito:** Una herramienta de diagnóstico de visión por computadora para rastrear objetos de colores específicos y determinar sus formas geométricas y dimensiones físicas usando la cámara Hikrobot.
**Paso a paso:**
1. Se conecta a la cámara Hikrobot y comienza a capturar fotogramas.
2. Carga `vision_mask.npy` si está disponible para limitar el procesamiento de imágenes al área del espacio de trabajo.
3. En un bucle, captura un fotograma, lo desenfoca para eliminar el ruido y lo convierte al espacio de color HSV.
4. Aplica una máscara de color HSV para el color activo actualmente (que se puede alternar mediante entradas de teclado 'r', 'g', 'b', 'y', 'w').
5. Encuentra contornos en la imagen enmascarada y filtra ruidos pequeños.
6. Analiza el perímetro y los vértices de cada contorno para determinar su forma geométrica (Triángulo, Cuadrado, Rectángulo, Círculo, Polígono) y dimensiones delimitadoras.
7. Calcula el centro geométrico de la forma y superpone esta información como texto y gráficos de seguimiento en el feed de video.

---

### `dobot_gui_control.py`
**Propósito:** Proporciona una interfaz gráfica de usuario (GUI) completa para controlar manualmente, configurar y monitorear el brazo Dobot.
**Paso a paso:**
1. Utiliza el marco PySide6 para construir una ventana de escritorio que contiene configuraciones de conexión, controles de eje y límites de configuración.
2. Establece una conexión en serie con el Dobot a través del puerto COM seleccionado por el usuario.
3. Ejecuta un temporizador de fondo que consulta continuamente la pose actual del robot (X, Y, Z, R, L) y actualiza las pantallas de la interfaz de usuario.
4. Proporciona botones "Jog" (Avanzar paso a paso) para mover manualmente cada eje de forma incremental.
5. Intercepta las solicitudes de movimiento y las valida contra los límites mínimo/máximo predefinidos de los ejes para prevenir colisiones de hardware.
6. Envía comandos de movimiento (`PTPMOVLXYZMode` y `SetPTPWithLCmd`) al robot para ejecutar el movimiento manual.

---

### `get_position.py`
**Propósito:** Una utilidad simple para recuperar y mostrar las coordenadas físicas actuales del robot y su riel lineal.
**Paso a paso:**
1. Se conecta al Dobot a través del puerto serie COM.
2. Limpia la cola de comandos para asegurar la ejecución inmediata.
3. Solicita la pose del efector final del robot (X, Y, Z, R).
4. Solicita la posición del riel lineal (L).
5. Imprime las coordenadas formateadas en la terminal y se desconecta.

---

### `x_z_circle.py`
**Propósito:** Demuestra cómo generar trayectorias matemáticas complejas haciendo que el Dobot dibuje un círculo en el plano vertical X-Z.
**Paso a paso:**
1. Se conecta al Dobot y habilita el riel lineal.
2. Configura velocidades de movimiento general y parámetros de aceleración.
3. Utiliza un bucle y funciones trigonométricas (seno y coseno) para calcular 36 puntos de coordenadas secuenciales que forman un círculo.
4. Encola 36 comandos individuales de movimiento punto a punto (`SetPTPWithLCmd`) para el Dobot.
5. Inicia la ejecución de la cola de comandos y espera en un bucle hasta que el robot termine de interpolar y dibujar el círculo completo.

---

### Ejemplos de Control Básico (`DobotControl_R200.py`, `DobotControl_adaptado.py`, `DobotControl_home.py`)
**Propósito:** Scripts fundamentales que demuestran la conexión básica, homing (regreso a inicio) y la lógica de movimiento punto a punto.
**Paso a paso (Flujo general):**
1. Se conecta al brazo Dobot.
2. Desencadena la secuencia de homing para restablecer el sistema de coordenadas interno del robot.
3. Habilita el accesorio del riel lineal.
4. Encola comandos de movimiento punto a punto codificados. Por ejemplo, `_R200` ordena al riel moverse a la posición 200, mientras que `_adaptado` prueba movimientos incrementales más pequeños del brazo.
5. Inicia la ejecución y espera a que se complete antes de desconectarse.

---

### `track_red.py`
**Propósito:** Un script de prueba de visión por computadora simple que rastrea objetos rojos utilizando una cámara web USB estándar.
**Paso a paso:**
1. Abre una conexión de captura de video a la cámara web USB predeterminada utilizando OpenCV.
2. Captura fotogramas continuamente en un bucle `while`.
3. Desenfoca y convierte los fotogramas al espacio de color HSV.
4. Aplica una máscara HSV de rango dual para aislar píxeles rojos (manejando el desbordamiento del matiz).
5. Encuentra contornos, calcula el centro usando momentos de imagen y dibuja un círculo de seguimiento en la pantalla.

---

### `track_red_mvs.py`
**Propósito:** Rastrea objetos rojos de manera similar a `track_red.py`, pero implementa específicamente el SDK de la cámara industrial Hikrobot en lugar de depender de un feed de cámara web estándar.
**Paso a paso:**
1. Inicializa el SDK de Hikrobot, busca la cámara y abre un manipulador (handle) de dispositivo.
2. Configura tamaños de paquetes de red óptimos y comienza a capturar fotogramas en modo de funcionamiento libre (free-run).
3. Recibe búferes de bytes sin procesar de la cámara y los remodela en matrices numpy compatibles con OpenCV según el formato de píxeles detectado (por ejemplo, RGB8 o BayerRG8).
4. Aplica el mismo umbral HSV, búsqueda de contornos y lógica de cálculo de centro que el script de cámara web estándar.
5. Apaga de forma segura el manipulador del SDK y libera la memoria de la cámara al salir.

---

### `DobotDllType.py`
**Propósito:** Una biblioteca contenedora de Python proporcionada por Dobot que interactúa con el SDK de C++ subyacente (`DobotDll.dll`).
**Paso a paso:**
1. Utiliza la biblioteca `ctypes` de Python para cargar la biblioteca de vínculos dinámicos (`CDLL`).
2. Define constantes, enumeraciones y estructuras de datos que coinciden perfectamente con la API de C++.
3. Mapea las llamadas a funciones de Python a sus correspondientes funciones exportadas de C++, manejando automáticamente los tipos de argumentos y los punteros de memoria.
