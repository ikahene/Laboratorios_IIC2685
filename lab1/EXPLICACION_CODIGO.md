# Cómo funciona nuestro Lab 1 (lectura de 10 minutos)

## La historia en 30 segundos

Un robot que se mueve **con los ojos cerrados**. Le damos una lista de poses en un `.txt`; él sabe dónde partió, calcula cuánto rato avanzar y girar para llegar a cada una (`tiempo = distancia / velocidad`) y lo ejecuta. Nunca mira dónde está realmente: eso es *dead reckoning* (navegación por estima).

Después le dimos "ojos": una cámara de profundidad que le avisa si hay algo adelante. Si hay obstáculo, el robot **pausa** su plan; cuando se despeja, lo **reanuda** donde quedó.

Y aparte, un control remoto con el teclado.

```
 poses.txt ─► [pose_loader] ──goal_list──► [dead_reckoning_nav] ──/cmd_vel──► [simulador]
                                                    ▲                              │
                                           /occupancy_state                 /real_pose, /odom
                                                    │                     (solo para medir)
 [cámara Kinect] ──/camera/depth/image_raw──► [obstacle_detector]

 [teleop] ──/cmd_vel──► [simulador]        (se usa solo, no junto al nav)
```

Cada nodo hace **una** cosa (leer archivo, mover, ver, teclado). Si algo falla, se escucha el tópico entre medio (`ros2 topic echo ...`) y se sabe al tiro de qué lado está el problema.

---

## 1. `pose_loader`: el que lee la lista de poses

**Idea:** es el cartero. Lee el `.txt`, mete todas las poses en un solo sobre (`PoseArray`) y lo entrega **una vez** en `goal_list`.

- Cada línea `x y yaw` (también acepta comas) se convierte en una `Pose`. El yaw se guarda como **cuaternión**, porque `Pose` no tiene un campo "ángulo": `quaternion_from_euler(0, 0, yaw)`.
- La ruta del archivo es un **parámetro ROS** (`ruta`), así se puede cambiar desde la terminal sin tocar el código. Se le aplica `expanduser` porque Python no entiende `~`.
- **Detalle clave:** antes de publicar espera a que alguien esté escuchando (`get_subscription_count() > 0`). Un tópico es como una radio en vivo: si hablas antes de que el otro sintonice, el mensaje se pierde.

---

## 2. `dead_reckoning_nav`: el que mueve el robot

Parte creyendo que está en `(1, 1, 0)`. Tras cada pose **asume** que llegó y actualiza su creencia (`pose_actual = goal_pose`). No usa `/odom` ni `/real_pose`: es **lazo abierto**.

Está organizado en los **tres niveles** que pide el enunciado. Piénsalo como un jefe, un planificador y un chofer:

**Nivel 3: `accion_mover_cb` (el jefe).** Recibe la lista de poses, convierte cada cuaternión de vuelta a yaw (`euler_from_quaternion`) y las entrega una por una al nivel 2.

**Nivel 2: `mover_robot_a_destino` (el planificador).** Para una pose: arma la lista de órdenes `[(v, w, t), ...]`, se la pasa al chofer, actualiza la pose estimada y loguea `Llegué a la pose estimada...`.

La lista la arma `calcular_lista_velocidades` con una **trayectoria en L**, como la figura 1 del enunciado:
1. Si hay que cambiar `y`: girar a mirar arriba o abajo y avanzar `|Δy| / 0.2` segundos.
2. Si hay que cambiar `x`: girar a mirar a la derecha o izquierda y avanzar `|Δx| / 0.2` segundos.
3. Girar al yaw final.

Cada giro lo calcula `comando_giro`:
- Toma el **giro más corto**: si la diferencia pasa de π, le resta 2π (mejor −90° que +270°).
- Si la diferencia es minúscula (< 0.005 rad, por escribir `1.57` en vez de π/2) no gira.
- Tiempo = `|ángulo| / 1.0 × factor_giro`. **Aquí, y solo aquí, se aplica el factor de corrección.**

**Nivel 1: `aplicar_velocidad` (el chofer).** Ejecuta cada orden `(v, w, t)`: publica el mismo `Twist` cada 0.05 s hasta cumplir `t` y al final publica un `Twist()` vacío para frenar.
- ¿Por qué repetirlo y no mandarlo una vez? El simulador tiene un "hombre muerto": si pasan 0.6 s sin órdenes, frena solo.
- El tiempo se lleva como un **cronómetro con pausa**: si hay obstáculo, publica "quieto" y **no descuenta tiempo**. Al despejarse sigue con lo que le faltaba, así no llega corto.

### Cómo puede "ver" mientras maneja

El jefe (`accion_mover_cb`) se queda ocupado toda la ruta (1–2 minutos). Con un solo hilo, nadie atendería los avisos de obstáculo hasta el final. Por eso:
- **`MultiThreadedExecutor`**: el nodo tiene varios "empleados" (hilos).
- **`ReentrantCallbackGroup`**: les da permiso para trabajar **al mismo tiempo**. Por defecto, aunque haya varios hilos, los callbacks de un nodo van de a uno.

Mientras uno maneja, otro atiende `occupancy_cb`: si llega algún 1 en `/occupancy_state`, pone `obstaculo_presente = True` y loguea `obstacle left / center / right`; con `(0, 0, 0)` loguea `Camino libre, continuando movimiento`. Solo loguea **cuando el estado cambia** (llegan ~10 mensajes por segundo y llenaría la terminal). El chofer revisa esa variable en cada vuelta.

---

## 3. El cuadrado y el factor de corrección (actividad 1.2)

`cuadrado.txt` tiene 4 vértices (`2 1 1.57`, `2 2 3.14`, `1 2 -1.57`, `1 1 0`) repetidos 3 veces. Cada pose deja al robot ya mirando hacia el siguiente lado, así que por vuelta hay **4 giros de 90°**.

**¿Por qué un factor de 1.111?** El simulador tiene programada "fricción": solo gira el **90 %** de lo que se le pide (`delta_yaw = 0.9 * vyaw * dt` en `kobuki_simulator.py`). Pides 90° y gira 81°. Solución: girar un poco más de tiempo, `1 / 0.9 ≈ 1.111`.

**¿Por qué solo en los giros?**
1. El simulador solo perturba los giros; el avance es exacto.
2. Un error de ángulo es mucho peor que uno de distancia. Es como una flecha mal apuntada: 1° de desvío casi no se nota al principio, pero el error lateral (`d · sin Δθ`) crece con cada metro y se arrastra a **todos** los lados siguientes. Un error de distancia se queda donde ocurrió.

**Sin factor:** cada esquina queda en ~81°, el cuadrado se va "cerrando en espiral" y el robot termina lejos del inicio.

**El launch `avanzar_y_rotar.xml`** levanta el simulador, el nav (con `factor_giro` como argumento, para correrlo con `factor_giro:=1.0`) y el loader con `cuadrado.txt`. Dos detalles:
- La ruta se busca con `$(find-pkg-share lab1)`, así funciona en cualquier PC.
- El loader parte **3 s después**. Al lanzar todo junto, los nodos tardan ~1 s en "encontrarse" en la red (descubrimiento DDS) y se perdía el primer segundo de órdenes: el primer lado salía de ~0.8 m en vez de 1 m.

**¿Por qué igual queda error? (reflexión que piden en las slides)**
- *Software:*
  - Lazo abierto: los errores se acumulan y nada los corrige.
  - El simulador calcula la pose a 10 Hz: un giro de 1.745 s cuenta como 1.7 o como 1.8 s, unos ±2.5° por esquina.
  - `sleep` y el sistema operativo no son exactos.
  - `1.57` no es exactamente π/2.
  - En el simulador `/odom` se calcula igual que `/real_pose` (solo parte en (0, 0)), así que en el gráfico coinciden. En un robot real no.
- *Hardware:* deslizamiento de ruedas, suelo irregular, ruedas que no miden exactamente lo nominal, motores que tardan en acelerar y frenar, batería baja, encoders y giroscopio imperfectos.

---

## 4. `teleop`: el control remoto

**Idea:** lee el teclado tecla por tecla y publica velocidades ~20 veces por segundo.

- Un diccionario traduce cada tecla a `(v, w)`: `i` adelante, `j` atrás, `a` y `s` giran, `q` y `w` curvas.
- Abre `/dev/tty` (la terminal misma) y no `stdin`, porque con `ros2 launch` el stdin del nodo no es el teclado.
- Pone la terminal en **modo cbreak**: cada tecla llega al instante, sin Enter ni eco.
- En cada vuelta del ciclo, `select` espera **máximo 0.05 s** por una tecla y luego publica igual. Eso mantiene vivo al "hombre muerto" del simulador.
- **Soltar = frenar:** si pasan 0.3 s sin teclas, vuelve a `(0, 0)`. Al mantener una tecla, el SO la repite sola y renueva el tiempo.
- **MAYÚS + tecla** deja el comando fijo (manos libres); **espacio** frena.
- Al salir (`finally`): frena el robot y **devuelve la terminal a la normalidad**; si no, quedaría sin mostrar lo que escribes.
- No llama `rclpy.spin` porque solo publica: no tiene nada que "escuchar".

Ojo: como publica `(0, 0)` todo el rato, si corre junto al nav "pelean" por `/cmd_vel`.

---

## 5. `obstacle_detector`: los ojos

**Idea:** mira la imagen de profundidad (cada píxel = distancia en metros), la parte en **tres franjas verticales** (izquierda, centro, derecha) y dice cuál está ocupada. Publica siempre, con cada imagen (~10 Hz), un `Vector3` en `/occupancy_state`: `x` izquierda, `y` centro, `z` derecha; 1 ocupado, 0 libre.

Paso a paso por imagen:
1. **A metros:** la Kinect real entrega milímetros (`uint16`) y el simulador metros (`float32`); `a_metros` deja ambas en metros, con `NaN` donde no hay dato. El mismo nodo sirve para los dos.
2. **Banda central:** usa solo las filas del 25 % al 75 % de la altura. Arriba está el techo y abajo el suelo, que no son obstáculos.
3. **Tres tercios** por columnas y `region_ocupada` en cada uno.

**Criterio de detección (va en las slides):** una región está ocupada si
- su punto más cercano está a **≤ 0.6775 m del sensor**, o sea **0.5 m desde la punta del robot** (el sensor está en el centro y el robot mide 0.355 m de diámetro: 0.5 + 0.1775). El umbral es el parámetro `umbral_m`;
- **o** si al menos el 10 % de sus píxeles son `NaN`.

¿Por qué los `NaN` cuentan? La Kinect tiene una distancia **mínima**: lo que está demasiado cerca (< 0.45 m en el simulador) no lo puede medir y aparece "en blanco". Si solo miráramos los píxeles válidos, una pared pegada al robot ¡parecería camino libre! Es como un ojo que no puede enfocar lo que tiene pegado a la nariz.

Todo se hace con **NumPy** sobre la matriz completa (`min`, máscaras de `NaN`), sin recorrer píxel por píxel, como pide el enunciado. Las mismas operaciones con `for` serían cientos de veces más lentas.

**El launch `accion_y_percepcion.xml`** levanta el simulador **con la Kinect** (con el mapa conectado para que vea las paredes), el detector, el nav y el loader con el cuadrado (también 3 s después).

---

## 6. Preguntas que nos pueden hacer

**Dead reckoning y movimiento**
1. ¿Qué datos usa dead reckoning? → Punto de partida, dirección y velocidades aplicadas (y el tiempo de cada una).
2. ¿Por qué el robot no termina exactamente en el origen aunque el código sea "perfecto"? → Lazo abierto + error acumulado (sección 3).
3. ¿Qué pasa si se publica un `Twist` una sola vez? → El simulador frena a los 0.6 s ("hombre muerto"); por eso se republica cada 0.05 s.
4. ¿Por qué trayectoria en L y no en diagonal? → La recomienda el enunciado: solo hay movimientos puros (o avanza o gira), más fáciles de calcular y depurar.
5. ¿Por qué `time.monotonic()` y no `time.time()`? → `monotonic` nunca retrocede (no le afectan ajustes del reloj); sirve para medir duraciones.

**Poses y mensajes**

6. ¿Qué pasa si `pose_loader` se lanza antes que el nav? → Espera con `get_subscription_count()` a que el nav escuche; no se pierde la lista.
7. ¿Por qué cuaternión y no el yaw directo? → `Pose` usa cuaterniones (evitan las singularidades de Euler y sirven para cualquier rotación 3D). En el plano: `q = (0, 0, sin(θ/2), cos(θ/2))`.
8. ¿Por qué `PoseArray`? → Lo recomienda el enunciado: toda la ruta en un solo mensaje y en orden.
9. ¿Y si el archivo de poses no existe? → No se verifica a propósito (siempre corremos en el mismo PC); `open` lanza `FileNotFoundError` y el error muestra la ruta.
10. ¿Por qué el loader parte 3 s después en los launch? → Para que los nodos terminen de descubrirse en DDS; si no, se pierde el primer segundo de órdenes.

**Factor de corrección**

11. ¿Por qué `1/0.9`? → El simulador aplica solo el 90 % del giro pedido.
12. ¿Dónde se aplica el factor y por qué ahí? → En el `return` de `comando_giro`: es el único lugar donde se calcula un tiempo de giro.
13. ¿Por qué solo en las rotaciones? → Solo los giros están perturbados, y un error de ángulo se amplifica en todos los tramos siguientes.
14. ¿Diferencia entre `/odom` y `/real_pose`? → `/odom` es lo que el robot *cree* (integrando sus ruedas); `/real_pose` es la verdad del simulador. En un robot real no existe: se mediría con cámaras externas.

**Percepción**

15. ¿Cómo reacciona el nav a obstáculos si `accion_mover_cb` está bloqueado en su ciclo? → `MultiThreadedExecutor` + `ReentrantCallbackGroup`: `occupancy_cb` corre en otro hilo y cambia `obstaculo_presente`.
16. ¿Por qué el robot no llega corto después de detenerse por un obstáculo? → Mientras está pausado no se descuenta tiempo del comando.
17. ¿Por qué un 10 % de `NaN` cuenta como obstáculo? → Lo que está a menos de ~0.45 m no se puede medir (distancia mínima del sensor).
18. ¿Por qué solo la banda central de la imagen? → Para no confundir el suelo ni el techo con obstáculos.
19. ¿Los 50 cm son desde la punta del robot? → Sí: el sensor mide desde el centro, así que el umbral es 0.5 + radio (0.1775) = 0.6775 m.
20. ¿Por qué `Vector3`? → Lo recomienda el enunciado; son justo 3 números (izquierda, centro, derecha).
21. ¿Qué pasa si el detector se cae? → El nav se queda con el último estado. Si era "libre", sigue sin percepción (una mejora sería un timeout de seguridad).

**Teleoperación**

22. ¿Por qué `teleop` abre `/dev/tty` y no usa `input()`? → `input()` espera Enter y, con `ros2 launch`, el stdin no es el teclado; `/dev/tty` en modo cbreak entrega cada tecla al instante.
23. ¿Por qué no llama `rclpy.spin`? → Solo publica; no tiene suscripciones ni timers que atender.
24. ¿Qué pasa si se cierra `teleop` a la fuerza? → El `finally` frena el robot y restaura la terminal; si igual queda rara, se arregla escribiendo `reset`.
