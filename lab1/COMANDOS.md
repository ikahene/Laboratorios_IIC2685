# Comandos para la presentación — Lab 1

> Chuleta rápida. Cada bloque se puede copiar tal cual.
> Si algo falla, revisar primero la sección **"Si algo sale mal"** al final.

---

## 0. Antes de empezar (una sola vez, en cada terminal nueva)

```bash
cd ~/iic2685_ws
colcon build --symlink-install --packages-select lab1   # solo si cambió setup.py o se agregó un archivo nuevo
source install/setup.bash
```

- `--symlink-install`: instala *enlaces* a `src/` en vez de copias → editar un `.py` o `.txt` en `src/` tiene efecto inmediato, sin recompilar.
- `--packages-select lab1`: compila solo nuestro paquete (no el simulador).
- `source install/setup.bash`: hay que hacerlo en **cada terminal** que abra; si no, `ros2` no encuentra `lab1`.

**Antes de cada prueba el robot debe estar en (1, 1, 0)**: lo más seguro es cerrar (Ctrl+C) y volver a lanzar el simulador/launch. (La tecla `p` en la ventana del simulador activa un modo para *arrastrar* el robot a mano; no lo deja exacto en (1, 1, 0).)

---

## 1.1 Prueba funcional (poses que entrega el profesor)

1. Escribir las poses del profesor en `~/iic2685_ws/src/lab1/lab1/poses.txt`, una por línea, `x y yaw` (yaw en **radianes**; también acepta `x, y, yaw`):
   ```bash
   code ~/iic2685_ws/src/lab1/lab1/poses.txt
   ```
   (no hay que recompilar después de editarlo)

2. Tres terminales:
   ```bash
   # Terminal 1: simulador
   ros2 launch very_simple_robot_simulator run_all.xml

   # Terminal 2: navegador (esperar a que el simulador esté arriba)
   ros2 run lab1 dead_reckoning_nav

   # Terminal 3: cliente que lee el txt y envía las poses
   ros2 run lab1 pose_loader
   ```

**Alternativa en un solo comando** (usa el mismo launch de la 1.2 pero con el archivo de la 1.1):
```bash
ros2 launch lab1 avanzar_y_rotar.xml ruta:=$HOME/iic2685_ws/src/lab1/lab1/poses.txt
```

Usar otro archivo con `ros2 run`:
```bash
ros2 run lab1 pose_loader --ros-args -p ruta:=/ruta/a/otro.txt
```
(`--ros-args -p nombre:=valor` fija un parámetro ROS; solo funciona porque el nodo lo *declara*.)

En la terminal del navegador aparece `Llegué a la pose estimada: x=..., y=..., yaw=...` en cada pose.

---

## 1.2 Avanzar y rotar (cuadrado 1 m × 3 vueltas)

**Demostración (con factor de corrección) — un solo comando, levanta simulador + nodos:**
```bash
ros2 launch lab1 avanzar_y_rotar.xml
```

**Sin factor de corrección** (para mostrar la diferencia):
```bash
ros2 launch lab1 avanzar_y_rotar.xml factor_giro:=1.0
```

- Las poses del cuadrado están en `~/iic2685_ws/src/lab1/lab1/cuadrado.txt`.
- El factor por defecto es `1.111311640184305` (≈ 1/0.9). Al partir el nodo lo imprime: `Factor de giro en ejecución: ...`.
- El `pose_loader` arranca 3 s después que el resto (a propósito, ver EXPLICACION_CODIGO.md).

Ver la pose real / odometría mientras se mueve (otra terminal, con `source` hecho):
```bash
ros2 topic echo /real_pose --once      # pose real del simulador (parte en 1,1)
ros2 topic echo /odom --once           # odometría (parte en 0,0: está desplazada en (1,1))
ros2 topic hz /real_pose               # confirma que el simulador publica (~10 Hz)
```

---

## 2. Teleoperación

```bash
ros2 launch lab1 teleop.xml
```
Levanta el simulador (con Kinect) y el nodo `teleop`. **Las teclas se presionan en esa misma terminal** (no en la ventana del simulador).

| Tecla | Movimiento |
|---|---|
| `i` | adelante 0.2 m/s |
| `j` | atrás −0.2 m/s |
| `a` | rota 1.0 rad/s (izquierda) |
| `s` | rota −1.0 rad/s (derecha) |
| `q` | curva: 0.2 m/s y 1.0 rad/s |
| `w` | curva: 0.2 m/s y −1.0 rad/s |
| MAYÚS + tecla | deja el comando fijo (manos libres) |
| Espacio | detener |

Sin MAYÚS, el robot se mueve solo mientras se mantiene presionada la tecla (se detiene 0.3 s después de soltarla).

Solo el nodo, si el simulador ya está corriendo:
```bash
ros2 run lab1 teleop
```

> ⚠️ No dejar `teleop` corriendo al mismo tiempo que `dead_reckoning_nav`: teleop publica `(0, 0)` en `/cmd_vel` 20 veces por segundo y "pelea" con el navegador.

---

## 3. Percepción + 3.1 Acción y percepción

**Demostración — un solo comando** (simulador con Kinect + `obstacle_detector` + nav + loader con el cuadrado):
```bash
ros2 launch lab1 accion_y_percepcion.xml
```

**Poner / quitar paredes durante la demo** (en la ventana del simulador, con el foco en ella):
- `w`: entra al modo *agregar pared* → dibujar la pared arrastrando con el mouse delante del robot. `w` otra vez para salir del modo.
- `d`: entra al modo *borrar pared* → clic sobre la pared. `d` otra vez para salir.

Qué debe verse en la terminal:
- `obstacle_detector`: `Estado de ocupación: (x, y, z)` cada vez que cambia (1 = ocupado; orden izquierda, centro, derecha).
- `dead_reckoning_nav`: `obstacle left` / `obstacle center` / `obstacle right` al detenerse, y `Camino libre, continuando movimiento` al reanudar.

Ver el estado en vivo (otra terminal, con `source` hecho):
```bash
ros2 topic echo /occupancy_state        # Vector3: x = izq, y = centro, z = der
ros2 topic hz /camera/depth/image_raw   # la Kinect simulada publica a ~10 Hz
```

Probar la detención **sin** detector (simular un obstáculo a mano, con `avanzar_y_rotar.xml` corriendo):
```bash
ros2 topic pub -r 10 /occupancy_state geometry_msgs/msg/Vector3 "{x: 0.0, y: 1.0, z: 0.0}"   # Ctrl+C para cortar
ros2 topic pub -r 10 /occupancy_state geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"   # camino libre, dejarlo unos segundos
```
(`-r 10` publica 10 veces por segundo; la primera tarda ~1 s en llegar por el descubrimiento de DDS.)

Solo el detector, si ya hay un simulador con Kinect corriendo:
```bash
ros2 run lab1 obstacle_detector
ros2 run lab1 obstacle_detector --ros-args -p umbral_m:=0.6775   # umbral medido desde el centro del robot [m] (0.5 + radio 0.1775)
```

---

## Diagnóstico rápido

```bash
ros2 node list                  # ¿qué nodos están vivos?
ros2 topic list                 # ¿qué tópicos existen?
ros2 topic info /cmd_vel        # ¿quién publica / quién escucha cmd_vel?
ros2 topic echo /goal_list      # ¿llegó la lista de poses?
ros2 param get /dead_reckoning_nav factor_giro   # factor en uso
ros2 pkg executables lab1       # debe listar dead_reckoning_nav, obstacle_detector, pose_loader, teleop
```

## Si algo sale mal

| Síntoma | Causa probable | Solución |
|---|---|---|
| `Package 'lab1' not found` | Falta `source` en esa terminal | `source ~/iic2685_ws/install/setup.bash` |
| `No executable found` | Se compiló antes de agregar el entry point | Recompilar (bloque 0) |
| Cambié el código y se comporta igual que antes | Se compiló sin `--symlink-install` y corre una copia vieja | `rm -rf build/lab1 install/lab1` y recompilar con `--symlink-install` |
| El robot no se mueve con `ros2 run` | `pose_loader` corrió antes que `dead_reckoning_nav` | Lanzar primero el nav (terminal 2) y después el loader |
| El recorrido sale desplazado | El robot no estaba en (1, 1, 0) | Reiniciar el launch / simulador |
| El robot se detiene y no sigue (3.1) | Quedó una pared a ≤ 0.5 m de la punta del robot, o algo delante muy cerca (el sensor ve NaN) | Borrar la pared (`d` + clic); mirar `ros2 topic echo /occupancy_state` |
| `obstacle_detector` no imprime "Conexión con la cámara" | No hay Kinect simulada (ej. se lanzó `run_all.xml`, que no la incluye) | Usar `accion_y_percepcion.xml` o `teleop.xml` |
| Robot se mueve "a tirones" o se frena | Otro nodo publica en `/cmd_vel` (ej. teleop) | `ros2 topic info /cmd_vel` y cerrar el que sobra |
| Terminal "rara" (no se ve lo que escribo) tras cerrar teleop a la fuerza | Quedó en modo cbreak | Escribir `reset` y Enter |
