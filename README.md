# Práctica 1 - UR3e con ROS 2

**Autores:** Iñigo Fiat Sarasola y Ainara Maroto Fernández

## Archivos del proyecto

Tenemos dos paquetes:
- **`robotica_practica1`**: código de la práctica.
  - `parte1.py`: tareas de la Parte 1.
  - `parte2.py`: retos de la Parte 2.
  - `control_robot.py`: una de las clases facilitadas en ALUD.
- **`robotica_interfaces`**: mensaje personalizado de la Parte 2. Está en un paquete aparte porque en ROS 2 los mensajes solo se pueden generar desde un paquete de tipo CMake.

## Compilación

Desde el contenedor:

```bash
cd ~/ros_workspace
colcon build --packages-select robotica_interfaces
colcon build --symlink-install --packages-select robotica_practica1
source install/setup.bash
```

El comando `source install/setup.bash` hay que ejecutarlo en cada terminal nueva.

## Ejecución

En una terminal se lanza la simulación, que debe quedarse abierta:

```bash
ros2 launch launcher_robots_lab_robotica sim_203.launch.py
```

En otra terminal se ejecuta la parte que se quiera probar:

| Parte | Comando |
|---|---|
| Tarea 1: lectura de las articulaciones | `ros2 run robotica_practica1 practica1` |
| Tarea 2: trayectorias | `ros2 run robotica_practica1 trayectorias` |
| Tarea 3: trayectorias y registro de datos | `ros2 run robotica_practica1 practica` |
| Parte 2 completa | `ros2 run robotica_practica1 practica2` |

Los datos se guardan en `~/ros_workspace/datos_trayectorias/`, en un CSV por trayectoria.

## Parte 1

**Tarea 1.** Leer en tiempo real el estado de las articulaciones. Hecho con un nodo suscrito a `/joint_states`, que muestra el tiempo, la posición y la velocidad de cada articulación.

**Tarea 2.** Antes de cada trayectoria, hacer que el robot vuelva a una posición inicial. Después realizar:
- **Joint:** mueve las articulaciones a varias posiciones.
- **Lineal:** el extremo del robot dibuja un cuadrado de 10 cm de lado.
- **Circular:** el extremo dibuja un círculo de 5 cm de radio, definido con 36 puntos.

Durante cada trayectoria se publica su nombre en el topic `/trayectoria_actual`. De esta forma, el nodo que guarda los datos sabe cuándo empieza y cuándo termina cada una.

**Tarea 3.** Guardar la posición, la velocidad y la aceleración de cada articulación con su marca de tiempo, separando cada trayectoria. Hecho creando un CSV por trayectoria. Como la aceleración no viene en `/joint_states`, la calculamos con la diferencia de velocidad entre dos muestras dividida entre el tiempo que pasa entre ellas.
