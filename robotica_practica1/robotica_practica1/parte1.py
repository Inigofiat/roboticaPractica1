"""Práctica 1: suscriptor (tarea 1), trayectorias (tarea 2) y grabación de datos (tarea 3)."""

import copy
import csv
import os
from datetime import datetime
from math import cos, pi, sin
from typing import Any, Callable, Dict, List, Optional, TextIO

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robotica_practica1.control_robot import ControlRobot

TOPIC_ARTICULACIONES: str = "/joint_states"
TAMANO_COLA: int = 10
# header.stamp separa el tiempo en segundos enteros y nanosegundos
NANOSEGUNDOS_POR_SEGUNDO: float = 1e9

# Orden fijo en el que se guardan y muestran las articulaciones, porque
# /joint_states no garantiza que lleguen siempre en el mismo orden
ARTICULACIONES: List[str] = [
    "shoulder_pan_joint",   # base
    "shoulder_lift_joint",  # hombro
    "elbow_joint",          # codo
    "wrist_1_joint",        # muñeca 1
    "wrist_2_joint",        # muñeca 2
    "wrist_3_joint",        # muñeca 3
]

# Topic con el nombre de la trayectoria en curso; el grabador lo usa para saber
# cuándo abrir y cerrar cada CSV. SIN_TRAYECTORIA indica que no hay ninguna activa.
TOPIC_TRAYECTORIA: str = "/trayectoria_actual"
SIN_TRAYECTORIA: str = ""

MEDIO_PI: float = pi / 2

# Postura de partida (radianes), a la que el robot vuelve antes de cada trayectoria
POSICION_INICIAL: List[float] = [0.0, -MEDIO_PI, -MEDIO_PI, -MEDIO_PI, MEDIO_PI, 0.0]

# Posturas que recorre la trayectoria Joint (radianes): primero gira la base
# a la izquierda y después a la derecha cambiando también hombro, codo y muñecas
POSICIONES_JOINT: List[List[float]] = [
    [pi / 4, -MEDIO_PI, -MEDIO_PI, -MEDIO_PI, MEDIO_PI, 0.0],
    [-pi / 4, -MEDIO_PI, -pi / 3, -2 * pi / 3, MEDIO_PI, pi / 4],
]

LADO_CUADRADO_M: float = 0.10  # lado del cuadrado de la trayectoria lineal (metros)

RADIO_CIRCULO_M: float = 0.05  # radio del círculo (metros)
PUNTOS_CIRCULO: int = 36       # más puntos dan un círculo más suave

CARPETA_DATOS: str = os.path.expanduser("~/ros_workspace/datos_trayectorias")
FORMATO_FECHA_ARCHIVO: str = "%Y%m%d_%H%M%S"
# Dos muestras más próximas que esto se consideran repetidas: dividir entre un
# intervalo casi nulo daría aceleraciones enormes y falsas
INTERVALO_MINIMO_S: float = 1e-4
PERIODO_LOG_S: float = 1.0  # como mucho un mensaje de log por segundo


# TAREA 1: suscriptor
class SuscriptorArticulaciones(Node):
    """Nodo que escucha /joint_states y muestra posición y velocidad del brazo."""

    def __init__(self) -> None:
        """Crea el nodo y se suscribe al topic de las articulaciones."""
        super().__init__("suscriptor_articulaciones")
        self.create_subscription(
            JointState, TOPIC_ARTICULACIONES, self._leer_articulaciones, TAMANO_COLA
        )

    def _leer_articulaciones(self, mensaje: JointState) -> None:
        """Callback: ROS lo ejecuta automáticamente con cada mensaje recibido."""
        tiempo: float = (
            mensaje.header.stamp.sec
            + mensaje.header.stamp.nanosec / NANOSEGUNDOS_POR_SEGUNDO
        )
        # Relaciona cada nombre con su posición en las listas del mensaje
        indice: Dict[str, int] = {}
        for posicion_en_lista, nombre in enumerate(mensaje.name):
            indice[nombre] = posicion_en_lista
        hay_velocidad: bool = len(mensaje.velocity) == len(mensaje.name)

        for articulacion in ARTICULACIONES:
            if articulacion not in indice:
                continue
            i: int = indice[articulacion]
            posicion: float = mensaje.position[i]
            if hay_velocidad:
                velocidad: float = mensaje.velocity[i]
            else:
                velocidad = 0.0
            self.get_logger().info(
                f"t={tiempo:.3f}s  {articulacion}: "
                f"pos={posicion:+.4f} rad  vel={velocidad:+.4f} rad/s"
            )


# TAREA 2: trayectorias
class MultiplesTrayectorias(ControlRobot):
    """Mueve el robot por varias trayectorias y anuncia cuál se está ejecutando."""

    def __init__(self) -> None:
        """Conecta con MoveIt (ControlRobot) y crea el publicador de la trayectoria."""
        super().__init__()
        self._publicador = self.create_publisher(String, TOPIC_TRAYECTORIA, TAMANO_COLA)

    def ejecutar_trayectorias(self) -> None:
        """Ejecuta las trayectorias, volviendo a la posición inicial antes de cada una."""
        trayectorias: Dict[str, Callable[[], bool]] = {
            "joint": self._trayectoria_joint,
            "lineal": self._trayectoria_lineal,
            "circular": self._trayectoria_circular,
        }
        for nombre, ejecutar in trayectorias.items():
            self._ir_a_posicion_inicial()
            self.get_logger().info(f"Iniciando trayectoria {nombre.upper()}")
            # Se anuncia antes y después para que el grabador solo guarde
            # los datos del movimiento, no los de la vuelta a la posición inicial
            self._anunciar_trayectoria(nombre)
            exito: bool = ejecutar()
            self._anunciar_trayectoria(SIN_TRAYECTORIA)
            if exito:
                self.get_logger().info(f"Trayectoria {nombre.upper()} completada")
            else:
                self.get_logger().error(f"La trayectoria {nombre.upper()} no se pudo completar")
        self._ir_a_posicion_inicial()

    def _anunciar_trayectoria(self, nombre: str) -> None:
        """Publica el nombre de la trayectoria en curso (o SIN_TRAYECTORIA)."""
        mensaje = String()
        mensaje.data = nombre
        self._publicador.publish(mensaje)

    def _ir_a_posicion_inicial(self) -> None:
        """Lleva el robot a la postura de partida."""
        if not self.mover_articulaciones(POSICION_INICIAL):
            self.get_logger().warn("No se pudo alcanzar la posición inicial")

    def _trayectoria_joint(self) -> bool:
        """Movimiento JOINT: cada articulación gira hasta los ángulos indicados."""
        for posicion in POSICIONES_JOINT:
            if not self.mover_articulaciones(posicion):
                return False
        return True

    def _trayectoria_lineal(self) -> bool:
        """Movimiento LINEAL: la punta recorre en línea recta un cuadrado horizontal."""
        esquina: Pose = self.pose_actual()
        cuadrado: List[Pose] = []
        # Se desplaza la esquina lado a lado; deepcopy guarda una copia de cada
        # vértice, porque si no todos apuntarían al mismo objeto y acabarían iguales
        esquina.position.x += LADO_CUADRADO_M
        cuadrado.append(copy.deepcopy(esquina))
        esquina.position.y += LADO_CUADRADO_M
        cuadrado.append(copy.deepcopy(esquina))
        esquina.position.x -= LADO_CUADRADO_M
        cuadrado.append(copy.deepcopy(esquina))
        esquina.position.y -= LADO_CUADRADO_M
        cuadrado.append(copy.deepcopy(esquina))
        return self.mover_trayectoria(cuadrado)

    def _trayectoria_circular(self) -> bool:
        """Movimiento CIRCULAR: la punta recorre un círculo que empieza y acaba donde está."""
        inicio: Pose = self.pose_actual()
        # El centro se sitúa a un radio de distancia, para que la posición actual
        # ya forme parte del círculo y el robot no tenga que desplazarse antes
        centro_x: float = inicio.position.x + RADIO_CIRCULO_M
        centro_y: float = inicio.position.y
        circulo: List[Pose] = []
        for i in range(1, PUNTOS_CIRCULO + 1):
            # La posición actual corresponde al ángulo pi; desde ahí se da una
            # vuelta completa, de modo que el último punto cierra el círculo
            angulo: float = pi + i * 2 * pi / PUNTOS_CIRCULO
            punto: Pose = copy.deepcopy(inicio)
            punto.position.x = centro_x + RADIO_CIRCULO_M * cos(angulo)
            punto.position.y = centro_y + RADIO_CIRCULO_M * sin(angulo)
            circulo.append(punto)
        return self.mover_trayectoria(circulo)


# TAREA 3: grabación de datos
class GrabadorCinematica(Node):
    """Guarda posición, velocidad y aceleración en un CSV distinto por trayectoria."""

    def __init__(self) -> None:
        """Prepara la carpeta de datos y se suscribe a la trayectoria y a las articulaciones."""
        super().__init__("grabador_cinematica")
        os.makedirs(CARPETA_DATOS, exist_ok=True)
        self._trayectoria_actual: str = SIN_TRAYECTORIA
        self._archivo: Optional[TextIO] = None
        self._escritor_csv: Optional[Any] = None  # objeto devuelto por csv.writer
        self._tiempo_inicio: float = 0.0
        self._tiempo_anterior: Optional[float] = None
        self._velocidades_anteriores: List[float] = []
        self._filas_guardadas: int = 0
        self.create_subscription(
            String, TOPIC_TRAYECTORIA, self._cambiar_trayectoria, TAMANO_COLA
        )
        self.create_subscription(
            JointState, TOPIC_ARTICULACIONES, self._guardar_muestra, TAMANO_COLA
        )
        self.get_logger().info(f"Guardando los datos en {CARPETA_DATOS}")

    def cerrar(self) -> None:
        """Cierra el CSV abierto, si lo hay."""
        self._cerrar_csv()

    def _cambiar_trayectoria(self, mensaje: String) -> None:
        """Cierra el CSV anterior y abre uno nuevo cuando cambia la trayectoria."""
        if mensaje.data == self._trayectoria_actual:
            return
        self._cerrar_csv()
        self._trayectoria_actual = mensaje.data
        if self._trayectoria_actual != SIN_TRAYECTORIA:
            self._abrir_csv(self._trayectoria_actual)

    def _abrir_csv(self, nombre_trayectoria: str) -> None:
        """Crea un CSV nuevo (p. ej. joint_20261106_113000.csv) con su cabecera."""
        fecha: str = datetime.now().strftime(FORMATO_FECHA_ARCHIVO)
        ruta: str = os.path.join(CARPETA_DATOS, f"{nombre_trayectoria}_{fecha}.csv")
        self._archivo = open(ruta, "w", newline="")
        self._escritor_csv = csv.writer(self._archivo)
        cabecera: List[str] = ["tiempo_s", "tiempo_relativo_s"]
        for magnitud in ("pos", "vel", "acc"):
            for articulacion in ARTICULACIONES:
                cabecera.append(f"{magnitud}_{articulacion}")
        self._escritor_csv.writerow(cabecera)
        # Cada trayectoria empieza de cero: sin muestra anterior con la que derivar
        self._tiempo_anterior = None
        self._velocidades_anteriores = []
        self._filas_guardadas = 0
        self.get_logger().info(f"Grabando '{nombre_trayectoria}' en {ruta}")

    def _cerrar_csv(self) -> None:
        """Cierra el CSV en curso, asegurando que todo queda escrito en disco."""
        if self._archivo is None:
            return
        self._archivo.close()
        self.get_logger().info(
            f"Trayectoria '{self._trayectoria_actual}' guardada: {self._filas_guardadas} filas"
        )
        self._archivo = None
        self._escritor_csv = None

    def _guardar_muestra(self, mensaje: JointState) -> None:
        """Calcula la aceleración y escribe una fila por cada mensaje de /joint_states."""
        if self._escritor_csv is None:
            return  # no hay ninguna trayectoria en curso
        tiempo: float = (
            mensaje.header.stamp.sec
            + mensaje.header.stamp.nanosec / NANOSEGUNDOS_POR_SEGUNDO
        )
        indice: Dict[str, int] = {}
        for posicion_en_lista, nombre in enumerate(mensaje.name):
            indice[nombre] = posicion_en_lista
        # Se descartan mensajes incompletos: sin las 6 articulaciones o sin
        # velocidades no se puede escribir una fila completa ni derivar
        if not all(articulacion in indice for articulacion in ARTICULACIONES):
            return
        if len(mensaje.velocity) != len(mensaje.name):
            return
        posiciones: List[float] = [mensaje.position[indice[a]] for a in ARTICULACIONES]
        velocidades: List[float] = [mensaje.velocity[indice[a]] for a in ARTICULACIONES]

        if self._tiempo_anterior is None:
            # Primera muestra: no hay anterior, así que la aceleración se toma como 0
            self._tiempo_inicio = tiempo
            aceleraciones: List[float] = [0.0] * len(ARTICULACIONES)
        else:
            intervalo: float = tiempo - self._tiempo_anterior
            if intervalo < INTERVALO_MINIMO_S:
                return
            aceleraciones = self._derivar(velocidades, self._velocidades_anteriores, intervalo)

        self._escritor_csv.writerow(
            [tiempo, tiempo - self._tiempo_inicio] + posiciones + velocidades + aceleraciones
        )
        self._filas_guardadas += 1
        self._tiempo_anterior = tiempo
        self._velocidades_anteriores = velocidades
        self.get_logger().info(
            f"'{self._trayectoria_actual}': {self._filas_guardadas} filas guardadas",
            throttle_duration_sec=PERIODO_LOG_S,
        )

    @staticmethod
    def _derivar(actuales: List[float], anteriores: List[float], intervalo: float) -> List[float]:
        """Derivada numérica de cada articulación: (valor_ahora - valor_antes) / intervalo."""
        derivadas: List[float] = []
        for ahora, antes in zip(actuales, anteriores):
            derivadas.append((ahora - antes) / intervalo)
        return derivadas


def main() -> None:
    """Tarea 1: lanza el suscriptor de las articulaciones."""
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


def main_trayectorias() -> None:
    """Tarea 2: ejecuta las trayectorias Joint, Lineal y Circular."""
    rclpy.init()
    nodo = MultiplesTrayectorias()
    try:
        nodo.ejecutar_trayectorias()
    except KeyboardInterrupt:
        pass
    nodo.ejecutor.shutdown()
    nodo.destroy_node()
    rclpy.try_shutdown()


def main_grabador() -> None:
    """Tarea 3: solo el grabador (junto a trayectorias, en otra terminal)."""
    rclpy.init()
    grabador = GrabadorCinematica()
    try:
        rclpy.spin(grabador)
    except KeyboardInterrupt:
        pass
    grabador.cerrar()
    grabador.destroy_node()
    rclpy.try_shutdown()


def main_practica() -> None:
    """Tareas 2 y 3 a la vez: mueve el robot y guarda un CSV por trayectoria."""
    rclpy.init()
    grabador = GrabadorCinematica()
    trayectorias = MultiplesTrayectorias()
    # El hilo de ControlRobot atiende también los mensajes del grabador,
    # así las trayectorias y la grabación funcionan a la vez en un solo programa
    trayectorias.ejecutor.add_node(grabador)
    try:
        trayectorias.ejecutar_trayectorias()
    except KeyboardInterrupt:
        pass
    trayectorias.ejecutor.shutdown()
    grabador.cerrar()
    grabador.destroy_node()
    trayectorias.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()