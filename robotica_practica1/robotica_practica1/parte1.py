"""Práctica 1: suscriptor (tarea 1), trayectorias (tarea 2) y grabación de datos (tarea 3)."""

from typing import Dict, List, Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import copy
from math import cos, pi, sin
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from robotica_practica1.control_robot import ControlRobot

TOPIC_ARTICULACIONES: str = "/joint_states"
TAMANO_COLA: int = 10
NANOSEGUNDOS_POR_SEGUNDO: float = 1e9

ARTICULACIONES: List[str] = [
    "shoulder_pan_joint",   # base
    "shoulder_lift_joint",  # hombro
    "elbow_joint",          # codo
    "wrist_1_joint",        # muñeca 1
    "wrist_2_joint",        # muñeca 2
    "wrist_3_joint",        # muñeca 3
]

TOPIC_TRAYECTORIA: str = "/trayectoria_actual"
SIN_TRAYECTORIA: str = "" 

MEDIO_PI: float = pi / 2

POSICION_INICIAL: List[float] = [0.0, -MEDIO_PI, -MEDIO_PI, -MEDIO_PI, MEDIO_PI, 0.0]

POSICIONES_JOINT: List[List[float]] = [
    [pi / 4, -MEDIO_PI, -MEDIO_PI, -MEDIO_PI, MEDIO_PI, 0.0],      
    [-pi / 4, -MEDIO_PI, -pi / 3, -2 * pi / 3, MEDIO_PI, pi / 4],  
]

LADO_CUADRADO_M: float = 0.10

RADIO_CIRCULO_M: float = 0.05
PUNTOS_CIRCULO: int = 36

# TAREA 1: suscriptor 
class SuscriptorArticulaciones(Node):
    """Nodo que escucha /joint_states y muestra posición y velocidad del brazo."""

    def __init__(self) -> None:
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
        """Movimiento CIRCULAR: la punta recorre un círculo horizontal."""
        centro: Pose = self.pose_actual()
        circulo: List[Pose] = []
        for i in range(PUNTOS_CIRCULO):
            angulo: float = i * 2 * math.pi / PUNTOS_CIRCULO
            punto: Pose = copy.deepcopy(centro)
            punto.position.x += RADIO_CIRCULO_M * math.cos(angulo)
            punto.position.y += RADIO_CIRCULO_M * math.sin(angulo)
            circulo.append(punto)
        return self.mover_trayectoria(circulo)

def main() -> None:
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()

def main_trayectorias() -> None:
    rclpy.init()
    nodo = MultiplesTrayectorias()
    try:
        nodo.ejecutar_trayectorias()
    except KeyboardInterrupt:
        pass
    nodo.ejecutor.shutdown()
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()