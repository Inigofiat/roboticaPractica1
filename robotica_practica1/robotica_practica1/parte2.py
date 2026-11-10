"""Parte 2 (retos opcionales): ingesta inteligente y mensaje personalizado."""

from typing import Dict, List

import rclpy
from sensor_msgs.msg import JointState

from robotica_practica1.parte1 import (
    ARTICULACIONES,
    INTERVALO_MINIMO_S,
    NANOSEGUNDOS_POR_SEGUNDO,
    PERIODO_LOG_S,
    GrabadorCinematica,
    MultiplesTrayectorias,
)

# Por debajo de esta velocidad (rad/s) una articulación se considera quieta.
# No se usa 0 porque, con el robot parado, la velocidad nunca es exactamente 0.
UMBRAL_VELOCIDAD: float = 1e-3


class GrabadorInteligente(GrabadorCinematica):
    """Grabador de la Parte 1 que solo guarda datos cuando el robot se mueve."""

    def _guardar_muestra(self, mensaje: JointState) -> None:
        """Igual que en la Parte 1, pero ignorando los periodos en que el robot está quieto."""
        if self._escritor_csv is None:
            return
        indice: Dict[str, int] = {}
        for posicion_en_lista, nombre in enumerate(mensaje.name):
            indice[nombre] = posicion_en_lista
        if not all(articulacion in indice for articulacion in ARTICULACIONES):
            return
        if len(mensaje.velocity) != len(mensaje.name):
            return
        posiciones: List[float] = [mensaje.position[indice[a]] for a in ARTICULACIONES]
        velocidades: List[float] = [mensaje.velocity[indice[a]] for a in ARTICULACIONES]

        if not self._esta_en_movimiento(velocidades):
            # Se olvida la muestra anterior para no derivar a través de la pausa
            self._tiempo_anterior = None
            self.get_logger().info(
                "Robot quieto: muestras ignoradas", throttle_duration_sec=PERIODO_LOG_S
            )
            return

        tiempo: float = (
            mensaje.header.stamp.sec
            + mensaje.header.stamp.nanosec / NANOSEGUNDOS_POR_SEGUNDO
        )
        if self._filas_guardadas == 0:
            self._tiempo_inicio = tiempo

        if self._tiempo_anterior is None:
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
    def _esta_en_movimiento(velocidades: List[float]) -> bool:
        """True si al menos una articulación supera UMBRAL_VELOCIDAD (en valor absoluto)."""
        return any(abs(velocidad) > UMBRAL_VELOCIDAD for velocidad in velocidades)


def main_grabador_inteligente() -> None:
    """Solo el grabador de la Parte 2 (junto a trayectorias, en otra terminal)."""
    rclpy.init()
    grabador = GrabadorInteligente()
    try:
        rclpy.spin(grabador)
    except KeyboardInterrupt:
        pass
    grabador.cerrar()
    grabador.destroy_node()
    rclpy.try_shutdown()


def main_parte2() -> None:
    """Trayectorias + grabador de la Parte 2 en un solo programa."""
    rclpy.init()
    grabador = GrabadorInteligente()
    trayectorias = MultiplesTrayectorias()
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
    main_parte2()