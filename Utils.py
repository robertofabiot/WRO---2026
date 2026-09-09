"""Funciones matemáticas y utilidades auxiliares generales.

Contiene operaciones que no dependen directamente del estado del chasis
ni de ningún mecanismo en particular.
"""

from pybricks.tools import wait
import config


class Utils:
    """Colección de métodos utilitarios estáticos."""

    @staticmethod
    def limitar(valor, minimo, maximo):
        """Mantiene un valor dentro de los límites mínimo y máximo indicados."""
        return max(minimo, min(maximo, valor))

    @staticmethod
    def error_angular(objetivo, actual):
        """Calcula la diferencia angular más corta entre dos rumbos (-180° a 180°)."""
        error = objetivo - actual
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    @staticmethod
    def error_angular_absoluto(objetivo, actual):
        """Calcula el camino angular más corto considerando acumulaciones mayores a 360°."""
        return (objetivo - actual + 180) % 360 - 180

    @staticmethod
    def emitir_sonido_confirmacion(hub):
        """Emite un beep de confirmación si config.SONIDO_ACTIVO está habilitado."""
        if config.SONIDO_ACTIVO and hub is not None:
            hub.speaker.beep(frequency=500, duration=100)
