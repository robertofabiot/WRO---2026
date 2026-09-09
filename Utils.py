import config

class Utils:
    @staticmethod
    def emitir_sonido_confirmacion(hub):
        """Emite el beep con el que cada movimiento avisa que termino.

        Se puede apagar entero desde config.SONIDO_ACTIVO: el beep bloquea el
        procesador y arruina el encadenado de movimientos rapidos.

        Argumentos:
            hub: PrimeHub, de donde sale el parlante.
        """
        if config.SONIDO_ACTIVO:
            hub.speaker.beep(frequency=500, duration=100)
