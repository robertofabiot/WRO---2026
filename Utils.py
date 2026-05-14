import config

class Utils:
    @staticmethod
    def emitir_sonido_confirmacion(hub):
        if config.SONIDO_ACTIVO:
            hub.speaker.beep(frequency=500, duration=100)