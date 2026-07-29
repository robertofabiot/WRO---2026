from pybricks.parameters import Color, Stop

class ArmadorMosaicos:
    def __init__(self, robot_instancia, sensor_color):
        self.robot = robot_instancia
        self.sensor_color = sensor_color
        
        # --- PATRÓN ESTRATEGIA (Diccionario de Rutinas) ---
        self.rutinas = {
            1: self._armar_verde_verde,
            2: self._armar_verde_amarillo,
            3: self._armar_azul,
            4: self._armar_amarillo,
            5: self._armar_blanco
        }

    def armar(self, numero_mosaico: int):
        rutina_a_ejecutar = self.rutinas.get(numero_mosaico, self._armar_verde_verde)
        print(f"Ejecutando rutina de armado para mosaico: {numero_mosaico}")
        rutina_a_ejecutar()

    # --- RUTINAS PRIVADAS ---
    def _armar_verde_verde(self):
        pass   

    def _armar_verde_amarillo(self):
        pass

    def _armar_azul(self):
        pass

    def _armar_amarillo(self):
        pass

    def _armar_blanco(self):
        pass