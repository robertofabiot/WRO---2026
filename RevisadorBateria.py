from robot import Robot # Importamos la clase para que el editor sepa qué es
import config

class RevisadorBateria:
    def __init__(self, robot_instancia: Robot):
        self.robot = robot_instancia

    def revisar_bateria(self):
        bateria = self._obtener_bateria()
        bateria_minima = config.BATERIA_MINIMA
        print(f"Bateria actual = {bateria}")
        if bateria < bateria_minima:
            print(f"Batería menor a {bateria_minima}")
            continuar = input("¿Desea continuar? (y/n): ")

            return continuar == "y"

        return True
    
    def _obtener_bateria(self):
        return self.robot.hub.battery.voltage()