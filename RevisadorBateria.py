from robot import Robot # Importamos la clase para que el editor sepa qué es

class RevisadorBateria:
    def __init__(self, robot_instancia: Robot):
        self.robot = robot_instancia

    def revisar_bateria(self):
        bateria = self._obtener_bateria()
        print(f"Bateria actual = {bateria}")
        if bateria < 8000:
            print("Batería menor a 8000")
            continuar = input("¿Desea continuar? (y/n): ")

            return continuar == "y"

        return True
    
    def _obtener_bateria(self):
        return self.robot.hub.battery.voltage()