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
            
            # Sonido "Gamer" de batería baja (versión mucho más grave)
            self.robot.hub.speaker.beep(350, 150) # Tono medio-bajo
            self.robot.hub.speaker.beep(250, 150) # Tono bajo
            self.robot.hub.speaker.beep(150, 150) # Tono más bajo
            self.robot.hub.speaker.beep(80, 600)  # Tono final muy grave (casi un zumbido)
            
            continuar = input("¿Desea continuar? (y/n): ")

            return continuar == "y"

        return True
    
    def _obtener_bateria(self):
        return self.robot.hub.battery.voltage()