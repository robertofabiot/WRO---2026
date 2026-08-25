from robot import Robot
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
            self.robot.hub.speaker.beep(350, 150)
            self.robot.hub.speaker.beep(250, 150)
            self.robot.hub.speaker.beep(150, 150)
            self.robot.hub.speaker.beep(80, 600)
            
            continuar = input("¿Desea continuar? (y/n): ")

            return continuar == "y"

        return True
    
    def _obtener_bateria(self):
        return self.robot.hub.battery.voltage()