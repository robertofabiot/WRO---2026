from robot import Robot
import config

class RevisadorBateria:
    """Chequeo de bateria previo a cada corrida.

    Los lazos DC del proyecto compensan el voltaje, pero por debajo de cierto
    nivel la bateria ya no da la potencia que piden los giros y la corrida
    sale distinta a lo calibrado.
    """

    def __init__(self, robot_instancia: Robot):
        """
        Argumentos:
            robot_instancia: instancia de Robot, de donde sale el hub.
        """
        self.robot = robot_instancia

    def revisar_bateria(self):
        """Avisa si la bateria esta por debajo de config.BATERIA_MINIMA.

        Devuelve True si se puede correr: o la bateria alcanza, o el usuario
        eligio seguir igual.
        """
        bateria = self._obtener_bateria()
        bateria_minima = config.BATERIA_MINIMA
        print(f"Bateria actual = {bateria}")
        
        if bateria < bateria_minima:
            print(f"Batería menor a {bateria_minima}")
            
            # Alerta sonora de bateria baja
            self.robot.hub.speaker.beep(350, 150)
            self.robot.hub.speaker.beep(250, 150)
            self.robot.hub.speaker.beep(150, 150)
            self.robot.hub.speaker.beep(80, 600)
            
            continuar = input("¿Desea continuar? (y/n): ")

            return continuar == "y"

        return True
    
    def _obtener_bateria(self):
        """Voltaje actual del hub, en milivoltios."""
        return self.robot.hub.battery.voltage()
