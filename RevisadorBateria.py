"""Módulo de comprobación del estado de la batería previo a la corrida."""

import config


class RevisadorBateria:
    """Chequeo de batería previo a cada corrida.

    Evita ejecutar el recorrido si el voltaje está por debajo de un umbral
    seguro, ya que las variaciones de potencia afectan el comportamiento
    de los giros y la tracción.
    """

    def __init__(self, robot_instancia):
        """
        Argumentos:
            robot_instancia: Instancia de la clase Robot.
        """
        self.robot = robot_instancia

    def revisar_bateria(self):
        """Comprueba si el voltaje está por encima de config.BATERIA_MINIMA.

        Devuelve True si la batería es suficiente o si el usuario decide
        continuar bajo su propio criterio.
        """
        bateria = self.obtener_voltaje()
        bateria_minima = config.BATERIA_MINIMA
        print("Voltaje actual:", bateria, "mV")

        if bateria < bateria_minima:
            print("ADVERTENCIA: Batería menor a", bateria_minima, "mV")

            # Secuencia sonora de alerta
            try:
                self.robot.hub.speaker.beep(350, 150)
                self.robot.hub.speaker.beep(250, 150)
                self.robot.hub.speaker.beep(150, 150)
                self.robot.hub.speaker.beep(80, 600)
            except Exception:
                pass

            try:
                continuar = input("¿Desea continuar de todas formas? (y/n): ")
                return continuar.strip().lower() == "y"
            except Exception:
                return True

        return True

    def obtener_voltaje(self):
        """Devuelve el voltaje actual del PrimeHub en milivoltios."""
        return self.robot.hub.battery.voltage()
