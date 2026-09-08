"""
Archivo para testear código.
"""


from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot
from ArmadorMosaicos import ArmadorMosaicos
from RevisadorBateria import RevisadorBateria
from Misiones import Misiones
import Utils
from pybricks.parameters import Color

# 1. Inicialización de Hardware
robot = Robot(
    port_izq=config.PORT_MOTOR_IZQ, 
    port_der=config.PORT_MOTOR_DER, 
    port_garra_trasera=config.PORT_GARRA_TRASERA, 
    port_garra_delantera=config.PORT_GARRA_DELANTERA,
    port_pinza=config.PORT_PINZA  # <- Pasas el puerto normal
)

sensor = ColorSensor(config.PORT_SENSOR_FRENTE)

# 2. Controladores de alto nivel
misiones = Misiones(robot, sensor)
armador = ArmadorMosaicos(robot, sensor, prueba=True)
revisador_bateria = RevisadorBateria(robot)

# 3. Flujo Principal
if __name__ == "__main__":
    if not revisador_bateria.revisar_bateria():
        print("Ejecución cancelada.")
    else:
        # Esto es un ejemplo
        robot.chasis.avanzar_recto(10)