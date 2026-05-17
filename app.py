from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot
from ArmadorMosaicos import ArmadorMosaicos
from RevisadorBateria import RevisadorBateria
from Misiones import Misiones

# 1. Inicialización de Hardware usando el archivo config
mi_robot = Robot(
    port_izq=config.PORT_MOTOR_IZQ,
    port_der=config.PORT_MOTOR_DER,
    port_elevador_delantero=config.PORT_ELEVADOR_DELANTERO,
    port_garra_delantera=config.PORT_GARRA_DELANTERA,
    port_garra_trasera=config.PORT_GARRA_TRASERA
)

# Inicialización de Sensores
sensor_frente = ColorSensor(config.PORT_SENSOR_FRENTE)

# 2. Controladores de alto nivel
misiones = Misiones(mi_robot, sensor_frente)
armador = ArmadorMosaicos(mi_robot, sensor_frente)
revisador_bateria = RevisadorBateria(mi_robot)

# 3. Flujo Principal
if __name__ == "__main__":
    if not revisador_bateria.revisar_bateria():
        print("Ejecución caancelada por batería baja.")
    else:
        # ZONA DE PRUEBAS: Descomenta la misión que quieras ejecutar
        # misiones.cemento_y_llana()
        # misiones.agarrar_bloques_blancos()
        numero_mosaico = misiones.detectar_mosaico()

        misiones.dejar_bloques_blancos()
        # misiones.agarrar_bloques_amarillos()
        # misiones.dejar_bloques_amarillos()
        # misiones.recoger_blppoques_azules()
        # misiones.dejar_bloques_azules_y_pala()
        
        # armador.armar(numero_mosaico=1)