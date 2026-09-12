"""Punto de entrada del robot.

El recorrido vive en Misiones, partido en misiones modulares y atómicas que se
pueden correr sueltas para pruebas en pista o encadenadas con recorrido_completo().
"""

from pybricks.pupdevices import ColorSensor
import config
from robot import Robot
from RevisadorBateria import RevisadorBateria
from Misiones import Misiones
from ArmadorMosaicos import ArmadorMosaicos

mi_robot = Robot(
    port_izq=config.PORT_MOTOR_IZQ,
    port_der=config.PORT_MOTOR_DER,
    port_garra_trasera=config.PORT_GARRA_TRASERA,
    port_garra_delantera=config.PORT_GARRA_DELANTERA,
    port_pinza=config.PORT_PINZA
)

sensor = ColorSensor(config.PORT_SENSOR_FRENTE)
armador = ArmadorMosaicos(mi_robot, sensor)
misiones = Misiones(mi_robot,sensor)
revisador_bateria = RevisadorBateria(mi_robot)

if __name__ == "__main__":
    if not revisador_bateria.revisar_bateria():
        print("Ejecución cancelada.")
    else:
        mi_robot.garra_trasera.establecer_cero()
        mi_robot.garra_delantera.establecer_cero()
        mi_robot.garra_delantera.establecer_cero_pinza()


        # --- ZONA DE PRUEBAS: descomenta lo que quieras ejecutar ---

        # Misiones individuales del Reto 1:
        # Cada una arranca donde termina la anterior.
        # misiones.agarrar_cemento()
        # misiones.dejar_llana()
        # misiones.dejar_cemento()
        # misiones.agarrar_verdes()
        # matriz = misiones.escanear_mosaico()
        # misiones.dejar_verdes()
        # misiones.agarrar_amarillos()
        # misiones.agarrar_azules()
        # misiones.agarrar_pala()
        # misiones.dejar_amarillos()
        # misiones.dejar_pala_y_azules()

        # Recorrido de armado de mosaicos individual:
        armador.armar(numero_mosaico = 2)

        # misiones.pruebas()



