"""Punto de entrada del robot.

El recorrido vive en Misiones, partido en secciones que se pueden correr
sueltas para probar en pista o encadenadas con recorrido_completo().
"""

from pybricks.pupdevices import ColorSensor
import config
from robot import Robot
from RevisadorBateria import RevisadorBateria
from Misiones import Misiones

mi_robot = Robot(
    port_izq=config.PORT_MOTOR_IZQ,
    port_der=config.PORT_MOTOR_DER,
    port_garra_trasera=config.PORT_GARRA_TRASERA,
    port_garra_delantera=config.PORT_GARRA_DELANTERA,
    port_pinza=config.PORT_PINZA
)

sensor = ColorSensor(config.PORT_SENSOR_FRENTE)
misiones = Misiones(mi_robot, sensor)
revisador_bateria = RevisadorBateria(mi_robot)

if __name__ == "__main__":
    if not revisador_bateria.revisar_bateria():
        print("Ejecución cancelada.")
    else:
        # --- ZONA DE PRUEBAS: descomenta lo que quieras ejecutar ---

        # 1. Corrida completa (Reto 1 completo + Matriz 2)
        misiones.recorrido_completo()

        # 2. Secciones individuales del Reto 1:
        # Cada una arranca donde termina la anterior, así que hay que
        # posicionar el robot donde lo dejaría la sección previa.
        # misiones.seccion_1_salida_y_cemento()
        # misiones.seccion_2_dejar_cemento_y_tomar_verdes()
        # matriz = misiones.seccion_3_escanear_matriz_y_dejar_verdes()
        # misiones.seccion_4_amarillos_y_azules()
        # misiones.seccion_5_tomar_pala_y_dejar_amarillos()
        # misiones.seccion_6_retorno_pala_y_acomodo()

        # 3. Recorrido de Matriz 2 suelto:
        # misiones.ejecutar_matriz_2()
        # misiones.dejar_bloques_matriz()
        # misiones.dejar_bloques_matriz2()
