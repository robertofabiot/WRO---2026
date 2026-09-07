from pybricks.pupdevices import ColorSensor
import config
from robot import Robot
from ArmadorMosaicos import ArmadorMosaicos
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
armador = ArmadorMosaicos(mi_robot, sensor, prueba=True)
revisador_bateria = RevisadorBateria(mi_robot)

if __name__ == "__main__":
    if not revisador_bateria.revisar_bateria():
        print("Ejecución cancelada.")
    else:
        # --- ZONA DE PRUEBAS: Descomenta la misión que quieras ejecutar ---
        # mi_robot.garra_trasera.establecer_cero()

        # misiones.cemento_y_llana()
        # misiones.recoger_pala()
        # misiones.dejar_cemento()
        # misiones.recoger_verdes()

        # numero_mosaico = misiones.detectar_mosaico()

        # misiones.dejar_verdes()
        # misiones.agarrar_amarillos()
        # misiones.agarrar_azules()
        # misiones.dejar_amarillos()
        # misiones.dejar_pala()

        # Para la corrida completa: usa la variable devuelta por detectar_mosaico()
        armador.armar(numero_mosaico = 3)
