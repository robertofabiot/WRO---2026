from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot
from ArmadorMosaicos import ArmadorMosaicos
from RevisadorBateria import RevisadorBateria
from Misiones import Misiones

# 1. Inicialización de Hardware
mi_robot = Robot(
    port_izq=config.PORT_MOTOR_IZQ, 
    port_der=config.PORT_MOTOR_DER, 
    port_garra_trasera=config.PORT_GARRA_TRASERA, 
    port_garra_delantera=config.PORT_GARRA_DELANTERA
)
sensor = ColorSensor(config.PORT_SENSOR_FRENTE)

# 2. Controladores de alto nivel
misiones = Misiones(mi_robot, sensor)
armador = ArmadorMosaicos(mi_robot, sensor)
revisador_bateria = RevisadorBateria(mi_robot)

# 3. Flujo Principal
if __name__ == "__main__":
    if not revisador_bateria.revisar_bateria():
        print("Ejecución cancelada.")
    else:
        # --- ZONA DE PRUEBAS: Descomenta la misión que quieras ejecutar ---
        misiones.agarrar_bloques_blancos()
        misiones.dejar_bloques_blancos()
        
        numero_mosaico = misiones.detectar_mosaico()
        
        misiones.agarrar_bloques_verdes()
        misiones.dejar_bloques_verdes()
        misiones.agarrar_bloques_amarillos()
        misiones.dejar_bloques_amarillos()
        
        misiones.cemento_y_llana()
        
        misiones.recoger_bloques_azules()
        misiones.dejar_bloques_azules_y_pala()
    
        """
        Para pruebas completas: quita el '= 1' y usa la variable 'numero_mosaico' 
        devuelta por la función detectar_mosaico()
        """
        armador.armar(numero_mosaico = 1)