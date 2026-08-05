from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot
from ArmadorMosaicos import ArmadorMosaicos
from RevisadorBateria import RevisadorBateria
from Misiones import Misiones
import gc 

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
        print("Ejecución cancelada por batería baja.")
    else:
        #, # ZONA DE PRUEBAS: Descomenta la misión que quieras ejecutar
        # misiones.prueba_precision()

        # misiones.pruebasIndividuales()

        # 1. Primera Misión
        misiones.cemento_y_llana()
        gc.collect() # Limpiamos la RAM de las variables temporales de la misión anterior

        # 2. Segunda Misión
        misiones.agarrar_bloques_blancos()
        gc.collect()

        # 3. Tercera Misión
        misiones.dejar_bloques_blancos()
        gc.collect()

        # 4. Cuarta Misión
        misiones.agarrar_bloques_verdes()
        gc.collect()

        # 5. Quinta Misión (Retorna valor)
        numero_mosaico = misiones.dejar_bloques_verdes_y_detectar_mosaico()
        gc.collect() # Súper importante aquí porque el escaneo generó variables en RAM

        # 6. Sexta Misión
        misiones.agarrar_bloques_amarillos()
        gc.collect()

        # 7. Séptima Misión
        misiones.dejar_bloques_amarillos()
        gc.collect()

        # 8. Octava Misión (Ataque a la pala)
        misiones.agarrar_bloques_azules_y_pala()
        gc.collect()

        # 9. Novena misión (AmarilloBlancoVerde)
        misiones.pruebas_matrizAmarilloBlancoVerde()
        gc.collect()