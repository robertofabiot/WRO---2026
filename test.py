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
        robot.navegacion.seguidor_linea_cruces_y_distancia(sensor, 80, 1, 0, 10, lado="izquierda", tiempo_acomodo_ms=0)
        robot.navegacion.giro_preciso_pd(-90, encadenado=True)
        robot.garra_trasera.ir_a_porcentaje(97, wait_after=False)
        robot.chasis.avanzar_recto(-32, velocidad=1000)
        robot.chasis.cuadrar_contra_pared(tiempo_ms=100, potencia=80, angulo_referencia=270)
        robot.chasis.avanzar_recto(55)
        robot.navegacion.giro_absoluto_pd(0)

        # Recoger amarillos
        robot.garra_delantera.ir_a_porcentaje(90)
        robot.chasis.avanzar_recto(14)
        robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        robot.garra_delantera.ir_a_porcentaje(0)
        
        robot.navegacion.giro_absoluto_pd(90)
        robot.navegacion.avanzar_tiempo_luego_color(sensor, 0.3, Color.WHITE, distancia_extra_cm=2)
        robot.navegacion.giro_absoluto_pd(180)
        robot.garra_delantera.ir_a_porcentaje(90)
        robot.garra_delantera.ir_a_porcentaje_pinza(67)
        robot.garra_delantera.ir_a_porcentaje(20)

        robot.garra_trasera.ir_a_porcentaje(0, wait_after=False)
        robot.chasis.avanzar_recto(-4)
        robot.navegacion.giro_absoluto_pd(2, kp=3, kd=20)
        robot.garra_delantera.ir_a_porcentaje(90)
        robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        robot.chasis.avanzar_recto(-16)
        robot.garra_delantera.abrir_al_tope(limite_potencia=100)
        robot.garra_delantera.ir_a_porcentaje(0)
        robot.chasis.avanzar_recto(16, velocidad=1000)
        
        robot.navegacion.giro_preciso_pd(38)
        robot.chasis.avanzar_recto(9)
        robot.garra_delantera.ir_a_porcentaje(90)

        robot.navegacion.giro_absoluto_pd(280)
        robot.garra_delantera.ir_a_porcentaje(0)
        robot.chasis.avanzar_recto(20)