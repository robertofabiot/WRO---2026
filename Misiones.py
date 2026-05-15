from pybricks.parameters import Stop, Color
from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot # Importado para el Type Hinting

class Misiones:
    def __init__(self, robot: Robot, sensor_frente: ColorSensor):
        self.robot = robot
        self.sensor = sensor_frente
        self.sensor_trasero = sensor_frente


    

    def _identificar_combinacion(self, sensor: ColorSensor, distancia_si_verde):
        color_principal = sensor.color()
        if color_principal not in config.MOSAICOS:
            return -1
        decision = config.MOSAICOS[color_principal]
        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_si_verde)
            color_anterior = sensor.color()
            if color_anterior not in decision: return -1
            return decision[color_anterior]
        return decision

    def pruebas(self):
        # self.robot.chasis.mover_en_arco(-7, distancia_cm=11, stop=Stop.BRAKE)
        self.robot.navegacion.seguidor_linea_distancia_prueba(self.sensor, velocidad_max=100, distancia_cm=50, lado="derecha", tiempo_acomodo_ms=800, kp=1.2, kd=3.5, k_freno=1.0, margen_cm=0)

    def cemento_y_llana(self):
        self.robot.mecanismos.garra_delantera.llevar_al_tope("positivo", velocidad=1000, limite_potencia=60)
        self.robot.chasis.mover_en_arco(radio_cm=14, distancia_cm=17, stop=Stop.NONE)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 110, 84, tiempo_acomodo_ms=0, margen_cm=10)
        self.robot.navegacion.giro_preciso_pd(-94, margen_grados=5)
        self.robot.mecanismos.garra_trasera.mover(167, velocidad=180, wait_after=False)
        self.robot.chasis.avanzar_recto(-12, velocidad=1000, frenado=Stop.COAST, margen_cm=2)
        wait(1)
        self.robot.navegacion.giro_preciso_pd(90, max_speed=1000, min_speed=40, kp=8.5, kd=115.0)
        wait(1)
        self.robot.chasis.avanzar_recto(-23, velocidad=1300, frenado=Stop.NONE)
        wait(1)
        self.robot.chasis.mover_en_arco(-135, distancia_cm=30, stop=Stop.COAST)
        self.robot.chasis.mover_motor_izquierdo(100)
        wait(1)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 110, 33, tiempo_acomodo_ms=100)
        self.robot.navegacion.giro_preciso_pd(90, max_speed=1000, min_speed=40, kp=8.5, kd=115.0)

    def agarrar_bloques_blancos(self):
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad= 1300, wait_after= False)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.chasis.mover_en_arco(-7, distancia_cm=11, stop=Stop.BRAKE)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm=15, lado="derecha", tiempo_acomodo_ms=800)
        wait(300)
        self.robot.navegacion.giro_preciso_pd(-180)
        wait(100)
        self.robot.chasis.avanzar_recto(-17, velocidad=1000)
        self.robot.mecanismos.garra_trasera.mover(170)

    def dejar_bloques_blancos(self):
        self.robot.chasis.avanzar_recto(5, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_motor_izquierdo(370, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_en_arco(radio_cm=-114, angulo=30, stop=Stop.NONE)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", distancia_cm=17)
        self.robot.chasis.avanzar_recto(-0.5)
        self.robot.navegacion.giro_eje_puro(218, kp=5, kd=10, min_speed=200)
        self.robot.chasis.avanzar_recto(-24, velocidad=1000, margen_cm=2)
        self.robot.mecanismos.garra_trasera.mover(-80, margen_grados=20)

    def detectar_mosaico(self):
        self.robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_motor_derecho(250, margen_grados=30)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 60, 15, tiempo_acomodo_ms=0, kp=0.45, kd=1.8, k_freno=0.8)
        self.robot.chasis.avanzar_recto(-28, velocidad=700)
        self.robot.mecanismos.garra_trasera.mover(45, frenado=Stop.HOLD)
        wait(100)
        mosaico = self._identificar_combinacion(self.sensor_trasero, -5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        if (mosaico == 1 or mosaico == 2):
            self.robot.chasis.avanzar_recto(5)
        return mosaico

    def agarrar_bloques_amarillos(self):
        self.robot.mecanismos.garra_trasera.mover(-50)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 19)
        self.robot.navegacion.giro_preciso_pd(-55)
        self.robot.chasis.avanzar_recto(25)
        self.robot.navegacion.giro_preciso_pd(45)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.YELLOW, distancia_cm=26)
        self.robot.chasis.avanzar_recto(-10, velocidad=1000)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
        self.robot.chasis.giro_preciso(-175)
        self.robot.chasis.mover_motor_derecho(30)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)

    def dejar_bloques_amarillos(self):
        self.robot.chasis.mover_motor_derecho(225, velocidad=1000, margen_grados=30)
        self.robot.chasis.avanzar_recto(60, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_motor_izquierdo(225, velocidad=1000, margen_grados=30)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 58, lado="izquierda", tiempo_acomodo_ms=800)
        wait(500)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.chasis.avanzar_recto(-17, velocidad=1000)
        self.robot.mecanismos.garra_trasera.mover(-55)

    def recoger_bloques_azules(self):
        self.robot.chasis.avanzar_recto(15, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 50)
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(13, velocidad=1000)
        self.robot.chasis.mover_motor_izquierdo(300)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.BLUE, distancia_cm=25)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
        self.robot.chasis.giro_preciso(-175)
        self.robot.chasis.mover_motor_derecho(30)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)

    def dejar_bloques_azules_y_pala(self):
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)
        self.robot.navegacion.giro_preciso_pd(-35)
        self.robot.chasis.avanzar_recto(52, velocidad=1100)
        self.robot.chasis.mover_motor_izquierdo(210)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.chasis.giro_preciso(-180)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.chasis.mover_motor_derecho(-150)
        self.robot.chasis.avanzar_recto(-46)
        self.robot.chasis.mover_motor_izquierdo(-120)
        self.robot.chasis.avanzar_recto(-80)
        self.robot.chasis.avanzar_recto(20, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.garra_trasera.mover(-55)