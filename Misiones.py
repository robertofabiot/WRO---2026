from pybricks.parameters import Stop, Color
from pybricks.tools import wait
import config
from robot import Robot

class Misiones:
    def __init__(self, robot : Robot, sensor_frente):
        self.robot = robot
        self.sensor = sensor_frente

    def _identificar_combinacion(self, sensor, distancia_si_verde):
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
    
    def agarrar_bloques_blancos(self):
        self.robot.chasis.mover_en_arco(radio_cm=13, distancia_cm=15, stop=Stop.NONE)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 157, tiempo_acomodo_ms=0)
        self.robot.chasis.avanzar_recto(25, velocidad=1000)
        self.robot.chasis.mover_motor_izquierdo(-514, 1000, frenado=Stop.NONE)
        self.robot.chasis.avanzar_hasta_choque(-100, tiempo_arranque_ms=0)
        self.robot.navegacion.avanzar_hasta_color(self.sensor, Color.BLACK, 500)
        self.robot.chasis.mover_motor_derecho(514, 1000, frenado=Stop.COAST)
        self.__recoger_bloques()

    #MISMA LÓGICA (DE MOMENTO)
    def dejar_bloques_blancos(self):
        self.robot.chasis.avanzar_recto(5, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_motor_izquierdo(370, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_en_arco(radio_cm=-114, angulo=24, stop=Stop.NONE)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", distancia_cm=17)
        self.robot.chasis.avanzar_recto(-0.5)
        self.robot.navegacion.giro_eje_puro(218, kp=5, kd=10, min_speed=200)
        self.robot.chasis.avanzar_recto(-24, velocidad=1000, margen_cm=2)
        self.robot.garra_trasera.subir(80, margen_grados=20)
    
    def detectar_mosaico(self):
        self.robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.girar_sobre_eje(135)
        self.robot.chasis.avanzar_recto(23)
        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        return mosaico

    def agarrar_bloques_verdes(self):
        self.__recoger_bloques()

    def dejar_bloques_verdes(self):
        self.robot.chasis.avanzar_recto(40)
        self.robot.chasis.giro_preciso(180, kp_nuevo=5)
        self.robot.chasis.avanzar_recto(38)
        self.robot.garra_trasera.subir_al_tope(1000, limite_potencia=100)
    
    def agarrar_bloques_amarillos(self):
        self.robot.chasis.avanzar_recto(60, 1000, frenado=Stop.NONE)
        self.robot.chasis.mover_motor_derecho(-514)
        self.robot.chasis.avanzar_hasta_choque(-1000)
        self.robot.navegacion.avanzar_hasta_color(self.sensor, Color.BLACK, 500)
        self.robot.chasis.mover_motor_izquierdo(514, 1000, frenado=Stop.COAST)
        self.__recoger_bloques()    

    #MISMA LÓGICA (DE MOMENTO)
    def dejar_bloques_amarillos(self):
        self.robot.chasis.mover_motor_derecho(225, velocidad=1000, margen_grados=30)
        self.robot.chasis.avanzar_recto(60, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_motor_izquierdo(225, velocidad=1000, margen_grados=30)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 58, lado="izquierda", tiempo_acomodo_ms=800)
        wait(500)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.chasis.avanzar_recto(-17, velocidad=1000)
        self.robot.garra_trasera.subir_al_tope(1000, limite_potencia=100)

    def cemento_y_llana(self):
        self.robot.chasis.avanzar_recto(18)
        self.robot.navegacion.giro_preciso_pd(-180, margen_grados=5)
        self.robot.garra_trasera.bajar(170, velocidad=180, wait_after=False)
        self.robot.chasis.avanzar_recto(-13, velocidad=1000, frenado=Stop.COAST, margen_cm=2)
        self.robot.navegacion.giro_preciso_pd(90, max_speed=1000, min_speed=40, kp=8.5, kd=115.0)
        self.robot.chasis.avanzar_recto(-23, velocidad=1000, frenado=Stop.NONE, margen_cm=3)
        self.robot.chasis.mover_en_arco(-65, distancia_cm=14, stop=Stop.NONE)
        self.robot.chasis.mover_en_arco(19, angulo=22)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 44, tiempo_acomodo_ms=0, margen_cm=8)
        self.robot.garra_trasera.subir(170, 1000, wait_after=False)
        wait(0) #Tiempo de ventaja de la garra para subir
        self.robot.chasis.latigazo()

    #SOLO SE QUITÓ EL ACOMODO Y SE DISMINUYÓ DISTANCIA AL SEGUIDOR
    def recoger_bloques_azules(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 5)
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(13, velocidad=1000)
        self.robot.chasis.mover_motor_izquierdo(300)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.BLUE, distancia_cm=25)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.garra_trasera.subir_al_tope(limite_potencia=100)
        self.robot.chasis.giro_preciso(-175)
        self.robot.chasis.mover_motor_derecho(30)
        self.robot.chasis.avanzar_recto(-20) 
        self.robot.garra_trasera.bajar_al_tope(limite_potencia=100)

    def dejar_bloques_azules_y_pala(self):
        self.robot.garra_trasera.bajar_al_tope(limite_potencia=100)
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
        self.robot.garra_trasera.subir(55)

    def __recoger_bloques(self):
        self.robot.garra_trasera.subir_al_tope(1000, limite_potencia=100)
        self.robot.garra_trasera.bajar(100, 1000, wait_after=False) 
        self.robot.chasis.avanzar_hasta_choque(-70, 60)
        self.robot.garra_trasera.bajar_al_tope(1000, limite_potencia=100)