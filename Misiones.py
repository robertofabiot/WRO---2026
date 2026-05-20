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
        if color_principal not in config.MOSAICOS: return -1  
        decision = config.MOSAICOS[color_principal]
        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_si_verde, encadenado=True)
            color_anterior = sensor.color()
            if color_anterior not in decision: return -1
            return decision[color_anterior]
        return decision
    
    def agarrar_bloques_blancos(self):
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=400, potencia=50, angulo_referencia=90)
        self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=True)
        self.robot.chasis.mover_en_arco(radio_cm=13, distancia_cm=15, encadenado=True)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 157, tiempo_acomodo_ms=0, encadenado=True, margen_cm=7)
        self.robot.chasis.mover_motor_izquierdo(300, velocidad=1000, margen_grados=50, encadenado=True)
        self.robot.chasis.avanzar_recto(12, 1000, margen_cm=7, encadenado=True)
        self.robot.navegacion.giro_absoluto_pd(358, max_speed=400) # Cierra el combo fluido
        self.__recoger_bloques(25, 300)

    def dejar_bloques_blancos(self):
        self.robot.chasis.avanzar_recto(7, velocidad=1000, margen_cm=7, encadenado=True)
        self.robot.chasis.mover_motor_izquierdo(370, velocidad=1000, encadenado=True)
        self.robot.chasis.avanzar_recto(47, 1000, margen_cm=7, encadenado=True)
        self.robot.chasis.mover_motor_derecho(370, velocidad=1000, encadenado=True)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 70, Color.GREEN, lado="derecha", distancia_cm=10, encadenado=True)
        self.robot.chasis.avanzar_recto(-0.5, encadenado=False)
        self.robot.navegacion.giro_absoluto_pd(225, max_speed=400, ruta_corta=False, encadenado=False)
        self.robot.chasis.avanzar_recto(-29, velocidad=1000, margen_cm=7)
        self.robot.garra_trasera.subir_al_tope(velocidad=1000, limite_potencia=100)

    def agarrar_bloques_verdes(self):
        self.robot.chasis.avanzar_recto(27, velocidad=1000, encadenado=True)
        self.robot.chasis.mover_motor_derecho(300, velocidad=1000, encadenado=True)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 23, tiempo_acomodo_ms=0, encadenado=True)
        self.robot.chasis.giro_preciso(180)
        self.__recoger_bloques(5, bajar=170)
    
    def detectar_mosaico(self):
        self.robot.chasis.avanzar_recto(88, velocidad=1000)
        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        return mosaico

    def dejar_bloques_verdes(self):
        self.robot.chasis.avanzar_recto(-24, encadenado=True)
        self.robot.chasis.giro_preciso(180, kp_nuevo=3.5)
        self.robot.garra_trasera.subir_al_tope(1000, limite_potencia=100)
    
    def agarrar_bloques_amarillos(self):
        self.robot.chasis.avanzar_recto(60, 1000, encadenado=True)
        self.robot.chasis.mover_motor_izquierdo(514, velocidad=1000)
        self.__recoger_bloques(20)  
    
    def agarrar_bloques_azules(self):
        self.robot.chasis.avanzar_recto(67, 1000)

    def dejar_bloques_amarillos(self):
        self.robot.chasis.mover_motor_izquierdo(514, velocidad=1000, encadenado=True)
        self.robot.chasis.avanzar_recto(50, velocidad=1000, encadenado=True)

    def cemento_y_llana(self):
        self.robot.chasis.avanzar_recto(18, encadenado=True)
        self.robot.navegacion.giro_preciso_pd(-180, margen_grados=5)
        self.robot.garra_trasera.bajar(170, velocidad=180, wait_after=False)
        self.robot.chasis.avanzar_recto(-13, velocidad=1000, margen_cm=2, encadenado=True)
        self.robot.navegacion.giro_preciso_pd(90, max_speed=1000, min_speed=40, kp=8.5, kd=115.0, encadenado=True)
        self.robot.chasis.avanzar_recto(-23, velocidad=1000, margen_cm=3, encadenado=True)
        self.robot.chasis.mover_en_arco(-65, distancia_cm=14, encadenado=True)
        self.robot.chasis.mover_en_arco(19, angulo=22, encadenado=True)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 44, tiempo_acomodo_ms=0, margen_cm=8)
        self.robot.garra_trasera.subir(170, 1000, wait_after=False)
        self.robot.chasis.latigazo()

    def dejar_bloques_azules_y_pala(self):
        self.robot.garra_trasera.bajar_al_tope(limite_potencia=100)
        self.robot.navegacion.giro_preciso_pd(-35, encadenado=True)
        self.robot.chasis.avanzar_recto(52, velocidad=1100, encadenado=True)
        self.robot.chasis.mover_motor_izquierdo(210, encadenado=True)
        self.robot.chasis.avanzar_recto(-10, encadenado=True)
        self.robot.chasis.giro_preciso(-180, encadenado=True)
        self.robot.chasis.avanzar_recto(-10, encadenado=True)
        self.robot.chasis.mover_motor_derecho(-150, encadenado=True)
        self.robot.chasis.avanzar_recto(-46, encadenado=True)
        self.robot.chasis.mover_motor_izquierdo(-120, encadenado=True)
        self.robot.chasis.avanzar_recto(-80, encadenado=True)
        self.robot.chasis.avanzar_recto(20, velocidad=1000, encadenado=True)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.garra_trasera.subir(55)

    def __recoger_bloques(self, distancia, wait_ms=200, bajar=165):
        self.robot.chasis.avanzar_recto(-distancia, 1000, wait_after=False)
        wait(wait_ms)
        self.robot.garra_trasera.bajar(bajar, velocidad=1000)