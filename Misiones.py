from pybricks.parameters import Color
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
        
        # Si la decisión es un diccionario, significa que leyó Verde y necesita desempatar
        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_si_verde)
            wait(50) # Micro-pausa para que el sensor lea sin vibraciones del motor
            color_anterior = sensor.color()
            
            self.robot.chasis.avanzar_recto(-distancia_si_verde)
            
            if color_anterior not in decision: 
                return -1
            return decision[color_anterior]
            
        return decision

    def _recoger_con_jaula_trasera(self, distancia, wait_ms=200):
        self.robot.chasis.avanzar_recto(-distancia, 1000, wait_after=False)
        wait(wait_ms)
        self.robot.garra_trasera.ir_a_porcentaje(97, velocidad=600)

    def cemento_y_llana(self):
        # Agarrar cemento
        self.robot.navegacion.giro_relativo_motor_izquierdo_turbo(90, max_potencia=100, min_potencia=80, encadenado=True)
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(self.sensor, 100, 2, 0, distancia_inicial_cm=35, tiempo_acomodo_ms=0, encadenado=True)
        self.robot.navegacion.giro_relativo_turbo(-90)
        self.robot.garra_trasera.ir_a_porcentaje(100, wait_after=False)
        self.robot.chasis.avanzar_recto(-10, encadenado=True)

        # Dejar llana
        self.robot.navegacion.giro_relativo_turbo(90, encadenado=True)
        self.robot.navegacion.desplazar_lateral_turbo(4, encadenado=True)
        self.robot.chasis.avanzar_recto(-50)

    def recoger_pala(self):
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(60, wait_after=False)
        self.robot.chasis.avanzar_recto(100, encadenado=False)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)

    def dejar_cemento(self):
        self.robot.navegacion.giro_relativo_motor_derecho_turbo(90, min_potencia=70)
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            distancia_total_cm=-30,
            distancia_accion_cm=0,
            accion_callback=lambda: self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000, wait_after=False),
            accion_sec_callback=lambda: self.robot.garra_delantera.ir_a_porcentaje_pinza(20, velocidad=1000, wait_after=False),
            delay_sec_ms=600,  
            margen_cm=2
        )
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=1200)
        self.robot.navegacion.giro_relativo_turbo(175)
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=1000, wait_after=False)
        self.robot.navegacion.giro_relativo_turbo(5)

    def recoger_verdes(self):
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, distancia_ciega_cm=0, color_objetivo=Color.WHITE, distancia_maxima_cm=10, velocidad_escaneo=900, encadenado=True)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, distancia_ciega_cm=2, color_objetivo=Color.BLACK, distancia_maxima_cm=10, velocidad_escaneo=150, distancia_extra_cm=9)
        self.robot.navegacion.giro_relativo_turbo(-90)
        self._recoger_con_jaula_trasera(65, wait_ms=800)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=200, potencia=100)

    def detectar_mosaico(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=0, distancia_cm=70)
        self.robot.navegacion.desplazamiento_lateral_turbo(-2.5, encadenado=True)
        self.robot.navegacion.giro_turbo(0)
        self.robot.chasis.avanzar_recto(10)
        return self._identificar_combinacion(self.sensor, 5)
    
    def dejar_verdes(self):
        self.robot.chasis.avanzar_recto(-35)
        self.robot.navegacion.giro_relativo_turbo(180)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.garra_trasera.ir_a_porcentaje(0, wait_after=False)
    
    def agarrar_amarillos(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=0, distancia_cm=70)
        self.robot.chasis.avanzar_recto(2)
        self.robot.navegacion.giro_absoluto_motor_derecho_turbo(90, encadenado=True)
        self.robot.chasis.avanzar_recto(28)
    
    def agarrar_azules(self):
        self.robot.navegacion.giro_turbo(291)
        self.robot.chasis.avanzar_recto(62)
        self.robot.navegacion.giro_turbo(0)
        self._recoger_con_jaula_trasera(25, wait_ms=300)

    def dejar_amarillos(self):
        self.robot.chasis.avanzar_recto(15)
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=False)
        self.robot.navegacion.desplazar_lateral_turbo(-15, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(60, wait_after=False)
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(self.sensor, 100, 2, distancia_extra_cm=10, distancia_inicial_cm=10, lado="izquierda", tiempo_acomodo_ms=0)
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=True)
        self.robot.navegacion.giro_turbo(90)
        self.robot.chasis.avanzar_recto(20)
    
    def dejar_pala(self):
        self.robot.chasis.avanzar_recto(-20)
        self.robot.navegacion.giro_turbo(0)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.BLUE, lado="izquierda", tiempo_acomodo_ms=0, distancia_cm=70, distancia_maxima_cm=80)