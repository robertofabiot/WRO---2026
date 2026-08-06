from pybricks.parameters import Stop, Color
from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot

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

 
    # --- MISIONES DE COMPETENCIA OPTIMIZADAS (MODO ASÍNCRONO) ---
    def cemento_y_llana(self):
        # 1. ARRANQUE FLUIDO
        self.robot.chasis.mover_en_arco(radio_cm=14, distancia_cm=20.5, stop=Stop.NONE, margen_cm=3)

        # 2. SEGUIDOR BLINDADO
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=89, lado="derecha", tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-92)

        # ¡ACCIÓN SIMULTÁNEA! Garra a máxima velocidad (1500) mientras retrocede
        self.robot.mecanismos.garra_trasera.mover(grados=170, velocidad=250, wait_after=False, frenado=Stop.COAST_SMART)
        self.robot.chasis.avanzar_recto(-7, velocidad=1000, frenado=Stop.BRAKE, margen_cm=0)

        self.robot.navegacion.giro_preciso_pd(90)
        self.robot.chasis.avanzar_recto(-22, velocidad=1300, frenado=Stop.NONE, margen_cm=4)
        self.robot.chasis.mover_en_arco(-210, distancia_cm=24, stop=Stop.NONE, margen_cm=1)

        # ¡ACCIÓN SIMULTÁNEA! Mueve chasis asíncrono y entra al seguidor
        self.robot.chasis.mover_motor_izquierdo(165, wait_after=False)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=51, lado="derecha", tiempo_acomodo_ms=0)
        
        self.robot.navegacion.giro_preciso_pd(90)

    def agarrar_bloques_blancos(self):
        # 1. ARRANQUE ASÍNCRONO EXPLOSIVO
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1500, wait_after=False)
        self.robot.chasis.avanzar_recto(-5, frenado=Stop.HOLD)

        # Encadenamiento fluido
        self.robot.chasis.mover_en_arco(-12, distancia_cm=16, stop=Stop.NONE, margen_cm=2)
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=19, lado="izquierda", tiempo_acomodo_ms=0, kp=1.2, kd=3.5)
        
        self.robot.chasis.drive_base.stop() 
    
        self.robot.navegacion.giro_preciso_pd(181)
        
        # 5. RECOLECCIÓN LETAL SIMULTÁNEA (Embestimos a velocidad 1000, no a 150)
        self.robot.mecanismos.garra_trasera.mover(172, velocidad=240, wait_after=False, frenado= Stop.COAST)
        self.robot.chasis.avanzar_recto(-21, velocidad=1000, frenado=Stop.HOLD)

    def dejar_bloques_blancos(self):
        self.robot.chasis.motor_derecha.hold() 
        self.robot.chasis.mover_motor_izquierdo(400, velocidad=1000, frenado=Stop.HOLD)
        
        self.robot.chasis.drive_base.settings(straight_speed=930, straight_acceleration=1500, turn_rate=config.TURN_RATE, turn_acceleration=config.STRAIGHT_ACCEL)
        
        self.robot.chasis.avanzar_recto(52.5, velocidad=930, frenado=Stop.HOLD)
        
        self.robot.chasis.drive_base.settings(straight_speed=config.STRAIGHT_SPEED, straight_acceleration=config.STRAIGHT_ACCEL, turn_rate=config.TURN_RATE, turn_acceleration=config.STRAIGHT_ACCEL)
        
        self.robot.navegacion.giro_preciso_pd(-55)

        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha",distancia_cm=60)
        
        self.robot.chasis.mover_motor_derecho(75, velocidad=800, frenado=Stop.HOLD)        
        
        self.robot.chasis.drive_base.stop() 
        self.robot.hub.imu.reset_heading(0)
        
        self.robot.navegacion.giro_preciso_pd(235.5, max_speed=700)       
        
        # 7. ENTREGA LETAL SIMULTÁNEA
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=500, wait_after=False)
        self.robot.chasis.avanzar_recto(-18, velocidad=1000, frenado=Stop.HOLD)

    def agarrar_bloques_verdes(self):
        self.robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.BRAKE)
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(370, velocidad=1300, margen_grados=30)
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=42, lado="izquierda", tiempo_acomodo_ms=0, kp=1.2, kd=3.5)
        self.robot.navegacion.giro_preciso_pd(181)
        
        # ACCIÓN SIMULTÁNEA MIENTRAS ENTRA
        self.robot.mecanismos.garra_trasera.mover(171, velocidad=200, frenado=Stop.COAST, wait_after=False)
        self.robot.chasis.avanzar_recto(-25, velocidad=1000)

    def dejar_bloques_verdes_y_detectar_mosaico(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=200, distancia_cm=95)
        self.robot.chasis.mover_motor_derecho(75)
        self.robot.chasis.avanzar_recto(16)

        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        if (mosaico == 1 or mosaico == 2):
            self.robot.chasis.avanzar_recto(-5, velocidad=1000)
            
        self.robot.chasis.avanzar_recto(-34, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(181)
        
        # ACCIÓN SIMULTÁNEA 
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=700, wait_after=False)
        self.robot.chasis.avanzar_recto(-20, velocidad=500)
        return mosaico 

    def agarrar_bloques_amarillos(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm= 18, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(31, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(48)

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=12, tiempo_acomodo_ms=0) # Eliminamos el acomodo para no perder tiempo
        self.robot.navegacion.giro_preciso_pd(-180)
        
        # ACCIÓN SIMULTÁNEA
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=240, wait_after=False)
        self.robot.chasis.avanzar_recto(-19, velocidad=1000)

    def dejar_bloques_amarillos(self):
        self.robot.navegacion.giro_preciso_pd(-60)
        self.robot.chasis.avanzar_recto(distancia_cm=76, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(60)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=63, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-90)
        
        # ACCIÓN SIMULTÁNEA
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=500, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-17, velocidad=1000)

    def agarrar_bloques_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(distancia_cm=16.2, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=47)
        self.robot.navegacion.giro_preciso_pd(-55)
        
        # ACCIÓN SIMULTÁNEA 1
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=300, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-13, velocidad=1000)
        
        # Gatillazo perfecto al vuelo (le agregamos vel_garra=1500 para que sea violento)
        self.robot.mecanismos.garra_trasera.avanzar_y_gatillar(chasis=self.robot.chasis, distancia_total_cm=35, vel_chasis=1000, distancia_trigger_cm=14, grados_garra=-168, vel_garra=1500)
        
        self.robot.navegacion.giro_preciso_pd(55)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor,  velocidad_max=100, distancia_cm=16, tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-182)
        
        # ACCIÓN SIMULTÁNEA 2
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=240, wait_after=False)
        self.robot.chasis.avanzar_recto(-22, velocidad=1000)
        
        self.robot.navegacion.giro_preciso_pd(-30)
        self.robot.chasis.avanzar_recto(distancia_cm=34.5, velocidad=950)
        self.robot.navegacion.giro_preciso_pd(30, margen_grados=10)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=155, lado="izquierda")  
        self.robot.chasis.avanzar_recto(distancia_cm=-21)
        self.robot.navegacion.giro_preciso_pd(90)
        self.robot.mecanismos.garra_trasera.mover(-170)

    def pruebas_matrizAmarilloBlancoVerde(self):
        self.robot.mecanismos.garra_delantera.abrir_al_tope(wait_after=False)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=15, tiempo_acomodo_ms=0)
        self.robot.mecanismos.elevador_delantero.mover(-400, velocidad=1000, wait_after= False)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.chasis.avanzar_recto(15)
        self.robot.mecanismos.garra_delantera.cerrar(800, wait_after=False)
        self.robot.mecanismos.elevador_delantero.mover(200)
        self.robot.chasis.avanzar_recto(-13)
        self.robot.navegacion.giro_preciso_pd(90.5)
        self.robot.chasis.mover_motor_izquierdo(200)
        self.robot.chasis.mover_motor_derecho(200)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=40)
        self.robot.navegacion.giro_preciso_pd(-90)
  
    def ejecutar_matriz_4(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=38)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.elevador_delantero.mover(-600, velocidad=600, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=23.5)
        self.robot.chasis.avanzar_recto(-23)
        self.robot.navegacion.giro_preciso_pd(90)
        self.robot.mecanismos.garra_delantera.cerrar(700, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=5)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=100, wait_after=False)
        self.robot.chasis.avanzar_recto(30)
        self.robot.chasis.avanzar_recto(-30)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=15)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=)

        