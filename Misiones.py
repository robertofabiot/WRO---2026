import gc # Herramienta vital para borrar el lag de memoria
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

    # --- FUNCIONES DE PRUEBA INTACTAS ---
    def prueba_precision(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=21,tiempo_acomodo_ms=100)
        self.robot.mecanismos.garra_delantera.cerrar(grados=630, velocidad=5000, wait_after=False)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.elevador_delantero.mover(grados=-500, velocidad=5000)
        self.robot.chasis.avanzar_recto(15)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope()
        self.robot.mecanismos.elevador_delantero.mover(grados=200)
        
    def pruebas_matrices(self):
        #Avanza como 67cm, gira a la izquierda, avanza 13cm, baja la garra y cuerra la garra. 
        # Retrocede 13cm, gira hacia la izquierda, avanza 60cm en el seguidor, abre la garra 
        # y deja los bloques en un lugar especifico, retrocede y gira hacia la derecha y avanza 13cm,
        # baja y cierra la garra,  
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=5000)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=15, tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.elevador_delantero.mover(-200, wait_after=False)
        self.robot.chasis.avanzar_recto(15.5)
        self.robot.mecanismos.garra_delantera.abrir_al_tope(velocidad=5000)
        self.robot.mecanismos.garra_delantera.abrir(grados=-130, velocidad=4000, wait_after=False)
        self.robot.mecanismos.elevador_delantero.mover(200, wait_after=False)
        self.robot.chasis.avanzar_recto(-17)
        self.robot.navegacion.giro_preciso_pd(-180)
        self.robot.mecanismos.elevador_delantero.mover(-270)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.navegacion.giro_preciso_pd(90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=58, tiempo_acomodo_ms=200)

    # --- MISIONES DE COMPETENCIA OPTIMIZADAS (MODO ASÍNCRONO) ---
    def cemento_y_llana(self):
        # 1. ARRANQUE FLUIDO
        self.robot.chasis.mover_en_arco(radio_cm=14, distancia_cm=17, stop=Stop.NONE, margen_cm=3)

        # 2. SEGUIDOR BLINDADO
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=91, lado="derecha", tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-92)

        # ¡ACCIÓN SIMULTÁNEA! Garra a máxima velocidad (1500) mientras retrocede
        self.robot.mecanismos.garra_trasera.mover(grados=170, velocidad=800, wait_after=False, frenado=Stop.COAST_SMART)
        self.robot.chasis.avanzar_recto(-6, velocidad=1000, frenado=Stop.BRAKE, margen_cm=0)

        self.robot.navegacion.giro_preciso_pd(90)
        self.robot.chasis.avanzar_recto(-27, velocidad=1300, frenado=Stop.NONE, margen_cm=4)
        self.robot.chasis.mover_en_arco(-140, distancia_cm=28, stop=Stop.NONE, margen_cm=1)

        # ¡ACCIÓN SIMULTÁNEA! Mueve chasis asíncrono y entra al seguidor
        self.robot.chasis.mover_motor_izquierdo(160, wait_after=False)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=50, lado="derecha", tiempo_acomodo_ms=0)
        
        self.robot.navegacion.giro_preciso_pd(90)
        gc.collect() # Limpiamos la RAM

    def agarrar_bloques_blancos(self):
        # 1. ARRANQUE ASÍNCRONO EXPLOSIVO
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1500, wait_after=False)
        self.robot.chasis.avanzar_recto(-5, frenado=Stop.HOLD)

        # Encadenamiento fluido
        self.robot.chasis.mover_en_arco(-14.5, distancia_cm=17.5, stop=Stop.NONE, margen_cm=2)
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=16, lado="derecha", tiempo_acomodo_ms=0, kp=1.2, kd=3.5)
        
        self.robot.chasis.drive_base.stop() 
        self.robot.hub.imu.reset_heading(0)
        self.robot.navegacion.giro_preciso_pd(180)
        
        # 5. RECOLECCIÓN LETAL SIMULTÁNEA (Embestimos a velocidad 1000, no a 150)
        self.robot.mecanismos.garra_trasera.mover(172, velocidad=300, wait_after=False)
        self.robot.chasis.avanzar_recto(-19, velocidad=1000, frenado=Stop.HOLD)
        gc.collect()

    def dejar_bloques_blancos(self):
        self.robot.chasis.motor_derecha.hold() 
        self.robot.chasis.mover_motor_izquierdo(400, velocidad=1000, frenado=Stop.HOLD)
        
        self.robot.chasis.drive_base.settings(straight_speed=930, straight_acceleration=1500, turn_rate=config.TURN_RATE, turn_acceleration=config.STRAIGHT_ACCEL)
        
        self.robot.chasis.avanzar_recto(57, velocidad=930, frenado=Stop.HOLD)
        
        self.robot.chasis.drive_base.settings(straight_speed=config.STRAIGHT_SPEED, straight_acceleration=config.STRAIGHT_ACCEL, turn_rate=config.TURN_RATE, turn_acceleration=config.STRAIGHT_ACCEL)
        
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", distancia_cm=35)
        self.robot.chasis.mover_motor_derecho(80, velocidad=800, frenado=Stop.HOLD)        
        
        self.robot.chasis.avanzar_recto(-1, velocidad=300, frenado=Stop.HOLD)

        self.robot.chasis.drive_base.stop() 
        self.robot.hub.imu.reset_heading(0)
        self.robot.navegacion.giro_preciso_pd(235)       
        
        # 7. ENTREGA LETAL SIMULTÁNEA
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1500, wait_after=False)
        self.robot.chasis.avanzar_recto(-18, velocidad=1000, frenado=Stop.HOLD)
        gc.collect()

    def agarrar_bloques_verdes(self):
        self.robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.BRAKE)
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(370, velocidad=1000, margen_grados=30)
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=42, lado="izquierda", tiempo_acomodo_ms=0, kp=1.2, kd=3.5)
        self.robot.navegacion.giro_preciso_pd(180)
        
        # ACCIÓN SIMULTÁNEA MIENTRAS ENTRA
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1500, frenado=Stop.HOLD, wait_after=False)
        self.robot.chasis.avanzar_recto(-19, velocidad=1000)
        gc.collect()

    def dejar_bloques_verdes_y_detectar_mosaico(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=200, distancia_cm=70)
        self.robot.chasis.mover_motor_derecho(80)
        self.robot.chasis.avanzar_recto(15)

        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        if (mosaico == 1 or mosaico == 2):
            self.robot.chasis.avanzar_recto(-5, velocidad=1000)
            
        self.robot.chasis.avanzar_recto(-20, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(180)
        
        # ACCIÓN SIMULTÁNEA 
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1500, wait_after=False)
        self.robot.chasis.avanzar_recto(-6, velocidad=1000)
        gc.collect()
        return mosaico 

    def agarrar_bloques_amarillos(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm=17, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(33, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(45)

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=13, tiempo_acomodo_ms=0) # Eliminamos el acomodo para no perder tiempo
        self.robot.navegacion.giro_preciso_pd(-180)
        
        # ACCIÓN SIMULTÁNEA
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1500, wait_after=False)
        self.robot.chasis.avanzar_recto(-20, velocidad=1000)
        gc.collect()

    def dejar_bloques_amarillos(self):
        self.robot.navegacion.giro_preciso_pd(-55)
        self.robot.chasis.avanzar_recto(distancia_cm=80, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(55)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=55, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-90)
        
        # ACCIÓN SIMULTÁNEA
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1500, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-17, velocidad=1000)
        gc.collect()

    def agarrar_bloques_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(distancia_cm=16, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=47)
        self.robot.navegacion.giro_preciso_pd(-50)
        
        # ACCIÓN SIMULTÁNEA 1
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1500, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-10, velocidad=1000)
        
        # Gatillazo perfecto al vuelo (le agregamos vel_garra=1500 para que sea violento)
        self.robot.mecanismos.garra_trasera.avanzar_y_gatillar(chasis=self.robot.chasis, distancia_total_cm=34, vel_chasis=1000, distancia_trigger_cm=15, grados_garra=-170, vel_garra=1500)
        
        self.robot.navegacion.giro_preciso_pd(55)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor,  velocidad_max=100, distancia_cm=16, tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-180)
        
        # ACCIÓN SIMULTÁNEA 2
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1500, wait_after=False)
        self.robot.chasis.avanzar_recto(-20, velocidad=1000)
        
        self.robot.navegacion.giro_preciso_pd(-30)
        self.robot.chasis.avanzar_recto(distancia_cm=36, velocidad=950)
        self.robot.navegacion.giro_preciso_pd(30, margen_grados=10)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=160, lado="izquierda")  
        gc.collect()

""" --------PARTE DE IDEA OLVIDADA---------
    def agarrar_bloques_amarillos_y_azules(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm=64, lado="izquierda")
        self.robot.chasis.mover_motor_izquierdo(700, frenado=Stop.BRAKE)
        self.robot.mecanismos.garra_trasera.mover(170,velocidad=130, wait_after=False)
        self.robot.chasis.avanzar_recto(-35)
        self.robot.chasis.avanzar_recto(65)
        self.robot.chasis.mover_motor_izquierdo(700)

    def dejar_bloques_amarillos_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(15)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=24 ,margen_cm=30)
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(10)
        self.robot.mecanismos.elevador_delantero.mover(90, wait_after=False)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope()
        self.robot.chasis.avanzar_recto(-10)
        self.robot.navegacion.giro_preciso_pd(45)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=26) 
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.chasis.avanzar_recto(-7)
        self.robot.mecanismos.garra_trasera(-170)
        """