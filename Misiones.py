<<<<<<< HEAD
=======
import gc
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d
from pybricks.parameters import Stop, Color
from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
<<<<<<< HEAD
from robot import Robot
=======
from robot import Robot 
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d

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
            self.robot.chasis.avanzar_recto(distancia_si_verde, frenado=Stop.HOLD)
            color_anterior = sensor.color()
            if color_anterior not in decision: return -1
            return decision[color_anterior]
        return decision

<<<<<<< HEAD
 
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
=======
    # --- FUNCIONES DE PRUEBA INTACTAS ---
    def pruebasIndividuales(self):
        self.robot.mecanismos.garra_delantera.cerrar(130)

    def prueba_precision(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=21,tiempo_acomodo_ms=100)
        self.robot.mecanismos.garra_delantera.cerrar(grados=630, velocidad=5000, wait_after=False)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.elevador_delantero.mover(grados=-500, velocidad=5000)
        self.robot.chasis.avanzar_recto(15)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope()
        self.robot.mecanismos.elevador_delantero.mover(grados=200)
        
    def pruebas_matrices(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=60, tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.elevador_delantero.mover(-400, velocidad=1000)
        self.robot.chasis.avanzar_recto(15)
        self.robot.mecanismos.garra_delantera.cerrar(600)
        self.robot.mecanismos.elevador_delantero.mover(200)
        self.robot.chasis.avanzar_recto(-15)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=40)
        self.robot.navegacion.giro_preciso_pd(92)
        self.robot.mecanismos.elevador_delantero.mover(-270)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.navegacion.giro_preciso_pd(90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=58, tiempo_acomodo_ms=200)

    # --- MISIONES DE COMPETENCIA CALIBRADAS ---
    def cemento_y_llana(self):
        self.robot.chasis.mover_en_arco(radio_cm=13, distancia_cm=20, stop=Stop.HOLD, margen_cm=3) # Fix: Frenado seguro
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=89, lado="derecha", tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-92)

        # Fix Asíncrono: Velocidad segura (1000), frenado fuerte (HOLD) y respiro de 20ms
        self.robot.mecanismos.garra_trasera.mover(grados=170, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
        self.robot.chasis.avanzar_recto(-7, velocidad=1000, frenado=Stop.HOLD, margen_cm=0)

        self.robot.navegacion.giro_preciso_pd(87)
        self.robot.chasis.avanzar_recto(-27, velocidad=1100, frenado=Stop.HOLD, margen_cm=4) # Bajamos a 1100 para control
        self.robot.chasis.mover_en_arco(-200, distancia_cm=24, stop=Stop.HOLD, margen_cm=1)

        self.robot.chasis.mover_motor_izquierdo(160, wait_after=False)
        wait(20)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=52, lado="derecha", tiempo_acomodo_ms=0)
        
        self.robot.navegacion.giro_preciso_pd(90)
        gc.collect()
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d

    def agarrar_bloques_blancos(self):
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
        self.robot.chasis.avanzar_recto(-5, frenado=Stop.HOLD)

<<<<<<< HEAD
        # Encadenamiento fluido
        self.robot.chasis.mover_en_arco(-12, distancia_cm=16, stop=Stop.NONE, margen_cm=2)
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=19, lado="izquierda", tiempo_acomodo_ms=0, kp=1.2, kd=3.5)
        
        self.robot.chasis.drive_base.stop() 
    
        self.robot.navegacion.giro_preciso_pd(181)
        
        # 5. RECOLECCIÓN LETAL SIMULTÁNEA (Embestimos a velocidad 1000, no a 150)
        self.robot.mecanismos.garra_trasera.mover(172, velocidad=240, wait_after=False, frenado= Stop.COAST)
        self.robot.chasis.avanzar_recto(-21, velocidad=1000, frenado=Stop.HOLD)
=======
        self.robot.chasis.mover_en_arco(-14, distancia_cm=17.5, stop=Stop.HOLD, margen_cm=2)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=19, lado="derecha", tiempo_acomodo_ms=0, kp=1.2, kd=3.5)
        
        self.robot.chasis.drive_base.stop() 
        self.robot.navegacion.giro_preciso_pd(185)
        
        self.robot.mecanismos.garra_trasera.mover(172, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
        self.robot.chasis.avanzar_recto(-23, velocidad=1000, frenado=Stop.HOLD)
        gc.collect()
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d

    def dejar_bloques_blancos(self):
        self.robot.chasis.motor_derecha.hold() 
        self.robot.chasis.mover_motor_izquierdo(400, velocidad=1000, frenado=Stop.HOLD)
        
        self.robot.chasis.drive_base.settings(straight_speed=930, straight_acceleration=1500, turn_rate=config.TURN_RATE, turn_acceleration=config.STRAIGHT_ACCEL)
<<<<<<< HEAD
        
        self.robot.chasis.avanzar_recto(52.5, velocidad=930, frenado=Stop.HOLD)
        
        self.robot.chasis.drive_base.settings(straight_speed=config.STRAIGHT_SPEED, straight_acceleration=config.STRAIGHT_ACCEL, turn_rate=config.TURN_RATE, turn_acceleration=config.STRAIGHT_ACCEL)
        
        self.robot.navegacion.giro_preciso_pd(-55)
=======
        self.robot.chasis.avanzar_recto(55, velocidad=930, frenado=Stop.HOLD) 
        self.robot.chasis.drive_base.settings(straight_speed=config.STRAIGHT_SPEED, straight_acceleration=config.STRAIGHT_ACCEL, turn_rate=config.TURN_RATE, turn_acceleration=config.STRAIGHT_ACCEL)
        
        self.robot.navegacion.giro_preciso_pd(-47)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", distancia_cm=35)
        self.robot.chasis.mover_motor_derecho(80, velocidad=800, frenado=Stop.HOLD)        
        self.robot.chasis.avanzar_recto(-1, velocidad=300, frenado=Stop.HOLD)
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d

        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha",distancia_cm=60)
        
        self.robot.chasis.mover_motor_derecho(75, velocidad=800, frenado=Stop.HOLD)        
        
        self.robot.chasis.drive_base.stop() 
        self.robot.hub.imu.reset_heading(0)
<<<<<<< HEAD
        
        self.robot.navegacion.giro_preciso_pd(235.5, max_speed=700)       
        
        # 7. ENTREGA LETAL SIMULTÁNEA
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=500, wait_after=False)
=======
        self.robot.navegacion.giro_preciso_pd(235, max_speed=700)       
        
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d
        self.robot.chasis.avanzar_recto(-18, velocidad=1000, frenado=Stop.HOLD)

    def agarrar_bloques_verdes(self):
        self.robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.HOLD) 
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(370, velocidad=1300, margen_grados=30)
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=42, lado="izquierda", tiempo_acomodo_ms=0, kp=1.2, kd=3.5)
<<<<<<< HEAD
        self.robot.navegacion.giro_preciso_pd(181)
        
        # ACCIÓN SIMULTÁNEA MIENTRAS ENTRA
        self.robot.mecanismos.garra_trasera.mover(171, velocidad=200, frenado=Stop.COAST, wait_after=False)
        self.robot.chasis.avanzar_recto(-25, velocidad=1000)

    def dejar_bloques_verdes_y_detectar_mosaico(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=200, distancia_cm=95)
        self.robot.chasis.mover_motor_derecho(75)
        self.robot.chasis.avanzar_recto(16)
=======
        self.robot.navegacion.giro_preciso_pd(183)
        
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1000, frenado=Stop.HOLD, wait_after=False)
        wait(20)
        self.robot.chasis.avanzar_recto(-25, velocidad=1000, frenado=Stop.HOLD)
        gc.collect()

    def dejar_bloques_verdes_y_detectar_mosaico(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=200, distancia_cm=70)
        self.robot.chasis.mover_motor_derecho(80)
        self.robot.chasis.avanzar_recto(16, frenado=Stop.HOLD) 
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d

        mosaico = self._identificar_combinacion(self.sensor, 5)
        
        if (mosaico == 1 or mosaico == 2):
            self.robot.chasis.avanzar_recto(-5, velocidad=1000, frenado=Stop.HOLD)
            
<<<<<<< HEAD
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
=======
        self.robot.chasis.avanzar_recto(-34, velocidad=1000, frenado=Stop.HOLD)
        self.robot.navegacion.giro_preciso_pd(185)
        
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
        self.robot.chasis.avanzar_recto(-18, velocidad=500, frenado=Stop.HOLD)
        gc.collect()
        return mosaico 

    def agarrar_bloques_amarillos(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm= 19, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(31, velocidad=1000, frenado=Stop.HOLD)
        self.robot.navegacion.giro_preciso_pd(48)

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=13, tiempo_acomodo_ms=0) 
        self.robot.navegacion.giro_preciso_pd(-180)
        
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
        self.robot.chasis.avanzar_recto(-19, velocidad=1000, frenado=Stop.HOLD)
        gc.collect()

    def dejar_bloques_amarillos(self):
        self.robot.navegacion.giro_preciso_pd(-60)
        self.robot.chasis.avanzar_recto(distancia_cm=78, velocidad=1000, frenado=Stop.HOLD)
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d
        self.robot.navegacion.giro_preciso_pd(60)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=63, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-90)
        
<<<<<<< HEAD
        # ACCIÓN SIMULTÁNEA
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=500, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-17, velocidad=1000)

    def agarrar_bloques_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(distancia_cm=16.2, velocidad=1000)
=======
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
        self.robot.chasis.avanzar_recto(distancia_cm=-17, velocidad=1000, frenado=Stop.HOLD)
        gc.collect()

    def agarrar_bloques_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(distancia_cm=15, velocidad=1000, frenado=Stop.HOLD)
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=47)
        self.robot.navegacion.giro_preciso_pd(-55)
        
<<<<<<< HEAD
        # ACCIÓN SIMULTÁNEA 1
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=300, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-13, velocidad=1000)
        
        # Gatillazo perfecto al vuelo (le agregamos vel_garra=1500 para que sea violento)
        self.robot.mecanismos.garra_trasera.avanzar_y_gatillar(chasis=self.robot.chasis, distancia_total_cm=35, vel_chasis=1000, distancia_trigger_cm=14, grados_garra=-168, vel_garra=1500)
=======
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
        self.robot.chasis.avanzar_recto(distancia_cm=-14, velocidad=1000, frenado=Stop.HOLD)
        
        # El gatillazo lo mantenemos brutal pero firme
        self.robot.mecanismos.garra_trasera.avanzar_y_gatillar(chasis=self.robot.chasis, distancia_total_cm=35, vel_chasis=1000, distancia_trigger_cm=15, grados_garra=-169, vel_garra=1500)
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d
        
        self.robot.navegacion.giro_preciso_pd(55)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor,  velocidad_max=100, distancia_cm=16, tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-182)
        
<<<<<<< HEAD
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
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=15, tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.elevador_delantero.mover(-400, velocidad=1000, wait_after= False)
        self.robot.chasis.avanzar_recto(15)
        self.robot.mecanismos.garra_delantera.cerrar(600, wait_after=False)
        self.robot.mecanismos.elevador_delantero.mover(200)
        self.robot.chasis.avanzar_recto(-15)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=40)
        self.robot.navegacion.giro_preciso_pd(92)
        


=======
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1000, wait_after=False, frenado=Stop.HOLD)
        wait(20)
        self.robot.chasis.avanzar_recto(-22, velocidad=1000, frenado=Stop.HOLD)
        
        self.robot.navegacion.giro_preciso_pd(-30)
        self.robot.chasis.avanzar_recto(distancia_cm=36, velocidad=950, frenado=Stop.HOLD)
        self.robot.navegacion.giro_preciso_pd(30, margen_grados=10)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=160, lado="izquierda")  
        self.robot.chasis.avanzar_recto(distancia_cm=-25, frenado=Stop.HOLD)
        self.robot.navegacion.giro_preciso_pd(90)
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, frenado=Stop.HOLD)
        gc.collect()
>>>>>>> 1cb8a7c2ec8fff0b2957d7596e99f14007eeec6d
