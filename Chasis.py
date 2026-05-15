from pybricks.parameters import Stop
from pybricks.tools import wait
import config
from Utils import Utils # <- Importamos la clase Utils

class Chasis:
    def __init__(self, drive_base, motor_izq, motor_der, hub, velocidad_base):
        self.drive_base = drive_base
        self.motor_izquierda = motor_izq
        self.motor_derecha = motor_der
        self.hub = hub
        self.velocidad_base = velocidad_base

    def avanzar_recto(self, distancia_cm, velocidad=None, frenado=Stop.BRAKE, wait_after=True, margen_cm=0):
        if velocidad is None:
            velocidad = self.velocidad_base
        velocidad = max(min(velocidad, 976), -976)
        distancia_mm = distancia_cm * 10
        
        if wait_after and margen_cm > 0:
            distancia_inicial = self.drive_base.distance()
            margen_mm = abs(margen_cm * 10)
            self.drive_base.straight(distancia_mm, then=frenado, wait=False)
            
            while abs(self.drive_base.distance() - distancia_inicial) < (abs(distancia_mm) - margen_mm):
                if self.drive_base.stalled():
                    break
                wait(2)
        else:
            self.drive_base.straight(distancia_mm, then=frenado, wait=wait_after)
            
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar
    
    def avance_milimetrico(self, milimetros, velocidad=400):
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        wait(100) 
        
        circunferencia = config.DIAMETRO_RUEDA * 3.1416
        grados_a_mover = (milimetros / circunferencia) * 360
        
        self.motor_izquierda.run_angle(velocidad, grados_a_mover, then=Stop.HOLD, wait=False)
        self.motor_derecha.run_angle(velocidad, grados_a_mover, then=Stop.HOLD, wait=True)
        
        wait(50)
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar

    # --- NUEVO MÉTODO PARA ABSOLUTE HEADING ---
    def cuadrar_contra_pared(self, tiempo_ms=1000, potencia=30, angulo_referencia=0, reversa=True):
        """
        Choca contra la pared para alinear físicamente el robot y establece 
        ese punto como el 'angulo_referencia' en el IMU absoluto.
        """
        self.drive_base.stop()
        
        potencia_aplicada = -potencia if reversa else potencia
        
        self.motor_izquierda.dc(potencia_aplicada)
        self.motor_derecha.dc(potencia_aplicada)
        wait(tiempo_ms) 
        
        self.motor_izquierda.brake()
        self.motor_derecha.brake()
        self.hub.imu.reset_heading(angulo_referencia) # Único reinicio permitido del IMU
        
        Utils.emitir_sonido_confirmacion(self.hub)
    
    def chocar_inteligente(self, distancia_acercamiento_cm, velocidad_acercamiento=800, potencia_choque=35, timeout_choque_ms=1500):
        es_reversa = distancia_acercamiento_cm < 0
        potencia_real = -abs(potencia_choque) if es_reversa else abs(potencia_choque)
        
        self.avanzar_recto(distancia_acercamiento_cm, velocidad=velocidad_acercamiento, wait_after=True)
        
        self.avanzar_hasta_choque(
            potencia=potencia_real, 
            umbral_velocidad=20, 
            tiempo_arranque_ms=150, 
            timeout_ms=timeout_choque_ms
        )
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar
            
    def mover_en_arco(self, radio_cm, angulo=None, distancia_cm=None, stop=Stop.HOLD, wait_after=True, margen_grados=0, margen_cm=0):
        radio_mm = radio_cm * 10
        distancia_mm = distancia_cm * 10 if distancia_cm is not None else None

        if wait_after and (margen_grados > 0 or margen_cm > 0):
            self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=False)
            
            if distancia_cm is not None and margen_cm > 0:
                dist_inicial = self.drive_base.distance()
                margen_mm_real = abs(margen_cm * 10)
                meta_mm = abs(distancia_mm)
                while abs(self.drive_base.distance() - dist_inicial) < (meta_mm - margen_mm_real):
                    if self.drive_base.stalled(): break
                    wait(2)
            elif angulo is not None and margen_grados > 0:
                ang_inicial = self.drive_base.angle()
                meta_ang = abs(angulo)
                while abs(self.drive_base.angle() - ang_inicial) < (meta_ang - margen_grados):
                    if self.drive_base.stalled(): break
                    wait(2)
        else:
            self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=wait_after)
            
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar

    def girar_sobre_eje(self, grados, wait_after=True, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_inicial = self.drive_base.angle()
            self.drive_base.turn(grados, wait=False)
            while abs(self.drive_base.angle() - angulo_inicial) < (abs(grados) - margen_grados):
                if self.drive_base.stalled(): break
                wait(2)
        else:
            self.drive_base.turn(grados, wait=wait_after)
            
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar

    # --- MÉTODO CORREGIDO ---
    def giro_preciso(self, angulo_objetivo, kp_nuevo=2.5, tolerancia=1, margen_grados=0):
        # Calculamos la meta de forma relativa sin borrar el mapa mental del robot
        angulo_inicial = self.hub.imu.heading()
        angulo_meta = angulo_inicial + angulo_objetivo
        
        kp = kp_nuevo
        min_speed = 50 
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_meta - angulo_actual
            if abs(error) <= max(tolerancia, margen_grados):
                break
            turn_rate = error * kp
            if turn_rate > 0:
                turn_rate = max(turn_rate, min_speed)
            else:
                turn_rate = min(turn_rate, -min_speed)
            self.drive_base.drive(0, turn_rate)
            wait(10)
        self.drive_base.stop()
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar

    def mover_motor_izquierdo(self, grados, velocidad=500, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_izquierda.angle() + grados
            self.motor_izquierda.run_angle(velocidad, grados, then=frenado, wait=False) 
            while abs(angulo_meta - self.motor_izquierda.angle()) > margen_grados:
                if self.motor_izquierda.stalled(): break
                wait(2)
        else:
            self.motor_izquierda.run_angle(velocidad, grados, then=frenado, wait=wait_after) 
            
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar

    def mover_motor_derecho(self, grados, velocidad=800, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_derecha.angle() + grados
            self.motor_derecha.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor_derecha.angle()) > margen_grados:
                if self.motor_derecha.stalled(): break
                wait(2)
        else:
            self.motor_derecha.run_angle(velocidad, grados, then=frenado, wait=wait_after)
            
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar

    def avanzar_hasta_choque(self, potencia=100, umbral_velocidad=50, tiempo_arranque_ms=300, timeout_ms=None):
        self.drive_base.stop() 
        self.motor_izquierda.dc(potencia)
        self.motor_derecha.dc(potencia)
        wait(tiempo_arranque_ms)
        
        tiempo_transcurrido = tiempo_arranque_ms
        paso_ms = 10 
        
        while True:
            vel_izq = abs(self.motor_izquierda.speed())
            vel_der = abs(self.motor_derecha.speed())
            
            if vel_izq < umbral_velocidad and vel_der < umbral_velocidad:
                break
            
            if timeout_ms is not None and tiempo_transcurrido >= timeout_ms:
                print("Advertencia: Timeout alcanzado en avanzar_hasta_choque")
                break
                
            wait(paso_ms)
            tiempo_transcurrido += paso_ms
            
        self.motor_izquierda.hold()
        self.motor_derecha.hold()
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar
    
    def latigazo(self, grados=45, velocidad_giro=1000, aceleracion=3000):
        settings_originales = self.drive_base.settings()
        angulo_inicial = self.hub.imu.heading()
        
        self.drive_base.settings(
            turn_rate=velocidad_giro,
            turn_acceleration=aceleracion
        )
        
        self.drive_base.turn(grados)
        angulo_post_golpe = self.hub.imu.heading()
        
        grados_de_regreso = angulo_inicial - angulo_post_golpe
        self.drive_base.turn(grados_de_regreso)
        
        self.drive_base.settings(
            straight_speed=settings_originales[0],
            straight_acceleration=settings_originales[1],
            turn_rate=settings_originales[2],
            turn_acceleration=settings_originales[3]
        )
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar
            
    def sacudir(self, iteraciones=5, potencia=100, tiempo_ms=60):
        self.drive_base.stop() 
        for _ in range(iteraciones):
            self.motor_izquierda.dc(potencia)
            self.motor_derecha.dc(-potencia)
            wait(tiempo_ms)
            self.motor_izquierda.dc(-potencia)
            self.motor_derecha.dc(potencia)
            wait(tiempo_ms)
        self.motor_izquierda.brake()
        self.motor_derecha.brake()
        wait(100)
        Utils.emitir_sonido_confirmacion(self.hub) # <- Sonido al terminar

    def compensar_voltaje(self, potencia_deseada):
        voltaje_actual = self.hub.battery.voltage()
        if voltaje_actual == 0: return potencia_deseada
        potencia_compensada = potencia_deseada * (8000 / voltaje_actual)
        return max(-100, min(100, potencia_compensada))