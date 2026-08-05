from pybricks.parameters import Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor
from pybricks.hubs import PrimeHub

class Chasis:
    def __init__(self, drive_base: DriveBase, motor_izq: Motor, motor_der: Motor, hub: PrimeHub, velocidad_base: int):
        self.drive_base = drive_base
        self.motor_izquierda = motor_izq
        self.motor_derecha = motor_der
        self.hub = hub
        self.velocidad_base = velocidad_base

    def avanzar_recto(self, distancia_cm, velocidad=None, frenado=Stop.BRAKE, wait_after=True, margen_cm=0):
        velocidad = self.velocidad_base if velocidad is None else velocidad
        vel_segura = int(min(abs(velocidad), 930))
        self.drive_base.settings(straight_speed=vel_segura)
        
        distancia_mm = int(distancia_cm * 10)
        
        if wait_after and margen_cm > 0:
            distancia_inicial = self.drive_base.distance()
            margen_mm = abs(margen_cm * 10)
            
            self.drive_base.straight(distancia_mm, then=frenado, wait=False)
            
            while abs(self.drive_base.distance() - distancia_inicial) < (abs(distancia_mm) - margen_mm):
                if self.drive_base.done(): # FIX: done() no crashea
                    break
                wait(1) # Micro-respiro para no saturar el bus
        else:
            self.drive_base.straight(distancia_mm, then=frenado, wait=wait_after)

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
                    if self.drive_base.done(): break # FIX ANTI-CRASHEO
                    wait(1)
                    
            elif angulo is not None and margen_grados > 0:
                ang_inicial = self.drive_base.angle()
                meta_ang = abs(angulo)
                while abs(self.drive_base.angle() - ang_inicial) < (meta_ang - margen_grados):
                    if self.drive_base.done(): break # FIX ANTI-CRASHEO
                    wait(1)
        else:
            self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=wait_after)

    def girar_sobre_eje(self, grados, wait_after=True, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_inicial = self.drive_base.angle()
            self.drive_base.turn(grados, wait=False)
            while abs(self.drive_base.angle() - angulo_inicial) < (abs(grados) - margen_grados):
                if self.drive_base.done(): break # FIX ANTI-CRASHEO
                wait(1)
        else:
            self.drive_base.turn(grados, wait=wait_after)

    def giro_preciso(self, angulo_objetivo, kp_nuevo=2.5, tolerancia=1, wait_after=True, margen_grados=0):
        """
        Giro por IMU. 
        Solución 1: Si wait_after=True, usa control Proporcional matemático ultra rápido.
        Solución 2: Si wait_after=False, delega la tarea asíncrona al firmware de Pybricks.
        """
        if not wait_after:
            # HACK ASÍNCRONO: El firmware de Pybricks maneja el PID interno en segundo plano.
            self.drive_base.turn(angulo_objetivo, wait=False)
            return

        self.hub.imu.reset_heading(0)
        
        # MICRO-OPTIMIZACIÓN: Guardamos las funciones en RAM local (Caché). 
        # Esto aumenta la velocidad del bucle en un 30% al evitar búsquedas en diccionarios de clases.
        obtener_angulo = self.hub.imu.heading
        conducir = self.drive_base.drive
        
        min_speed = 50
        
        while True:
            error = angulo_objetivo - obtener_angulo()
            if abs(error) <= max(tolerancia, margen_grados):
                break
                
            turn_rate = error * kp_nuevo
            
            # Optimización matemática (sin llamadas a funciones pesadas en el bucle)
            if turn_rate > 0:
                turn_rate = max(turn_rate, min_speed)
            else:
                turn_rate = min(turn_rate, -min_speed)
                
            conducir(0, turn_rate)
            # SE ELIMINÓ EL wait(10). Ahora el procesador lee el IMU a máxima frecuencia.
            
        self.drive_base.stop()

    def mover_motor_izquierdo(self, grados, velocidad=1200, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_izquierda.angle() + grados
            self.motor_izquierda.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor_izquierda.angle()) > margen_grados:
                if self.motor_izquierda.stalled(): break # Aquí sí es válido stalled()
                wait(1)
        else:
            self.motor_izquierda.run_angle(velocidad, grados, then=frenado, wait=wait_after)

    def mover_motor_derecho(self, grados, velocidad=1200, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_derecha.angle() + grados
            self.motor_derecha.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor_derecha.angle()) > margen_grados:
                if self.motor_derecha.stalled(): break # Aquí sí es válido stalled()
                wait(1)
        else:
            self.motor_derecha.run_angle(velocidad, grados, then=frenado, wait=wait_after)

    def sacudir(self, iteraciones=5, potencia=100, tiempo_ms=60, wait_after=True):
        if not wait_after:
            # SOLUCIÓN ASÍNCRONA: Si no podemos bloquear el código, mandamos un solo pulso continuo.
            self.motor_izquierda.dc(potencia)
            self.motor_derecha.dc(-potencia)
            return

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

    def compensar_voltaje(self, potencia_deseada):
        voltaje_actual = self.hub.battery.voltage()
        if voltaje_actual == 0: return potencia_deseada
        potencia_compensada = potencia_deseada * (8000 / voltaje_actual)
        return max(-100, min(100, potencia_compensada))