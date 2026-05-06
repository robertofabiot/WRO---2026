from pybricks.parameters import Stop
from pybricks.tools import wait

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

    def girar_sobre_eje(self, grados, wait_after=True, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_inicial = self.drive_base.angle()
            self.drive_base.turn(grados, wait=False)
            while abs(self.drive_base.angle() - angulo_inicial) < (abs(grados) - margen_grados):
                if self.drive_base.stalled(): break
                wait(2)
        else:
            self.drive_base.turn(grados, wait=wait_after)

    def giro_preciso(self, angulo_objetivo, kp_nuevo=2.5, tolerancia=1, margen_grados=0):
        self.hub.imu.reset_heading(0) 
        kp = kp_nuevo
        min_speed = 50 
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_objetivo - angulo_actual
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

    def mover_motor_izquierdo(self, grados, velocidad=500, wait_after=True, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_izquierda.angle() + grados
            self.motor_izquierda.run_angle(velocidad, grados, wait=False)
            while abs(angulo_meta - self.motor_izquierda.angle()) > margen_grados:
                if self.motor_izquierda.stalled(): break
                wait(2)
        else:
            self.motor_izquierda.run_angle(velocidad, grados, wait=wait_after)

    def mover_motor_derecho(self, grados, velocidad=800, wait_after=True, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_derecha.angle() + grados
            self.motor_derecha.run_angle(velocidad, grados, wait=False)
            while abs(angulo_meta - self.motor_derecha.angle()) > margen_grados:
                if self.motor_derecha.stalled(): break
                wait(2)
        else:
            self.motor_derecha.run_angle(velocidad, grados, wait=wait_after)
            
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