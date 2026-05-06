from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Stop, Color
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import config

class Robot:
    def __init__(self, port_izq, port_der, port_eje_central, port_garra_delantera): 
        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        
        # Motores de tracción
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE)
        self.motor_derecha = Motor(port_der, Direction.CLOCKWISE)
        
        # Motores de mecanismos
        self.motor_eje_central = Motor(port_eje_central, Direction.COUNTERCLOCKWISE)
        self.motor_garra_delantera = Motor(port_garra_delantera)
        
        # Se reemplazan los números quemados (56, 160, etc.) por config
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, config.DIAMETRO_RUEDA, config.SEPARACION_RUEDAS)
        self.drive_base.use_gyro(True)
        
        self.VELOCIDAD_BASE = config.VELOCIDAD_BASE
        self.drive_base.settings(straight_speed=config.STRAIGHT_SPEED, straight_acceleration=config.STRAIGHT_ACCEL, turn_rate=config.TURN_RATE)
        
    # region CHASIS - Movimiento Básico
    def avanzar_recto(self, distancia_cm, velocidad=None, frenado=Stop.BRAKE, wait_after=True, margen_cm=0):
        if velocidad is None:
            velocidad = self.VELOCIDAD_BASE
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
    # endregion

    # region CHASIS - Movimiento Complejo y Seguidores
    def giro_preciso_pd(self, angulo_relativo, max_speed=800, min_speed=40, kp=4.0, kd=18.0, margen_grados=0):
        angulo_inicial = self.hub.imu.heading()
        angulo_meta = angulo_inicial + angulo_relativo
        tolerancia = 1    
        error_previo = 0
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_meta - angulo_actual
            if abs(error) <= max(tolerancia, margen_grados):
                break
            derivada = error - error_previo
            turn_rate = (error * kp) + (derivada * kd)
            if turn_rate > 0:
                turn_rate = min(max(turn_rate, min_speed), max_speed)
            else:
                turn_rate = max(min(turn_rate, -min_speed), -max_speed)
            self.drive_base.drive(0, turn_rate)
            error_previo = error
            wait(10)
        self.drive_base.stop()

    def giro_eje_puro(self, angulo_relativo, kp=3.5, kd=15.0, max_speed=600, min_speed=30, margen_grados=0):
        self.drive_base.stop()
        angulo_inicial = self.hub.imu.heading()
        angulo_meta = angulo_inicial + angulo_relativo
        tolerancia = 1    
        error_previo = 0
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_meta - angulo_actual
            if abs(error) <= max(tolerancia, margen_grados):
                break
            derivada = error - error_previo
            magnitud = abs((error * kp) + (derivada * kd))
            velocidad_giro = max(min_speed, min(magnitud, max_speed))
            if error > 0:
                self.motor_izquierda.run(velocidad_giro)
                self.motor_derecha.run(-velocidad_giro)
            else:
                self.motor_izquierda.run(-velocidad_giro)
                self.motor_derecha.run(velocidad_giro)
            error_previo = error
            wait(10)
        self.motor_izquierda.hold()
        self.motor_derecha.hold()

    def seguidor_linea_distancia(self, sensor_color, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp = 0.85, kd = 2.5, k_freno = 0.6, margen_cm=0):
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_objetivo = (distancia_cm / circunferencia) * 360
        grados_margen = (margen_cm / circunferencia) * 360 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)
        
        self.motor_izquierda.reset_angle(0)
        self.motor_derecha.reset_angle(0)
        cronometro = StopWatch()
        velocidad_minima = 25
        tiempo_aceleracion_ms = 0 
        last_error = 0
        objetivo_reflexion = 35
        multiplicador_lado = 1 if lado == "derecha" else -1
        cronometro.reset()
        cronometro.resume()
        
        while True:
            grados_actuales = (abs(self.motor_izquierda.angle()) + abs(self.motor_derecha.angle())) / 2
            if grados_actuales >= grados_objetivo_real:
                break
            tiempo_actual = cronometro.time()
            if tiempo_actual < tiempo_acomodo_ms:
                velocidad_actual = velocidad_minima
            elif tiempo_actual < (tiempo_acomodo_ms + tiempo_aceleracion_ms):
                tiempo_en_rampa = tiempo_actual - tiempo_acomodo_ms
                progreso = tiempo_en_rampa / tiempo_aceleracion_ms if tiempo_aceleracion_ms > 0 else 1
                velocidad_actual = velocidad_minima + ((velocidad_max - velocidad_minima) * progreso)
            else:
                velocidad_actual = velocidad_max

            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado
            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(velocidad_minima, velocidad_base) 
            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))
            self.motor_izquierda.dc(potencia_izq)
            self.motor_derecha.dc(potencia_der)
            last_error = error
            wait(1)
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        cronometro.pause()

    def seguidor_linea_color(self, sensor_color, velocidad_max, color_objetivo, lado="derecha", tiempo_acomodo_ms=800, distancia_cm=None, lecturas_confirmacion=3):
        cronometro = StopWatch()
        velocidad_minima = 25
        velocidad_enfoque = 50 
        tiempo_aceleracion_ms = 0 
        
        if distancia_cm is None:
            velocidad_max = min(velocidad_max, 70) 
            
        kp = 0.85  
        kd = 2.5   
        k_freno = 0.6 
        last_error = 0
        objetivo_reflexion = 35
        multiplicador_lado = 1 if lado == "derecha" else -1
        contador_color = 0 
        
        if distancia_cm is not None:
            diametro_rueda = 5.6
            circunferencia = 3.1416 * diametro_rueda
            grados_objetivo = (distancia_cm / circunferencia) * 360
            self.motor_izquierda.reset_angle(0)
            self.motor_derecha.reset_angle(0)
            
        cronometro.reset()
        cronometro.resume()
        
        while True:
            if self.detectar_color_preciso(sensor_color) == color_objetivo:
                contador_color += 1
                if contador_color >= lecturas_confirmacion:
                    break 
            else:
                contador_color = 0 
                
            tiempo_actual = cronometro.time()
            if tiempo_actual < tiempo_acomodo_ms:
                vel_aceleracion = velocidad_minima
            elif tiempo_actual < (tiempo_acomodo_ms + tiempo_aceleracion_ms):
                tiempo_en_rampa = tiempo_actual - tiempo_acomodo_ms
                progreso = tiempo_en_rampa / tiempo_aceleracion_ms if tiempo_aceleracion_ms > 0 else 1
                vel_aceleracion = velocidad_minima + ((velocidad_max - velocidad_minima) * progreso)
            else:
                vel_aceleracion = velocidad_max
                
            if distancia_cm is not None:
                grados_actuales = (abs(self.motor_izquierda.angle()) + abs(self.motor_derecha.angle())) / 2
                progreso_distancia = grados_actuales / grados_objetivo
                progreso_distancia = min(1.0, progreso_distancia) 
                vel_desaceleracion = velocidad_max - ((velocidad_max - velocidad_enfoque) * progreso_distancia)
                vel_desaceleracion = max(velocidad_enfoque, vel_desaceleracion) 
                velocidad_actual = min(vel_aceleracion, vel_desaceleracion)
            else:
                velocidad_actual = vel_aceleracion
                
            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado
            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(velocidad_minima, velocidad_base) 
            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))
            
            self.motor_izquierda.dc(potencia_izq)
            self.motor_derecha.dc(potencia_der)
            last_error = error
            wait(1)
            
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        cronometro.pause()
    
    def seguidor_linea_distancia_desacelerado(self, sensor_color, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp = 0.85, kd = 2.5, k_freno = 0.6, margen_cm=0):
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_objetivo = (distancia_cm / circunferencia) * 360
        grados_margen = (margen_cm / circunferencia) * 360 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)
        
        self.motor_izquierda.reset_angle(0)
        self.motor_derecha.reset_angle(0)
        cronometro = StopWatch()
        velocidad_minima = 25
        velocidad_enfoque = 50 
        tiempo_aceleracion_ms = 0 
        last_error = 0
        objetivo_reflexion = 35
        multiplicador_lado = 1 if lado == "derecha" else -1
        cronometro.reset()
        cronometro.resume()
        
        while True:
            grados_actuales = (abs(self.motor_izquierda.angle()) + abs(self.motor_derecha.angle())) / 2
            if grados_actuales >= grados_objetivo_real:
                break
                
            tiempo_actual = cronometro.time()
            if tiempo_actual < tiempo_acomodo_ms:
                vel_aceleracion = velocidad_minima
            elif tiempo_actual < (tiempo_acomodo_ms + tiempo_aceleracion_ms):
                tiempo_en_rampa = tiempo_actual - tiempo_acomodo_ms
                progreso = tiempo_en_rampa / tiempo_aceleracion_ms if tiempo_aceleracion_ms > 0 else 1
                vel_aceleracion = velocidad_minima + ((velocidad_max - velocidad_minima) * progreso)
            else:
                vel_aceleracion = velocidad_max

            progreso_distancia = grados_actuales / grados_objetivo
            progreso_distancia = min(1.0, progreso_distancia) 
            vel_desaceleracion = velocidad_max - ((velocidad_max - velocidad_enfoque) * progreso_distancia)
            vel_desaceleracion = max(velocidad_enfoque, vel_desaceleracion) 
            
            velocidad_actual = min(vel_aceleracion, vel_desaceleracion)

            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado
            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(velocidad_minima, velocidad_base) 
            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))
            
            self.motor_izquierda.dc(potencia_izq)
            self.motor_derecha.dc(potencia_der)
            last_error = error
            wait(1)
            
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        cronometro.pause()
    
    def seguir_hasta_interseccion(self, sensor_delantero, sensor_trasero, velocidad_base=35, lado="derecha", kp=0.85, kd=2.5):
        objetivo_reflexion = 35 
        umbral_delantero = 15   
        umbral_trasero = 15      

        multiplicador_lado = 1 if lado == "derecha" else -1
        last_error = 0

        while True:
            ref_delantero = sensor_delantero.reflection()
            ref_trasero = sensor_trasero.reflection()

            if ref_delantero <= umbral_delantero and ref_trasero <= umbral_trasero:
                break

            error = ref_delantero - objetivo_reflexion
            derivative = error - last_error
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado

            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction

            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))

            self.motor_izquierda.dc(potencia_izq)
            self.motor_derecha.dc(potencia_der)
        
            last_error = error
            wait(1)

        self.motor_izquierda.hold()
        self.motor_derecha.hold()
    # endregion

    # region MOTORES DE TRACCIÓN (Individuales)
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
    # endregion

    # region MECANISMOS - Garra Delantera
    def abrir_garra_delantera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_garra_delantera.angle() + abs(grados)
            self.motor_garra_delantera.run_angle(velocidad, abs(grados), then=frenado, wait=False)
            while abs(angulo_meta - self.motor_garra_delantera.angle()) > margen_grados:
                if self.motor_garra_delantera.stalled(): break
                wait(2)
        else:
            self.motor_garra_delantera.run_angle(velocidad, abs(grados), then=frenado, wait=wait_after)
    
    def abrir_garra_delantera_al_tope(self, velocidad=800, limite_potencia=50):
        self.motor_garra_delantera.run_until_stalled(abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)

    def cerrar_garra_delantera_al_tope(self, velocidad=800, limite_potencia=50):
        self.motor_garra_delantera.run_until_stalled(-abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)
    # endregion

    # region MECANISMOS - Eje Central y Garra Trasera
    def mover_eje_central(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_eje_central.angle() + grados
            self.motor_eje_central.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor_eje_central.angle()) > margen_grados:
                if self.motor_eje_central.stalled(): break
                wait(2)
        else:
            self.motor_eje_central.run_angle(velocidad, grados, then=frenado, wait=wait_after)
    
    def llevar_eje_central_al_tope(self, direccion, velocidad=1000, limite_potencia=60):
        if direccion == "positivo" or direccion == 1:
            vel_real = abs(velocidad)
        elif direccion == "negativo" or direccion == -1:
            vel_real = -abs(velocidad)
        else:
            print("Error: La dirección debe ser 'positivo' o 'negativo'.")
            return None
        angulo_tope = self.motor_eje_central.run_until_stalled(vel_real, then=Stop.HOLD, duty_limit=limite_potencia)
        return angulo_tope

    def mover_garra_trasera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_eje_central(grados, velocidad, wait_after, frenado, margen_grados)
    # endregion

    # region SENSORES Y UTILIDADES
    def detectar_color_preciso(self, sensor):
        color_hsv = sensor.hsv()
        h = color_hsv.h
        s = color_hsv.s
        v = color_hsv.v
        
        if s < 35:
            if v > 60: return Color.WHITE
            else: return Color.BLACK
        else:
            if h < 95 or h > 310: return Color.YELLOW
            elif h < 185: return Color.GREEN
            else: return Color.BLUE

    def identificar_combinacion(self, sensor, distancia_si_verde):
        color_principal = sensor.color()
        if color_principal not in config.MOSAICOS: # <-- Cambio aquí
            print("Error: Color principal no reconocido")
            print(f"Color escaneado: {color_principal}") 
            return -1  
        decision = config.MOSAICOS[color_principal] # <-- Cambio aquí
        if type(decision) is dict:
            self.avanzar_recto(distancia_si_verde)
            color_anterior = sensor.color()
            if color_anterior not in decision:
                print(f"Error: El segundo color ({color_anterior}) no completa un patrón verde válido.")
                return -1
            return decision[color_anterior]
        return decision
    # endregion