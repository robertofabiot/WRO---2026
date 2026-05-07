from pybricks.tools import StopWatch, wait
from pybricks.parameters import Color

class Navegacion:
    def __init__(self, chasis):
        self.chasis = chasis
        
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

    def giro_preciso_pd(self, angulo_relativo, max_speed=800, min_speed=40, kp=4.0, kd=18.0, margen_grados=0):
        angulo_inicial = self.chasis.hub.imu.heading()
        angulo_meta = angulo_inicial + angulo_relativo
        tolerancia = 1    
        error_previo = 0
        while True:
            angulo_actual = self.chasis.hub.imu.heading()
            error = angulo_meta - angulo_actual
            if abs(error) <= max(tolerancia, margen_grados):
                break
            derivada = error - error_previo
            turn_rate = (error * kp) + (derivada * kd)
            if turn_rate > 0:
                turn_rate = min(max(turn_rate, min_speed), max_speed)
            else:
                turn_rate = max(min(turn_rate, -min_speed), -max_speed)
            self.chasis.drive_base.drive(0, turn_rate)
            error_previo = error
            wait(10)
        self.chasis.drive_base.stop()

    def giro_eje_puro(self, angulo_relativo, kp=3.5, kd=15.0, max_speed=600, min_speed=30, margen_grados=0):
        self.chasis.drive_base.stop()
        angulo_inicial = self.chasis.hub.imu.heading()
        angulo_meta = angulo_inicial + angulo_relativo
        tolerancia = 1    
        error_previo = 0
        while True:
            angulo_actual = self.chasis.hub.imu.heading()
            error = angulo_meta - angulo_actual
            if abs(error) <= max(tolerancia, margen_grados):
                break
            derivada = error - error_previo
            magnitud = abs((error * kp) + (derivada * kd))
            velocidad_giro = max(min_speed, min(magnitud, max_speed))
            if error > 0:
                self.chasis.motor_izquierda.run(velocidad_giro)
                self.chasis.motor_derecha.run(-velocidad_giro)
            else:
                self.chasis.motor_izquierda.run(-velocidad_giro)
                self.chasis.motor_derecha.run(velocidad_giro)
            error_previo = error
            wait(10)
        self.chasis.motor_izquierda.hold()
        self.chasis.motor_derecha.hold()

    def seguidor_linea_distancia(self, sensor_color, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp = 0.85, kd = 2.5, k_freno = 0.6, margen_cm=0):
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_objetivo = (distancia_cm / circunferencia) * 360
        grados_margen = (margen_cm / circunferencia) * 360 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)
        
        self.chasis.motor_izquierda.reset_angle(0)
        self.chasis.motor_derecha.reset_angle(0)
        cronometro = StopWatch()
        velocidad_minima = 25
        tiempo_aceleracion_ms = 0 
        last_error = 0
        objetivo_reflexion = 35
        multiplicador_lado = 1 if lado == "derecha" else -1
        cronometro.reset()
        cronometro.resume()
        
        while True:
            grados_actuales = (abs(self.chasis.motor_izquierda.angle()) + abs(self.chasis.motor_derecha.angle())) / 2
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
            
            # --- Aplicando compensación de voltaje ---
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            last_error = error
            wait(1)
            
        self.chasis.motor_izquierda.stop()
        self.chasis.motor_derecha.stop()
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
            self.chasis.motor_izquierda.reset_angle(0)
            self.chasis.motor_derecha.reset_angle(0)
            
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
                grados_actuales = (abs(self.chasis.motor_izquierda.angle()) + abs(self.chasis.motor_derecha.angle())) / 2
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
            
            # --- Aplicando compensación de voltaje ---
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            last_error = error
            wait(1)
            
        self.chasis.motor_izquierda.stop()
        self.chasis.motor_derecha.stop()
        cronometro.pause()
    
    def seguidor_linea_distancia_desacelerado(self, sensor_color, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp = 0.85, kd = 2.5, k_freno = 0.6, margen_cm=0):
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_objetivo = (distancia_cm / circunferencia) * 360
        grados_margen = (margen_cm / circunferencia) * 360 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)
        
        self.chasis.motor_izquierda.reset_angle(0)
        self.chasis.motor_derecha.reset_angle(0)
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
            grados_actuales = (abs(self.chasis.motor_izquierda.angle()) + abs(self.chasis.motor_derecha.angle())) / 2
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
            
            # --- Aplicando compensación de voltaje ---
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            last_error = error
            wait(1)
            
        self.chasis.motor_izquierda.stop()
        self.chasis.motor_derecha.stop()
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

            # --- Aplicando compensación de voltaje ---
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
        
            last_error = error
            wait(1)

        self.chasis.motor_izquierda.hold()
        self.chasis.motor_derecha.hold()