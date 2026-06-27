from pybricks.tools import StopWatch, wait
from pybricks.parameters import Color, Stop
from pybricks.pupdevices import ColorSensor
from Chasis import Chasis # Importamos para el Type Hinting
import config

class Navegacion:
    def __init__(self, chasis: Chasis):
        self.chasis = chasis

    def detectar_color_preciso(self, sensor: ColorSensor):
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

    def giro_preciso_pd(self, angulo_relativo, max_speed=1000, min_speed=40, kp=8.5, kd=115.0, margen_grados=0):
        """
        Caballo de Troya: Mantiene los argumentos viejos para que Misiones.py no tire error, 
        pero usa control de hardware en C (DriveBase) para un giro violento, sin lag y 100% exacto.
        """
    
        # 1. Guardamos la configuración de pista
        turn_rate_original = config.TURN_RATE
        
        # 2. MODO BESTIA: Límite físico absoluto del motor
        self.chasis.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED, 
            straight_acceleration=config.STRAIGHT_ACCEL, 
            turn_rate=350,          # Velocidad tope sin crashear el hub
            turn_acceleration=1500  # Aceleración absurda para romper inercia
        )
        
        # 3. EL GIRO PERFECTO: 
        # Al tener use_gyro(True) en robot.py, esto lee el IMU nativamente a 1000Hz.
        # Si le faltan 5 grados y pesa mucho, el firmware le inyectará el 100% de la batería 
        # si es necesario para obligarlo a llegar, y frenará en seco por hardware.
        self.chasis.drive_base.turn(angulo_relativo)
        
        # 4. Restauramos la paz
        self.chasis.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED, 
            straight_acceleration=config.STRAIGHT_ACCEL, 
            turn_rate=turn_rate_original, 
            turn_acceleration=config.STRAIGHT_ACCEL
        )

    def giro_eje_puro(self, angulo_relativo, kp=1.6, kd=12.0, max_power=100, min_power=18, margen_grados=0):
        self.chasis.drive_base.stop()
        
        # Caché de funciones
        obtener_angulo = self.chasis.hub.imu.heading
        dc_izq = self.chasis.motor_izquierda.dc
        dc_der = self.chasis.motor_derecha.dc
        
        angulo_inicial = obtener_angulo()
        angulo_meta = angulo_inicial + angulo_relativo
        tolerancia = 1
        error_previo = 0
        
        while True:
            error = angulo_meta - obtener_angulo()
            if abs(error) <= max(tolerancia, margen_grados):
                break
                
            derivada = error - error_previo
            
            # MAGIA NEGRA: Calculamos potencia de voltaje (0 a 100%), no velocidad (0 a 1000)
            magnitud = abs((error * kp) + (derivada * kd))
            
            # Saturamos el voltaje al 100% como máximo
            potencia = max(min_power, min(magnitud, max_power))
            
            if error > 0:
                dc_izq(potencia)
                dc_der(-potencia)
            else:
                dc_izq(-potencia)
                dc_der(potencia)
                
            error_previo = error
            # Aumentamos a 5ms. El giroscopio se actualiza a ~200Hz. 
            # Si leemos más rápido, la derivada lee "0" artificialmente y pierde fuerza de frenado.
            wait(5) 
            
        # Freno electromagnético violento al terminar
        self.chasis.motor_izquierda.brake()
        self.chasis.motor_derecha.brake()


    def seguidor_linea_atras(self, sensor_color: ColorSensor, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp=0.85, kd=2.5, k_freno=0.6, margen_cm=0):
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_objetivo = (distancia_cm / circunferencia) * 360
        grados_margen = (margen_cm / circunferencia) * 360 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)
        
        self.chasis.motor_izquierda.reset_angle(0)
        self.chasis.motor_derecha.reset_angle(0)
        cronometro = StopWatch()
        
        # Setăm viteza minimă 
        velocidad_minima = 25 
        tiempo_aceleracion_ms = 0
        last_error = 0
        objetivo_reflexion = 35
        
        # SECRETUL: Inversăm multiplicatorul lateral pentru că mergem înapoi!
        multiplicador_lado = -1 if lado == "derecha" else 1 
        
        cronometro.reset()
        cronometro.resume()
        
        while True:
            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            
            # 1. El multiplicador de lado DEBE SER IGUAL que cuando vas hacia adelante
            # (Asegúrate de que arriba diga: multiplicador_lado = 1 if lado == "derecha" else -1)
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado
            
            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(velocidad_minima, velocidad_base)
            
            # 2. EL ARREGLO CRÍTICO: Los signos de la corrección se INVIERTEN
            # Al motor izquierdo se le SUMA la corrección, al derecho se le RESTA.
            potencia_izq = -velocidad_base + correction
            potencia_der = -velocidad_base - correction
            
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            
            last_error = error
            wait(1)

    def seguidor_linea_distancia(self, sensor_color: ColorSensor, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=0, tiempo_aceleracion_ms=350, kp=0.85, kd=2.5, k_freno=0.3, margen_cm=0, frenado_final=Stop.HOLD):

        
        # 1. CÁLCULO FÍSICO PURO (Inmune a fallos de DriveBase)
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        
        # Convertimos la distancia deseada a grados exactos del motor
        grados_totales = (distancia_cm / circunferencia) * 360
        grados_margen = (margen_cm / circunferencia) * 360 if margen_cm > 0 else 0
        grados_meta = max(0, grados_totales - grados_margen)
        
        # Zona balística (Últimos 2.5 cm en grados)
        grados_desaceleracion = (2.5 / circunferencia) * 360 
        
        # Guardamos el ángulo inicial real sin intentar resetearlo (que suele fallar)
        angulo_inicial_izq = self.chasis.motor_izquierda.angle()
        angulo_inicial_der = self.chasis.motor_derecha.angle()
        
        cronometro = StopWatch()
        velocidad_minima = 30 
        last_error = 0
        objetivo_reflexion = 35
        multiplicador_lado = 1 if lado == "derecha" else -1
        
        cronometro.reset()
        cronometro.resume()
        
        while True:
            # 2. LECTURA A PRUEBA DE FALLOS: Cuántos grados ha girado cada llanta desde que empezamos
            avance_izq = abs(self.chasis.motor_izquierda.angle() - angulo_inicial_izq)
            avance_der = abs(self.chasis.motor_derecha.angle() - angulo_inicial_der)
            
            # Promediamos para saber el avance real del centro del robot
            grados_avanzados = (avance_izq + avance_der) / 2
            grados_restantes = grados_meta - grados_avanzados
            
            # Condición de salida exacta
            if grados_restantes <= 0:
                break
                
            tiempo_actual = cronometro.time()
            
            # --- CONTROL DE TRACCIÓN (Aceleración sin derrape) ---
            if tiempo_actual < tiempo_acomodo_ms:
                velocidad_actual = velocidad_minima
            elif tiempo_actual < (tiempo_acomodo_ms + tiempo_aceleracion_ms):
                tiempo_en_rampa = tiempo_actual - tiempo_acomodo_ms
                progreso = tiempo_en_rampa / tiempo_aceleracion_ms
                velocidad_actual = velocidad_minima + ((velocidad_max - velocidad_minima) * progreso)
            else:
                velocidad_actual = velocidad_max
                
            # --- ATERRIZAJE BALÍSTICO ---
            if grados_restantes < grados_desaceleracion:
                progreso_freno = grados_restantes / grados_desaceleracion
                velocidad_actual = velocidad_minima + ((velocidad_actual - velocidad_minima) * progreso_freno)
                
            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            
            # Supresor de latigazo
            if grados_restantes < grados_desaceleracion:
                correction = ((error * (kp * 0.4)) + (derivative * kd)) * multiplicador_lado
            else:
                correction = ((error * kp) + (derivative * kd)) * multiplicador_lado
                
            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(velocidad_minima, velocidad_base) 
            
            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction
            
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            
            last_error = error
            wait(1)
            
        # 3. FRENO ACTIVO INMEDIATO
        if frenado_final == Stop.HOLD:
            self.chasis.motor_izquierda.hold()
            self.chasis.motor_derecha.hold()
        elif frenado_final == Stop.BRAKE:
            self.chasis.motor_izquierda.brake()
            self.chasis.motor_derecha.brake()
        else:
            self.chasis.motor_izquierda.stop()
            self.chasis.motor_derecha.stop()
            
        cronometro.pause()

    def seguidor_linea_distancia(self, sensor_color: ColorSensor, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp=0.85, kd=2.5, k_freno=0.6, margen_cm=0, frenado_final= Stop.HOLD):        
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_objetivo = (distancia_cm / circunferencia) * 360
        grados_margen = (margen_cm / circunferencia) * 360 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)
        
        # EL SECRETO 1: Calculamos los últimos 4 cm para iniciar el frenado recto
        grados_desaceleracion = (4.0 / circunferencia) * 360 
        
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
            # Cálculo más preciso del avance
            grados_izq = abs(self.chasis.motor_izquierda.angle())
            grados_der = abs(self.chasis.motor_derecha.angle())
            grados_actuales = (grados_izq + grados_der) / 2
            
            grados_restantes = grados_objetivo_real - grados_actuales
            
            # Condición de salida exacta
            if grados_restantes <= 0:
                break
                
            tiempo_actual = cronometro.time()
            
            # --- MANEJO DE VELOCIDAD ---
            if tiempo_actual < tiempo_acomodo_ms:
                velocidad_actual = velocidad_minima
            elif tiempo_actual < (tiempo_acomodo_ms + tiempo_aceleracion_ms):
                tiempo_en_rampa = tiempo_actual - tiempo_acomodo_ms
                progreso = tiempo_en_rampa / tiempo_aceleracion_ms if tiempo_aceleracion_ms > 0 else 1
                velocidad_actual = velocidad_minima + ((velocidad_max - velocidad_minima) * progreso)
            else:
                velocidad_actual = velocidad_max
                
            # EL SECRETO 2: Rampa balística final. Si faltan menos de 4cm, "aterrizamos" la velocidad.
            if grados_restantes < grados_desaceleracion:
                progreso_freno = grados_restantes / grados_desaceleracion
                velocidad_actual = velocidad_minima + ((velocidad_actual - velocidad_minima) * progreso_freno)
                
            # --- CÁLCULO PID ---
            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            
            # EL SECRETO 3: Supresor de latigazo. 
            # En la zona de frenado, suavizamos el KP a la mitad para que no gire el chasis bruscamente.
            if grados_restantes < grados_desaceleracion:
                correction = ((error * (kp * 0.5)) + (derivative * kd)) * multiplicador_lado
            else:
                correction = ((error * kp) + (derivative * kd)) * multiplicador_lado
                
            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(velocidad_minima, velocidad_base)
            
            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction
            
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            last_error = error
            wait(1)
            
        # EL SECRETO 4: Freno Activo. Obliga a la llanta a bloquearse en ese milímetro.
        if frenado_final == Stop.HOLD:
            self.chasis.motor_izquierda.hold()
            self.chasis.motor_derecha.hold()
        elif frenado_final == Stop.BRAKE:
            self.chasis.motor_izquierda.brake()
            self.chasis.motor_derecha.brake()
        else:
            self.chasis.motor_izquierda.stop()
            self.chasis.motor_derecha.stop()
            
        cronometro.pause()

    def seguir_linea_extremo(
        self,
        sensor_color: ColorSensor,
        velocidad_max=100,
        distancia_cm=70,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=120,
        kp=1.15,
        kd=2.6,
        k_freno=0.15,
        objetivo_reflexion=27,
        correccion_max=100,
        margen_cm=0,
        perfil_salida="encadenado",
        captura_inicial=True,
        tiempo_captura_ms=260,
        potencia_captura=55,
        kp_captura=2.4,
        margen_captura=5,
        lecturas_estables_captura=2
    ):
        """
        Seguidor de línea por borde usando dc() adaptado a la arquitectura Chasis.
        """
        # 1. Obtenemos las medidas desde config
        diametro_rueda_cm = config.DIAMETRO_RUEDA / 10
        circunferencia_cm = 3.14159 * diametro_rueda_cm
        grados_objetivo = (distancia_cm / circunferencia_cm) * 360

        grados_margen = (margen_cm / circunferencia_cm) * 360 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)

        multiplicador_lado = 1 if lado == "derecha" else -1

        # Reseteamos motores usando el chasis
        self.chasis.motor_izquierda.reset_angle(0)
        self.chasis.motor_derecha.reset_angle(0)

        # =====================================================
        # FASE 1: CAPTURA INICIAL DEL BORDE
        # =====================================================
        if captura_inicial:
            reloj_captura = StopWatch()
            reloj_captura.reset()
            estables = 0

            while reloj_captura.time() < tiempo_captura_ms:
                lectura = sensor_color.reflection()
                error = lectura - objetivo_reflexion

                if abs(error) <= margen_captura:
                    estables += 1
                    if estables >= lecturas_estables_captura:
                        break
                else:
                    estables = 0

                correction = error * kp_captura * multiplicador_lado
                correction = max(-correccion_max, min(correction, correccion_max)) # self.limitar reemplazado

                velocidad_base = 28 if abs(error) > 22 else potencia_captura

                potencia_izq = velocidad_base - correction
                potencia_der = velocidad_base + correction

                potencia_izq = max(-100, min(100, potencia_izq))
                potencia_der = max(-100, min(100, potencia_der))

                # Usamos los motores del chasis con compensación de voltaje
                self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
                self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))

                wait(2)

            self.chasis.motor_izquierda.brake()
            self.chasis.motor_derecha.brake()
            wait(8)
            self.chasis.motor_izquierda.reset_angle(0)
            self.chasis.motor_derecha.reset_angle(0)

        # =====================================================
        # FASE 2: SEGUIMIENTO RÁPIDO
        # =====================================================
        cronometro = StopWatch()
        cronometro.reset()

        velocidad_minima = 55
        last_error = 0
        last_derivative = 0

        while True:
            # Cálculo de promedio de grados en línea (reemplaza self.distancia_promedio_grados)
            grados_actuales = (abs(self.chasis.motor_izquierda.angle()) + abs(self.chasis.motor_derecha.angle())) / 2

            if grados_actuales >= grados_objetivo_real:
                break

            tiempo_actual = cronometro.time()

            if tiempo_actual < tiempo_acomodo_ms:
                velocidad_actual = velocidad_minima
            elif tiempo_actual < tiempo_acomodo_ms + tiempo_aceleracion_ms:
                progreso = (tiempo_actual - tiempo_acomodo_ms) / tiempo_aceleracion_ms
                velocidad_actual = velocidad_minima + ((velocidad_max - velocidad_minima) * progreso)
            else:
                velocidad_actual = velocidad_max

            lectura = sensor_color.reflection()
            error = lectura - objetivo_reflexion

            derivative = ((error - last_error) * 0.82) + (last_derivative * 0.18)
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado
            correction = max(-correccion_max, min(correction, correccion_max))

            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(38, velocidad_base) # Límite inferior

            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction

            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))

            # Aplicamos la potencia con compensación de voltaje para mayor fiabilidad
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))

            last_error = error
            last_derivative = derivative
            wait(2)

        # Manejo de salida (reemplaza self.terminar_movimiento)
        if perfil_salida == "brake":
            self.chasis.motor_izquierda.brake()
            self.chasis.motor_derecha.brake()
        else:
            self.chasis.motor_izquierda.stop()
            self.chasis.motor_derecha.stop()

    def seguidor_linea_color(self, sensor_color: ColorSensor, velocidad_max, color_objetivo, lado="derecha", tiempo_acomodo_ms=800, distancia_cm=None, lecturas_confirmacion=3):
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
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            last_error = error
            wait(1)
            
        self.chasis.motor_izquierda.stop()
        self.chasis.motor_derecha.stop()
        cronometro.pause()

    def seguidor_linea_distancia_desacelerado(self, sensor_color: ColorSensor, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp=0.85, kd=2.5, k_freno=0.6, margen_cm=0):
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
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            last_error = error
            wait(1)
            
        self.chasis.motor_izquierda.stop()
        self.chasis.motor_derecha.stop()
        cronometro.pause()

    def seguir_hasta_interseccion(self, sensor_delantero: ColorSensor, sensor_trasero: ColorSensor, velocidad_base=35, lado="derecha", kp=0.85, kd=2.5):
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
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(potencia_izq))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(potencia_der))
            last_error = error
            wait(1)
            
        self.chasis.motor_izquierda.hold()
        self.chasis.motor_derecha.hold()