from pybricks.tools import StopWatch, wait
from pybricks.parameters import Color
from Utils import Utils

class Navegacion:
    def __init__(self, chasis):
        self.chasis = chasis
        
    def detectar_color_preciso(self, sensor):
        color_hsv = sensor.hsv()
        h, s, v = color_hsv.h, color_hsv.s, color_hsv.v
        if s < 35:
            if v > 60: return Color.WHITE
            else: return Color.BLACK
        else:
            if h < 95 or h > 310: return Color.YELLOW
            elif h < 185: return Color.GREEN
            else: return Color.BLUE

    def giro_preciso_pd(self, angulo_relativo, max_speed=800, min_speed=40, kp=4.0, kd=18.0, margen_grados=0, encadenado=False):
        angulo_meta = self.chasis.hub.imu.heading() + angulo_relativo
        error_previo = 0
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            if abs(error) <= max(1, margen_grados): break
            turn_rate = (error * kp) + ((error - error_previo) * kd)
            turn_rate = min(max(turn_rate, min_speed), max_speed) if turn_rate > 0 else max(min(turn_rate, -min_speed), -max_speed)
            self.chasis.drive_base.drive(0, turn_rate)
            error_previo = error
            wait(10)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def giro_eje_puro(self, angulo_relativo, kp=3.5, kd=15.0, max_speed=600, min_speed=30, margen_grados=0, encadenado=False):
        self.chasis.drive_base.stop()
        angulo_meta = self.chasis.hub.imu.heading() + angulo_relativo
        error_previo = 0
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            if abs(error) <= max(1, margen_grados): break
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
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.hold()
            self.chasis.motor_derecha.hold()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def giro_absoluto_pd(self, angulo_objetivo, max_speed=800, min_speed=40, kp=4.0, kd=18.0, margen_grados=0, ruta_corta=True, encadenado=False):
        self.chasis.drive_base.stop()
        error_previo = 0
        angulo_actual_inicial = self.chasis.hub.imu.heading()
        error_bruto_inicial = angulo_objetivo - angulo_actual_inicial
        error_corto_inicial = (error_bruto_inicial + 180) % 360 - 180
        
        if ruta_corta:
            giro_requerido = error_corto_inicial
        else:
            giro_requerido = error_corto_inicial - 360 if error_corto_inicial > 0 else error_corto_inicial + 360 if error_corto_inicial < 0 else 0
                
        angulo_meta = angulo_actual_inicial + giro_requerido
        
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            if abs(error) <= max(1, margen_grados): break
            
            derivada = error - error_previo
            turn_rate = (error * kp) + (derivada * kd)
            
            # --- ZONA DINÁMICA ---
            # Si faltan menos de 15 grados, quitamos la restricción de 400 
            # y dejamos que el PD frene suavemente hasta 40 para no oscilar.
            min_speed_actual = min_speed if abs(error) > 15 else 40 
            
            turn_rate = min(max(turn_rate, min_speed_actual), max_speed) if turn_rate > 0 else max(min(turn_rate, -min_speed_actual), -max_speed)
            
            self.chasis.drive_base.drive(0, turn_rate)
            error_previo = error
            wait(10)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def seguidor_linea_distancia(self, sensor_color, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp=0.85, kd=2.5, k_freno=0.6, margen_cm=0, encadenado=False):
        grados_objetivo = (distancia_cm / (3.1416 * 5.6)) * 360
        grados_objetivo_real = max(0, grados_objetivo - ((margen_cm / (3.1416 * 5.6)) * 360 if margen_cm > 0 else 0))
        
        self.chasis.motor_izquierda.reset_angle(0)
        self.chasis.motor_derecha.reset_angle(0)
        cronometro = StopWatch()
        last_error = 0
        multiplicador_lado = 1 if lado == "derecha" else -1
        cronometro.reset()
        cronometro.resume()
        
        while True:
            if (abs(self.chasis.motor_izquierda.angle()) + abs(self.chasis.motor_derecha.angle())) / 2 >= grados_objetivo_real: break
            t = cronometro.time()
            velocidad_actual = 25 if t < tiempo_acomodo_ms else velocidad_max

            error = sensor_color.reflection() - 35
            correction = ((error * kp) + ((error - last_error) * kd)) * multiplicador_lado
            velocidad_base = max(25, velocidad_actual - (abs(error) * k_freno))
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(max(-100, min(100, velocidad_base - correction))))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(max(-100, min(100, velocidad_base + correction))))
            last_error = error
            wait(1)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.stop()
            self.chasis.motor_derecha.stop()
            cronometro.pause()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def seguidor_linea_color(self, sensor_color, velocidad_max, color_objetivo, lado="derecha", tiempo_acomodo_ms=800, distancia_cm=None, lecturas_confirmacion=3, encadenado=False):
        cronometro = StopWatch()
        velocidad_max = min(velocidad_max, 70) if distancia_cm is None else velocidad_max
        last_error, contador_color = 0, 0
        multiplicador_lado = 1 if lado == "derecha" else -1
        
        if distancia_cm is not None:
            grados_objetivo = (distancia_cm / (3.1416 * 5.6)) * 360
            self.chasis.motor_izquierda.reset_angle(0)
            self.chasis.motor_derecha.reset_angle(0)
            
        cronometro.reset()
        cronometro.resume()
        
        while True:
            if self.detectar_color_preciso(sensor_color) == color_objetivo:
                contador_color += 1
                if contador_color >= lecturas_confirmacion: break 
            else:
                contador_color = 0 
                
            velocidad_actual = 25 if cronometro.time() < tiempo_acomodo_ms else velocidad_max
                
            error = sensor_color.reflection() - 35
            correction = ((error * 0.85) + ((error - last_error) * 2.5)) * multiplicador_lado
            velocidad_base = max(25, velocidad_actual - (abs(error) * 0.6)) 
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(max(-100, min(100, velocidad_base - correction))))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(max(-100, min(100, velocidad_base + correction))))
            last_error = error
            wait(1)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.stop()
            self.chasis.motor_derecha.stop()
            cronometro.pause()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)
    
    def giro_absoluto_motor_izquierdo(self, angulo_objetivo, max_speed=800, min_speed=120, kp=4.0, kd=18.0, margen_grados=0, ruta_corta=True, encadenado=False):
        """
        Gira hacia un ángulo absoluto pivotando sobre la rueda derecha (bloqueada).
        Usa únicamente el motor izquierdo.
        """
        self.chasis.drive_base.stop()
        self.chasis.motor_derecha.hold() 
        
        error_previo = 0
        factor_conversion = 5.71 # Convierte grados del robot a grados del motor
        
        angulo_actual_inicial = self.chasis.hub.imu.heading()
        error_bruto_inicial = angulo_objetivo - angulo_actual_inicial
        error_corto_inicial = (error_bruto_inicial + 180) % 360 - 180
        
        if ruta_corta:
            giro_requerido = error_corto_inicial
        else:
            giro_requerido = error_corto_inicial - 360 if error_corto_inicial > 0 else (error_corto_inicial + 360 if error_corto_inicial < 0 else 0)
                
        angulo_meta = angulo_actual_inicial + giro_requerido
        
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            if abs(error) <= max(1, margen_grados): break
            
            derivada = error - error_previo
            turn_rate = ((error * kp) + (derivada * kd)) * factor_conversion
            
            velocidad_aplicar = min(max(turn_rate, min_speed), max_speed) if turn_rate > 0 else max(min(turn_rate, -min_speed), -max_speed)
                
            self.chasis.motor_izquierda.run(velocidad_aplicar)
            error_previo = error
            wait(10)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.hold()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def giro_absoluto_motor_derecho(self, angulo_objetivo, max_speed=800, min_speed=120, kp=4.0, kd=18.0, margen_grados=0, ruta_corta=True, encadenado=False):
        """
        Gira hacia un ángulo absoluto pivotando sobre la rueda izquierda (bloqueada).
        Usa únicamente el motor derecho.
        """
        self.chasis.drive_base.stop()
        self.chasis.motor_izquierda.hold() 
        
        error_previo = 0
        factor_conversion = 5.71 # Convierte grados del robot a grados del motor
        
        angulo_actual_inicial = self.chasis.hub.imu.heading()
        error_bruto_inicial = angulo_objetivo - angulo_actual_inicial
        error_corto_inicial = (error_bruto_inicial + 180) % 360 - 180
        
        if ruta_corta:
            giro_requerido = error_corto_inicial
        else:
            giro_requerido = error_corto_inicial - 360 if error_corto_inicial > 0 else (error_corto_inicial + 360 if error_corto_inicial < 0 else 0)
                
        angulo_meta = angulo_actual_inicial + giro_requerido
        
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            if abs(error) <= max(1, margen_grados): break
            
            derivada = error - error_previo
            turn_rate = ((error * kp) + (derivada * kd)) * factor_conversion
            
            velocidad_aplicar = min(max(turn_rate, min_speed), max_speed) if turn_rate > 0 else max(min(turn_rate, -min_speed), -max_speed)
                
            self.chasis.motor_derecha.run(-velocidad_aplicar) # Invertido por física
            error_previo = error
            wait(10)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_derecha.hold()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)
    
    def avanzar_manteniendo_rumbo(self, distancia_cm, velocidad=800, angulo_objetivo=None, kp=2.5, kd=10.0, margen_cm=0, encadenado=False):
        """
        Avanza una distancia manteniendo un rumbo fijo usando un Controlador PD.
        Si se le pasa un 'angulo_objetivo', corregirá su trayectoria hacia ese ángulo absoluto del mapa mientras avanza.
        Si no se le pasa, mantendrá exactamente el rumbo actual.
        """
        # 1. Determinamos el ángulo al que queremos aferrarnos
        if angulo_objetivo is None:
            angulo_meta = self.chasis.hub.imu.heading()
        else:
            angulo_meta = angulo_objetivo

        # 2. Preparamos las distancias (en milímetros para la precisión del drive_base)
        dist_inicial = self.chasis.drive_base.distance()
        distancia_mm_objetivo = abs(distancia_cm * 10)
        margen_mm = abs(margen_cm * 10)
        
        # Ajustamos el signo de la velocidad (soporta ir en reversa si distancia_cm es negativo)
        velocidad_real = abs(velocidad) if distancia_cm > 0 else -abs(velocidad)
        
        error_previo = 0
        
        while True:
            # Condición de salida por distancia recorrida
            distancia_actual = abs(self.chasis.drive_base.distance() - dist_inicial)
            if distancia_actual >= max(1, distancia_mm_objetivo - margen_mm):
                break
                
            # Calculamos el error continuo (soporta cruces de 360 grados a negativos)
            error_bruto = angulo_meta - self.chasis.hub.imu.heading()
            error = (error_bruto + 180) % 360 - 180
            
            # Controlador PD para la rotación mientras se avanza
            derivada = error - error_previo
            turn_rate = (error * kp) + (derivada * kd)
            
            # Tope de seguridad: limitamos el giro máximo para evitar que el robot 
            # sacrifique demasiado el avance lineal intentando girar de golpe
            turn_rate = min(max(turn_rate, -200), 200) 
            
            # Aplicamos la velocidad constante y la corrección de rotación al mismo tiempo
            self.chasis.drive_base.drive(velocidad_real, turn_rate)
            
            error_previo = error
            wait(10)
            
        # Cierre del movimiento soportando tu sistema fluido
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)
    
    def avanzar_tiempo_luego_color(self, sensor_color, tiempo_ciego_s, color_objetivo, velocidad_alta=950, velocidad_escaneo=150, lecturas_confirmacion=2, encadenado=False):
        """
        Avanza a máxima velocidad durante un tiempo ciego (ignorando derrapes), 
        luego reduce la velocidad abruptamente y avanza hasta detectar un color específico.
        """
        from pybricks.tools import StopWatch, wait # (Asegúrate de que estas estén importadas arriba)
        
        cronometro = StopWatch()
        contador_color = 0
        
        # 1. ETAPA DE AVANCE CIEGO
        cronometro.reset()
        cronometro.resume()
        
        # Le decimos que avance recto. Al tener use_gyro(True) en el drive_base,
        # Pybricks corregirá automáticamente cualquier giro no deseado.
        self.chasis.drive_base.drive(velocidad_alta, 0)
        
        # Esperamos bloqueando el código hasta que pasen los segundos solicitados
        while cronometro.time() < (tiempo_ciego_s * 1000):
            wait(10)
            
        # 2. ETAPA DE ESCANEO LENTO
        self.chasis.drive_base.drive(velocidad_escaneo, 0)
        
        while True:
            # Usamos tu misma lógica de detección precisa y confiable
            if self.detectar_color_preciso(sensor_color) == color_objetivo:
                contador_color += 1
                # Pedimos confirmaciones consecutivas para evitar falsos positivos
                # por destellos de luz o el borde difuminado de la línea
                if contador_color >= lecturas_confirmacion: 
                    break
            else:
                contador_color = 0 
                
            wait(5) # Frecuencia de escaneo
            
        # 3. TERMINACIÓN
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)
    
    def seguidor_linea_cruces(self, sensor_color, velocidad_max, cruces_objetivo, lado="derecha", tiempo_acomodo_ms=800, kp=0.85, kd=2.5, k_freno=0.6, encadenado=False):
        """
        Sigue la línea y cuenta las intersecciones perpendiculares negras.
        Se detiene al alcanzar el número de cruces objetivo.
        """
        cronometro = StopWatch()
        last_error = 0
        multiplicador_lado = 1 if lado == "derecha" else -1
        
        cruces_detectados = 0
        en_cruce = False # Bandera para no contar el mismo cruce varias veces
        umbral_negro = 15 # Valor de reflexión para negro puro (ajústalo según tu calibración)
        umbral_salida = 25 # Valor para considerar que ya volvimos al borde
        
        self.chasis.motor_izquierda.reset_angle(0)
        self.chasis.motor_derecha.reset_angle(0)
        
        cronometro.reset()
        cronometro.resume()
        
        while cruces_detectados < cruces_objetivo:
            t = cronometro.time()
            velocidad_actual = 25 if t < tiempo_acomodo_ms else velocidad_max

            # 1. Leer sensor
            reflexion_actual = sensor_color.reflection()
            error = reflexion_actual - 35
            
            # 2. Lógica de detección de cruces
            if reflexion_actual <= umbral_negro and not en_cruce:
                # Acabamos de entrar a una intersección negra
                cruces_detectados += 1
                en_cruce = True
                print(f"Cruce {cruces_detectados}/{cruces_objetivo} detectado")
            elif reflexion_actual >= umbral_salida and en_cruce:
                # Ya salimos de la intersección y volvimos al borde
                en_cruce = False

            # 3. Lógica PD (Control de tracción)
            correction = ((error * kp) + ((error - last_error) * kd)) * multiplicador_lado
            velocidad_base = max(25, velocidad_actual - (abs(error) * k_freno))
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(max(-100, min(100, velocidad_base - correction))))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(max(-100, min(100, velocidad_base + correction))))
            last_error = error
            wait(1)
            
        # 4. Terminación fluida o total
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.stop()
            self.chasis.motor_derecha.stop()
            cronometro.pause()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def seguidor_linea_cruces_y_distancia(self, sensor_color, velocidad_max, cruces_objetivo, distancia_extra_cm, distancia_inicial_cm=0, lado="derecha", tiempo_acomodo_ms=800, kp=0.85, kd=2.5, k_freno=0.6, margen_cm=0, encadenado=False):
        """
        Combina un avance inicial ciego a cruces, la detección de cruces y un avance extra por distancia 
        en un solo movimiento fluido. Mantiene el mismo lazo PID para evitar derrapes.
        """
        from pybricks.tools import StopWatch, wait # (Asegúrate de que existan)
        from Utils import Utils
        
        cronometro = StopWatch()
        last_error = 0
        multiplicador_lado = 1 if lado == "derecha" else -1
        
        cruces_detectados = 0
        en_cruce = False
        umbral_negro = 15 
        umbral_salida = 25 
        
        # --- Variables de fases ---
        en_distancia_inicial = distancia_inicial_cm > 0
        grados_objetivo_inicial = max(0, (distancia_inicial_cm / (3.1416 * 5.6)) * 360)
        
        buscando_cruces = cruces_objetivo > 0
        grados_objetivo_extra = max(0, ((distancia_extra_cm - margen_cm) / (3.1416 * 5.6)) * 360)
        grados_inicio_extra = 0
        
        self.chasis.motor_izquierda.reset_angle(0)
        self.chasis.motor_derecha.reset_angle(0)
        
        cronometro.reset()
        cronometro.resume()
        
        while True:
            # Medida global de grados para las distancias (inicial y extra)
            grados_recorridos_totales = (abs(self.chasis.motor_izquierda.angle()) + abs(self.chasis.motor_derecha.angle())) / 2

            # --- EVALUACIÓN DE SALIDA Y FASES ---
            if en_distancia_inicial:
                # 1. Fase: Avance inicial ignorando cruces
                if grados_recorridos_totales >= grados_objetivo_inicial:
                    en_distancia_inicial = False # Termina avance inicial, empieza a buscar cruces
            elif not buscando_cruces:
                # 3. Fase: Avance extra después de haber encontrado los cruces
                grados_recorridos_extra = grados_recorridos_totales - grados_inicio_extra
                if grados_recorridos_extra >= grados_objetivo_extra:
                    break
                    
            t = cronometro.time()
            velocidad_actual = 25 if t < tiempo_acomodo_ms else velocidad_max

            # 1. Leer sensor
            reflexion_actual = sensor_color.reflection()
            error = reflexion_actual - 35
            
            # 2. Lógica de cruces (Solo se activa si ya pasamos la distancia inicial)
            if not en_distancia_inicial and buscando_cruces:
                if reflexion_actual <= umbral_negro and not en_cruce:
                    cruces_detectados += 1
                    en_cruce = True
                    print(f"Cruce {cruces_detectados}/{cruces_objetivo} detectado")
                    
                    if cruces_detectados >= cruces_objetivo:
                        buscando_cruces = False
                        # Guardamos el odómetro actual para empezar a medir la distancia extra desde este punto exacto
                        grados_inicio_extra = grados_recorridos_totales
                
                elif reflexion_actual >= umbral_salida and en_cruce:
                    en_cruce = False

            # 3. Lógica PD ininterrumpida
            correction = ((error * kp) + ((error - last_error) * kd)) * multiplicador_lado
            velocidad_base = max(25, velocidad_actual - (abs(error) * k_freno))
            
            self.chasis.motor_izquierda.dc(self.chasis.compensar_voltaje(max(-100, min(100, velocidad_base - correction))))
            self.chasis.motor_derecha.dc(self.chasis.compensar_voltaje(max(-100, min(100, velocidad_base + correction))))
            last_error = error
            wait(1)
            
        # 4. Terminación fluida o total
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.stop()
            self.chasis.motor_derecha.stop()
            cronometro.pause()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def curva_coordenada_local(self, x_cm, y_cm, velocidad=600, kp_giro=4.5, tolerancia_cm=1.0, encadenado=False):
        """
        Hace que el robot dibuje una curva fluida hacia el punto relativo (x_cm, y_cm).
        Implementación 100% nativa sin depender del módulo 'math'.
        x_cm: Distancia hacia adelante (positivo) o atrás (negativo).
        y_cm: Desplazamiento lateral (positivo izquierda, negativo derecha).
        """
        # 1. Constantes matemáticas locales
        PI = 3.14159265
        HALF_PI = 1.57079632
        TWO_PI = 6.28318530

        # 2. Mini-motor trigonométrico interno (Aproximaciones eficientes)
        def seno(a):
            # Normaliza el ángulo entre -PI y PI
            a = (a + PI) % TWO_PI - PI
            a2 = a * a
            # Serie de Taylor (grado 7) para altísima precisión en odometría
            return a - (a * a2) / 6.0 + (a2 * a2 * a) / 120.0 - (a2 * a2 * a2 * a) / 5040.0

        def coseno(a):
            return seno(a + HALF_PI)

        def atan2_aprox(y, x):
            # Manejo de la singularidad x = 0
            if x == 0:
                return HALF_PI if y > 0 else (-HALF_PI if y < 0 else 0)
            
            z = y / x
            # Aproximación polinomial racional (muy rápida para procesadores sin FPU fuerte)
            # Calcula atan(z)
            abs_z = z if z > 0 else -z
            if abs_z <= 1:
                atan = z / (1.0 + 0.28086 * z * z)
            else:
                inv_z = 1.0 / z
                atan = HALF_PI - inv_z / (1.0 + 0.28086 * inv_z * inv_z)
                if y < 0: 
                    atan -= PI
            
            # Ajuste por cuadrantes
            if x < 0:
                if y >= 0:
                    atan += PI
                else:
                    atan -= PI
            return atan

        # Conversión a milímetros
        target_x = x_cm * 10.0
        target_y = y_cm * 10.0
        
        # Registrar estado inicial
        dist_inicial = self.chasis.drive_base.distance()
        angulo_inicial = self.chasis.hub.imu.heading()
        
        x_act, y_act = 0.0, 0.0
        dist_previa = 0.0
        
        while True:
            # Integración de odometría diferencial
            dist_actual = self.chasis.drive_base.distance() - dist_inicial
            delta_dist = dist_actual - dist_previa
            
            # Ángulo relativo actual (Conversión de grados a radianes manual)
            angulo_relativo_rad = (self.chasis.hub.imu.heading() - angulo_inicial) * PI / 180.0
            
            # Proyección del movimiento
            x_act += delta_dist * coseno(angulo_relativo_rad)
            y_act += delta_dist * seno(angulo_relativo_rad)
            dist_previa = dist_actual
            
            # Error hacia la coordenada objetivo
            error_x = target_x - x_act
            error_y = target_y - y_act
            
            # Pitágoras sin math.sqrt (usando exponente fraccionario)
            distancia_restante = (error_x**2 + error_y**2) ** 0.5
            
            if distancia_restante <= (tolerancia_cm * 10.0):
                break
                
            # Calcular el ángulo del vector objetivo
            alfa_objetivo_rad = atan2_aprox(error_y, error_x)
            error_angulo_rad = alfa_objetivo_rad - angulo_relativo_rad
            
            # Normalizar el ángulo entre -PI y PI (previene oscilaciones y bucles infinitos)
            error_angulo_rad = (error_angulo_rad + PI) % TWO_PI - PI
            
            # Conversión de radianes a grados manual
            error_angulo_grados = error_angulo_rad * 180.0 / PI
            
            # Control dinámico
            velocidad_lineal = velocidad * coseno(error_angulo_rad)
            
            if x_cm < 0:
                abs_velocidad = velocidad_lineal if velocidad_lineal > 0 else -velocidad_lineal
                velocidad_lineal = -abs_velocidad
            else:
                velocidad_lineal = 50 if velocidad_lineal < 50 else velocidad_lineal
                
            turn_rate = error_angulo_grados * kp_giro
            
            self.chasis.drive_base.drive(velocidad_lineal, turn_rate)
            wait(10)
            
        # Gestión del cierre (encadenamiento)
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)