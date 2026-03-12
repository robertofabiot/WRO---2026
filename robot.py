from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Stop, Color
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

mosaicos = {
    Color.GREEN: {Color.GREEN: 1, Color.YELLOW: 2}, 
    Color.BLUE: 3, 
    Color.YELLOW: 4, 
    Color.WHITE: 5
}

class Robot:
    def __init__(self, port_izq, port_der, port_garra): #port_garra falta
        """
        Configuración inicial del robot.
        Define la geometría, los puertos y activa la estabilización por giroscopio.
        """
        # 1. Inicializar el Hub (Orientación: Arriba=Z, Frente=X)
        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        
        # 2. Configurar Motores
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE)
        self.motor_derecha = Motor(port_der, Direction.CLOCKWISE)
        self.motor_garra = Motor(port_garra, Direction.COUNTERCLOCKWISE)
        
        # 3. Configurar Base Motriz (DriveBase)
        # Rueda: 56mm, Eje: 160mm
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, 56, 160)
        
        # 4. Activar Estabilización Gyro (Crucial para ir recto)
        self.drive_base.use_gyro(True)
        
        # 5. Configurar velocidades por defecto (basado en 'base_settings')
        self.VELOCIDAD_BASE = 950
        self.drive_base.settings(straight_speed=700, straight_acceleration=700, turn_rate=500)

    # region Movimiento Básico (Chasis)

    def avanzar_recto(self, distancia_cm, velocidad=None, frenado=Stop.BRAKE, wait_after=True):
        if velocidad is None:
            velocidad = self.VELOCIDAD_BASE
            
        velocidad = max(min(velocidad, 976), -976)
        distancia_mm = distancia_cm * 10
        self.drive_base.straight(distancia_mm, then=frenado, wait=wait_after)
    
    def avanzar_hasta_color(self, sensor_color, color_objetivo, velocidad=300):
        """
        Avanza o retrocede en línea recta perfecta hasta que el sensor detecte el color especificado.
        """
        self.drive_base.drive(velocidad, 0)
        
        while True:
            color_actual = sensor_color.color()
            if color_actual == color_objetivo:
                break
            wait(1)
            
        self.drive_base.stop()

    def girar_sobre_eje(self, grados):
        """Gira el robot sobre su propio eje."""
        self.drive_base.turn(grados)
    
    def giro_preciso(self, angulo_objetivo, kp_nuevo=2.5, tolerancia=1):
        """Gira el robot a un ángulo exacto usando el giroscopio y un Control Proporcional."""
        self.hub.imu.reset_heading(0) 
        
        kp = kp_nuevo
        min_speed = 50 
        tolerancia = tolerancia 
        
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_objetivo - angulo_actual
            
            if abs(error) <= tolerancia:
                break
                
            turn_rate = error * kp
            
            if turn_rate > 0:
                turn_rate = max(turn_rate, min_speed)
            else:
                turn_rate = min(turn_rate, -min_speed)
                
            self.drive_base.drive(0, turn_rate)
            wait(10)
            
        self.drive_base.stop()

    def mover_en_arco(self, radio_cm, angulo=None, distancia_cm=None, stop=Stop.HOLD, wait_after=True):
        """Realiza una curva suave."""
        radio_mm = radio_cm * 10
        distancia_mm = distancia_cm * 10 if distancia_cm is not None else None
        self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=wait_after)

    # endregion

    # region Control de Motores Individuales

    def mover_motor_izquierdo(self, grados, velocidad=500):
        """Mueve solo la rueda izquierda."""
        self.motor_izquierda.run_angle(velocidad, grados)

    def mover_motor_derecho(self, grados, velocidad=800, wait_after=True):
        """Mueve solo la rueda derecha."""
        self.motor_derecha.run_angle(velocidad, grados, wait=wait_after)

    def mover_garra(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD):
        """Mueve el motor de la garra/accesorio."""
        self.motor_garra.run_angle(velocidad, grados, then=frenado,wait=wait_after)
        
    def mover_garra_segura(self, grados, velocidad=600, empuje_cm=1):
        """
        Mueve la garra. Si detecta que se atascó contra algo, 
        avanza el chasis un poco para destrabarla.
        """
        # 1. Iniciamos el movimiento de la garra SIN pausar el código
        self.motor_garra.run_angle(velocidad, grados, wait=False)
        
        # 2. Monitoreamos mientras la garra siga en movimiento
        while not self.motor_garra.done():
            # Si el motor se atasca físicamente...
            if self.motor_garra.stalled():
                # Avanzamos el robot la distancia indicada para liberarlo
                self.avanzar_recto(empuje_cm)
            
            # Pequeña pausa de 10ms para no saturar el cerebro del hub
            wait(10)
    
    def mover_garra_dc(self, grados, potencia=100, empuje_cm=1):
        """
        Mueve la garra inyectando voltaje directo (DC) para máxima velocidad.
        Monitorea el ángulo para detenerse y la velocidad para detectar atascos.
        """
        angulo_inicial = self.motor_garra.angle()
        angulo_meta = angulo_inicial + grados
        
        # Determinar si el movimiento es positivo o negativo
        direccion = 1 if grados >= 0 else -1
        
        # 1. Inyectar potencia pura (por defecto 100%)
        # Aseguramos que el valor esté entre -100 y 100
        potencia_real = max(-100, min(100, potencia)) * direccion
        self.motor_garra.dc(potencia_real)
        
        # Pequeña pausa de gracia para que el motor empiece a moverse físicamente
        # antes de empezar a medir si está atascado
        wait(50)
        
        # 2. Bucle de monitoreo
        while True:
            angulo_actual = self.motor_garra.angle()
            
            # Condición de parada: Si ya llegamos o pasamos la meta
            if direccion == 1 and angulo_actual >= angulo_meta:
                break
            elif direccion == -1 and angulo_actual <= angulo_meta:
                break
                
            # Condición de atasco: Si el motor está recibiendo potencia pero casi no gira
            if abs(self.motor_garra.speed()) < 15:
                # Avanzamos el robot para destrabarlo
                self.avanzar_recto(empuje_cm)
                # Le damos un instante para que recupere velocidad tras liberarse
                wait(50) 
            
            # Pausa breve para no saturar el procesador
            wait(10)
            
        # 3. Frenar en seco al llegar a la meta
        self.motor_garra.hold()

    def giro_preciso_pd(self, angulo_relativo, max_speed=800, min_speed=40, kp=4.0, kd=18.0):
        """
        Gira el robot usando un Control PD dinámico.
        Permite ajustar la velocidad máxima y las constantes para giros de alta precisión.
        """
        angulo_inicial = self.hub.imu.heading()
        angulo_meta = angulo_inicial + angulo_relativo
        
        tolerancia = 1    
        error_previo = 0
        
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_meta - angulo_actual
            
            if abs(error) <= tolerancia:
                break
                
            derivada = error - error_previo
            turn_rate = (error * kp) + (derivada * kd)
            
            # Aplicamos los límites de velocidad dinámicos
            if turn_rate > 0:
                turn_rate = min(max(turn_rate, min_speed), max_speed)
            else:
                turn_rate = max(min(turn_rate, -min_speed), -max_speed)
                
            self.drive_base.drive(0, turn_rate)
            error_previo = error
            wait(10)
            
        self.drive_base.stop()

    def giro_eje_puro(self, angulo_relativo, kp=3.5, kd=15.0, max_speed=600, min_speed=30):
        """
        Fuerza un giro perfecto sobre el eje del robot ignorando el DriveBase.
        Manda la misma magnitud de velocidad a ambos motores en direcciones opuestas,
        reguladas por el giroscopio.
        """
        # 1. Detener cualquier orden previa de la base motriz
        self.drive_base.stop()
        
        angulo_inicial = self.hub.imu.heading()
        angulo_meta = angulo_inicial + angulo_relativo
        
        tolerancia = 1    
        error_previo = 0
        
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_meta - angulo_actual
            
            # Condición de salida
            if abs(error) <= tolerancia:
                break
                
            derivada = error - error_previo
            
            # Calculamos la magnitud bruta necesaria
            magnitud = abs((error * kp) + (derivada * kd))
            
            # Clamping dinámico para no ir muy rápido ni muy lento
            velocidad_giro = max(min_speed, min(magnitud, max_speed))
            
            # 2. Control directo e inverso de los motores
            # Si el error es positivo, necesitamos girar a la derecha (CW)
            # Izquierda avanza, Derecha retrocede.
            if error > 0:
                self.motor_izquierda.run(velocidad_giro)
                self.motor_derecha.run(-velocidad_giro)
            # Si el error es negativo, necesitamos girar a la izquierda (CCW)
            # Izquierda retrocede, Derecha avanza.
            else:
                self.motor_izquierda.run(-velocidad_giro)
                self.motor_derecha.run(velocidad_giro)
                
            error_previo = error
            wait(10)
            
        # 3. Freno en seco usando hold() en motores individuales para máxima retención
        self.motor_izquierda.hold()
        self.motor_derecha.hold()

    # endregion

    def seguir_linea(self, sensor_color, distancia_cm=None, velocidad=150, kp=3.6, kd=1.0):
        """Sigue la línea usando control PD inyectando voltaje directo (Duty Cycle)."""
        last_error = 0 
        
        if distancia_cm is not None:
            self.drive_base.reset()
            distancia_mm = distancia_cm * 10
            
        while True: 
            if distancia_cm is not None and abs(self.drive_base.distance()) >= distancia_mm:
                break
                
            current_reflection = sensor_color.reflection()
            error = current_reflection - 35
            derivative = error - last_error
            correction = (error * kp) + (derivative * kd)

            self.motor_izquierda.dc(velocidad + correction)
            self.motor_derecha.dc(velocidad - correction)
            
            last_error = error
            self.esperar(10)
            
        self.motor_izquierda.brake()
        self.motor_derecha.brake()

    def seguidor_linea_distancia(self, sensor_color, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800):
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_objetivo = (distancia_cm / circunferencia) * 360

        self.motor_izquierda.reset_angle(0)
        self.motor_derecha.reset_angle(0)

        cronometro = StopWatch()
        velocidad_minima = 25
        tiempo_aceleracion_ms = 0 
        
        # --- AJUSTES DEL CONTROLADOR ---
        kp = 0.85  # Reducido para evitar la oscilación agresiva
        kd = 2.5   # Aumentado para frenar el tambaleo (actúa como amortiguador)
        k_freno = 0.6 # Factor nuevo: reduce la velocidad si hay mucho error
        # -------------------------------
        
        last_error = 0
        objetivo_reflexion = 35

        multiplicador_lado = 1 if lado == "derecha" else -1

        cronometro.reset()
        cronometro.resume()

        while True:
            grados_actuales = (abs(self.motor_izquierda.angle()) + abs(self.motor_derecha.angle())) / 2

            if grados_actuales >= grados_objetivo:
                break

            tiempo_actual = cronometro.time()

            # Lógica de aceleración
            if tiempo_actual < tiempo_acomodo_ms:
                velocidad_actual = velocidad_minima
            elif tiempo_actual < (tiempo_acomodo_ms + tiempo_aceleracion_ms):
                tiempo_en_rampa = tiempo_actual - tiempo_acomodo_ms
                progreso = tiempo_en_rampa / tiempo_aceleracion_ms if tiempo_aceleracion_ms > 0 else 1
                velocidad_actual = velocidad_minima + ((velocidad_max - velocidad_minima) * progreso)
            else:
                velocidad_actual = velocidad_max

            # Lectura del sensor y cálculo de errores
            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            
            # Cálculo de la corrección
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado

            # --- NUEVO: REDUCCIÓN DINÁMICA DE VELOCIDAD ---
            # Si el robot se desvía (error alto), reduce su velocidad base para estabilizarse.
            # Si va perfectamente en la línea (error 0), va a la velocidad_actual máxima.
            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(velocidad_minima, velocidad_base) # Nunca ir más lento que la mínima

            # Cálculo final de motores
            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction

            # --- NUEVO: LÍMITES DE SEGURIDAD (CLAMPING) ---
            # Evita que el .dc() reciba valores fuera del rango permitido (-100 a 100)
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))

            self.motor_izquierda.dc(potencia_izq)
            self.motor_derecha.dc(potencia_der)

            last_error = error
            wait(1)

        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        cronometro.pause()

    def seguidor_linea_color(self, sensor_color, velocidad_max, color_objetivo, lado="derecha", tiempo_acomodo_ms=800, distancia_cm=None):
        """
        Sigue la línea usando control PD.
        Si se pasa una distancia_cm, desacelera suavemente hasta velocidad 50 antes de llegar a esa marca.
        Se detiene cuando el sensor detecta el color_objetivo.
        """
        cronometro = StopWatch()
        velocidad_minima = 25
        velocidad_enfoque = 50 # Velocidad segura para leer el color
        tiempo_aceleracion_ms = 0 
        
        # Límite por seguridad para la lectura (si no hay desaceleración)
        if distancia_cm is None:
            velocidad_max = min(velocidad_max, 70) 
            
        # --- AJUSTES DEL CONTROLADOR ---
        kp = 0.85  
        kd = 2.5   
        k_freno = 0.6 
        # -------------------------------
        
        last_error = 0
        objetivo_reflexion = 35
        multiplicador_lado = 1 if lado == "derecha" else -1

        # Si hay distancia, preparamos la matemática de las ruedas
        if distancia_cm is not None:
            diametro_rueda = 5.6
            circunferencia = 3.1416 * diametro_rueda
            grados_objetivo = (distancia_cm / circunferencia) * 360
            self.motor_izquierda.reset_angle(0)
            self.motor_derecha.reset_angle(0)

        cronometro.reset()
        cronometro.resume()

        while True:
            # --- CONDICIÓN DE PARADA ---
            if sensor_color.color() == color_objetivo:
                break

            tiempo_actual = cronometro.time()

            # 1. Lógica de aceleración (Rampa por tiempo)
            if tiempo_actual < tiempo_acomodo_ms:
                vel_aceleracion = velocidad_minima
            elif tiempo_actual < (tiempo_acomodo_ms + tiempo_aceleracion_ms):
                tiempo_en_rampa = tiempo_actual - tiempo_acomodo_ms
                progreso = tiempo_en_rampa / tiempo_aceleracion_ms if tiempo_aceleracion_ms > 0 else 1
                vel_aceleracion = velocidad_minima + ((velocidad_max - velocidad_minima) * progreso)
            else:
                vel_aceleracion = velocidad_max

            # 2. Lógica de desaceleración (Rampa por distancia)
            if distancia_cm is not None:
                grados_actuales = (abs(self.motor_izquierda.angle()) + abs(self.motor_derecha.angle())) / 2
                
                # Calculamos qué porcentaje de la ruta hemos completado (de 0.0 a 1.0)
                progreso_distancia = grados_actuales / grados_objetivo
                progreso_distancia = min(1.0, progreso_distancia) # Evitar que pase de 100%
                
                # Interpolación: Baja de velocidad_max a 50 gradualmente
                vel_desaceleracion = velocidad_max - ((velocidad_max - velocidad_enfoque) * progreso_distancia)
                vel_desaceleracion = max(velocidad_enfoque, vel_desaceleracion) # Nunca bajar de 50 aquí
                
                # La velocidad real será la menor entre lo que dicta la aceleración y la desaceleración
                velocidad_actual = min(vel_aceleracion, vel_desaceleracion)
            else:
                velocidad_actual = vel_aceleracion

            # Lectura de la línea (reflexión)
            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            
            # Cálculo de la corrección PD
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado

            # Reducción dinámica de velocidad por desviación
            velocidad_base = velocidad_actual - (abs(error) * k_freno)
            velocidad_base = max(velocidad_minima, velocidad_base) 

            # Mezcla para los motores
            potencia_izq = velocidad_base - correction
            potencia_der = velocidad_base + correction

            # Límites de seguridad (Clamping)
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))

            self.motor_izquierda.dc(potencia_izq)
            self.motor_derecha.dc(potencia_der)

            last_error = error
            wait(1)

        # Freno en seco al detectar el color
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        cronometro.pause()

    def identificar_combinacion(self, sensor, distancia_si_verde):
        """
        Identifica una combinación de colores minimizando las lecturas del sensor.
        Retorna el ID de la combinación (1-5) o -1 si hay un error de lectura.
        """
        # 1. Realizamos la primera y única lectura obligatoria.
        color_principal = sensor.color()

        # 2. Búsqueda rápida en el diccionario principal.
        if color_principal not in mosaicos:
            print("Error: Color principal no reconocido")
            # SOLUCIÓN: Usamos f-strings para imprimir el objeto Color sin errores
            print(f"Color escaneado: {color_principal}") 
            return -1  

        decision = mosaicos[color_principal]

        # 3. Verificamos si necesitamos desempatar (Caso VERDE).
        if type(decision) is dict:
            self.avanzar_recto(distancia_si_verde)

            # 4. SOLO en este caso gastamos tiempo leyendo el sensor por segunda vez.
            color_anterior = sensor.color()

            # Verificamos si la segunda lectura es válida antes de retornar
            if color_anterior not in decision:
                print(f"Error: El segundo color ({color_anterior}) no completa un patrón verde válido.")
                return -1
                
            return decision[color_anterior]

        # 5. Si no era un diccionario, retornamos el ID directo.
        return decision
    
    def esperar(self, milisegundos):
        """Pausa el programa."""
        wait(milisegundos)

    def resetear_giroscopio(self):
        """Reinicia el ángulo del giroscopio a 0."""
        self.hub.imu.reset_heading(0)
        
    def obtener_angulo(self):
        """Devuelve el ángulo actual del robot."""
        return self.hub.imu.heading()