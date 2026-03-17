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
    def __init__(self, port_izq, port_der, port_eje_central, port_garra_delantera): 
        """
        Configuración inicial del robot.
        """
        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        
        # Motores de tracción
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE)
        self.motor_derecha = Motor(port_der, Direction.CLOCKWISE)
        
        # Motores de mecanismos
        self.motor_eje_central = Motor(port_eje_central, Direction.COUNTERCLOCKWISE)
        self.motor_garra_delantera = Motor(port_garra_delantera)
        
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, 56, 160)
        self.drive_base.use_gyro(True)
        
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
        self.drive_base.drive(velocidad, 0)
        while True:
            if sensor_color.color() == color_objetivo:
                break
            wait(1)
        self.drive_base.stop()

    def girar_sobre_eje(self, grados):
        self.drive_base.turn(grados)
    
    def giro_preciso(self, angulo_objetivo, kp_nuevo=2.5, tolerancia=1):
        self.hub.imu.reset_heading(0) 
        kp = kp_nuevo
        min_speed = 50 
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
        radio_mm = radio_cm * 10
        distancia_mm = distancia_cm * 10 if distancia_cm is not None else None
        self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=wait_after)
    # endregion

    # region Control de Motores Individuales
    def mover_motor_izquierdo(self, grados, velocidad=500):
        self.motor_izquierda.run_angle(velocidad, grados)

    def mover_motor_derecho(self, grados, velocidad=800, wait_after=True):
        self.motor_derecha.run_angle(velocidad, grados, wait=wait_after)
    # endregion

    # region GARRA DELANTERA
    def abrir_garra_delantera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD):
        """Abre los brazos de la garra delantera moviendo el motor en positivo."""
        self.motor_garra_delantera.run_angle(velocidad, abs(grados), then=frenado, wait=wait_after)

    def cerrar_garra_delantera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD):
        """Cierra los brazos de la garra delantera moviendo el motor en negativo."""
        self.motor_garra_delantera.run_angle(velocidad, -abs(grados), then=frenado, wait=wait_after)
    
    def abrir_garra_delantera_al_tope(self, velocidad=800, limite_potencia=50):
        """
        Abre la garra frontal hasta el tope mecánico. 
        Si ya está totalmente abierta, detecta el atasco rápido y el código continúa.
        """
        # Forzamos velocidad positiva para asegurar que abra
        self.motor_garra_delantera.run_until_stalled(abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)

    def cerrar_garra_delantera_al_tope(self, velocidad=800, limite_potencia=50):
        """
        Cierra la garra frontal hasta atrapar el objeto o llegar al tope mecánico.
        Si ya está cerrada, detecta el atasco rápido y el código continúa.
        """
        # Forzamos velocidad negativa para asegurar que cierre
        self.motor_garra_delantera.run_until_stalled(-abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)

    # endregion
    # endregion

    # region EJE CENTRAL Y GARRA TRASERA
    def mover_eje_central(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD):
        """Función base que controla el eje central de los engranajes."""
        self.motor_eje_central.run_angle(velocidad, grados, then=frenado, wait=wait_after)
    
    def llevar_eje_central_al_tope(self, direccion, velocidad=1000, limite_potencia=60):
        """
        Mueve el eje central a máxima velocidad hasta chocar con el límite físico.
        
        :param direccion: "positivo" o "negativo" (también acepta 1 o -1).
        :param velocidad: Velocidad en grados/segundo (1000 es el máximo práctico).
        :param limite_potencia: Fuerza máxima (0-100). 60% permite ir rápido pero frena seguro al chocar.
        :return: El ángulo exacto en el que se atascó.
        """
        # Validamos la dirección elegida
        if direccion == "positivo" or direccion == 1:
            vel_real = abs(velocidad)
        elif direccion == "negativo" or direccion == -1:
            vel_real = -abs(velocidad)
        else:
            print("Error: La dirección debe ser 'positivo' o 'negativo'.")
            return None
            
        # Ejecutamos el movimiento hasta el atasco
        # then=Stop.HOLD asegura que una vez en el tope, el motor mantenga la posición y no se caiga
        angulo_tope = self.motor_eje_central.run_until_stalled(vel_real, then=Stop.HOLD, duty_limit=limite_potencia)
        
        return angulo_tope

    def mover_garra_trasera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD):
        """Alias para mantener claridad en el código. Llama a mover_eje_central."""
        self.mover_eje_central(grados, velocidad, wait_after, frenado)
        
    def subir_garra_trasera(self, grados, velocidad=600):
        """Sube la garra trasera (asegura movimiento en positivo)."""
        self.mover_eje_central(abs(grados), velocidad)

    def bajar_garra_trasera(self, grados, velocidad=600):
        """Baja la garra trasera (asegura movimiento en negativo)."""
        self.mover_eje_central(-abs(grados), velocidad)

    def mover_garra_trasera_segura(self, grados, velocidad=600, empuje_cm=1):
        """Mueve la garra trasera con protección contra atascos."""
        self.motor_eje_central.run_angle(velocidad, grados, wait=False)
        while not self.motor_eje_central.done():
            if self.motor_eje_central.stalled():
                self.avanzar_recto(empuje_cm)
            wait(10)
    
    def mover_garra_trasera_dc(self, grados, potencia=100, empuje_cm=1):
        """Mueve la garra trasera inyectando voltaje directo (DC)."""
        angulo_inicial = self.motor_eje_central.angle()
        angulo_meta = angulo_inicial + grados
        direccion = 1 if grados >= 0 else -1
        
        potencia_real = max(-100, min(100, potencia)) * direccion
        self.motor_eje_central.dc(potencia_real)
        wait(50)
        
        while True:
            angulo_actual = self.motor_eje_central.angle()
            if direccion == 1 and angulo_actual >= angulo_meta:
                break
            elif direccion == -1 and angulo_actual <= angulo_meta:
                break
                
            if abs(self.motor_eje_central.speed()) < 15:
                self.avanzar_recto(empuje_cm)
                wait(50) 
            wait(10)
            
        self.motor_eje_central.hold()
    # endregion

    # region Giros Complejos y Seguidores (Sin Cambios)
    def giro_preciso_pd(self, angulo_relativo, max_speed=800, min_speed=40, kp=4.0, kd=18.0):
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
            if turn_rate > 0:
                turn_rate = min(max(turn_rate, min_speed), max_speed)
            else:
                turn_rate = max(min(turn_rate, -min_speed), -max_speed)
            self.drive_base.drive(0, turn_rate)
            error_previo = error
            wait(10)
        self.drive_base.stop()

    def giro_eje_puro(self, angulo_relativo, kp=3.5, kd=15.0, max_speed=600, min_speed=30):
        self.drive_base.stop()
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

    def seguir_linea(self, sensor_color, distancia_cm=None, velocidad=150, kp=3.6, kd=1.0):
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

    def seguidor_linea_distancia(self, sensor_color, velocidad_max, distancia_cm, lado="derecha", tiempo_acomodo_ms=800, kp = 0.85, kd = 2.5, k_freno = 0.6):
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_objetivo = (distancia_cm / circunferencia) * 360
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
            if grados_actuales >= grados_objetivo:
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

    def seguidor_linea_color(self, sensor_color, velocidad_max, color_objetivo, lado="derecha", tiempo_acomodo_ms=800, distancia_cm=None):
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
        if distancia_cm is not None:
            diametro_rueda = 5.6
            circunferencia = 3.1416 * diametro_rueda
            grados_objetivo = (distancia_cm / circunferencia) * 360
            self.motor_izquierda.reset_angle(0)
            self.motor_derecha.reset_angle(0)
        cronometro.reset()
        cronometro.resume()
        while True:
            if sensor_color.color() == color_objetivo:
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

    def acomodar_en_linea(self, sensor_color, velocidad_base=50, lado="derecha", tiempo_estabilizado_ms=250, distancia_max_cm=15):
        diametro_rueda = 5.6
        circunferencia = 3.1416 * diametro_rueda
        grados_maximos = (distancia_max_cm / circunferencia) * 360 if distancia_max_cm else None
        self.motor_izquierda.reset_angle(0)
        self.motor_derecha.reset_angle(0)
        cronometro = StopWatch()
        velocidad_minima = 25
        kp = 0.85 
        kd = 2.5  
        k_freno = 0.6 
        last_error = 0
        objetivo_reflexion = 35
        multiplicador_lado = 1 if lado == "derecha" else -1
        tiempo_estable_inicio = 0
        tolerancia_error = 5      
        tolerancia_correccion = 5 
        cronometro.reset()
        cronometro.resume()
        while True:
            grados_actuales = (abs(self.motor_izquierda.angle()) + abs(self.motor_derecha.angle())) / 2
            if grados_maximos and grados_actuales >= grados_maximos:
                break 
            current_reflection = sensor_color.reflection()
            error = current_reflection - objetivo_reflexion
            derivative = error - last_error
            correction = ((error * kp) + (derivative * kd)) * multiplicador_lado
            if abs(error) <= tolerancia_error and abs(correction) <= tolerancia_correccion:
                if tiempo_estable_inicio == 0:
                    tiempo_estable_inicio = cronometro.time()
                elif cronometro.time() - tiempo_estable_inicio >= tiempo_estabilizado_ms:
                    break 
            else:
                tiempo_estable_inicio = 0 
            velocidad_actual = velocidad_base - (abs(error) * k_freno)
            velocidad_actual = max(velocidad_minima, velocidad_actual) 
            potencia_izq = velocidad_actual - correction
            potencia_der = velocidad_actual + correction
            potencia_izq = max(-100, min(100, potencia_izq))
            potencia_der = max(-100, min(100, potencia_der))
            self.motor_izquierda.dc(potencia_izq)
            self.motor_derecha.dc(potencia_der)
            last_error = error
            wait(10) 
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        cronometro.pause()
        grados_finales = (abs(self.motor_izquierda.angle()) + abs(self.motor_derecha.angle())) / 2
        distancia_recorrida_cm = (grados_finales / 360) * circunferencia
        return distancia_recorrida_cm

    def identificar_combinacion(self, sensor, distancia_si_verde):
        color_principal = sensor.color()
        if color_principal not in mosaicos:
            print("Error: Color principal no reconocido")
            print(f"Color escaneado: {color_principal}") 
            return -1  
        decision = mosaicos[color_principal]
        if type(decision) is dict:
            self.avanzar_recto(distancia_si_verde)
            color_anterior = sensor.color()
            if color_anterior not in decision:
                print(f"Error: El segundo color ({color_anterior}) no completa un patrón verde válido.")
                return -1
            return decision[color_anterior]
        return decision
    
    def esperar(self, milisegundos):
        wait(milisegundos)

    def resetear_giroscopio(self):
        self.hub.imu.reset_heading(0)
        
    def obtener_angulo(self):
        return self.hub.imu.heading()
    # endregion
