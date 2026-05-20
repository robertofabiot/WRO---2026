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