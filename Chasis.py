from pybricks.parameters import Stop
from pybricks.tools import StopWatch, wait
import config
from Utils import Utils

class Chasis:
    def __init__(self, drive_base, motor_izq, motor_der, hub, velocidad_base):
        self.drive_base = drive_base
        self.motor_izquierda = motor_izq
        self.motor_derecha = motor_der
        self.hub = hub
        self.velocidad_base = velocidad_base
        # Ultimo (velocidad, aceleracion) aplicado al drive_base. Sirve para no
        # llamar a settings() en cada movimiento: solo cuando el valor cambia.
        self._ajustes_aplicados = None

    def _aplicar_velocidad(self, velocidad, aceleracion=None):
        """
        Traduce un pedido de velocidad a settings() del drive_base.

        straight() y arc() no aceptan velocidad: la toman de settings(). Sin
        esto el parametro 'velocidad' se calculaba y se tiraba a la basura, y
        los movimientos lentos de aproximacion corrian a STRAIGHT_SPEED.

        El tope es config.VELOCIDAD_RECTA, que es un valor que el drive_base ya
        acepta hoy. Asi los movimientos rapidos quedan exactamente igual que
        antes y solo cambian los que pedian ir mas lento.
        """
        if velocidad is None:
            velocidad = self.velocidad_base
        if aceleracion is None:
            aceleracion = config.ACELERACION_RECTA

        velocidad = max(1, min(abs(velocidad), config.VELOCIDAD_RECTA))
        ajustes = (velocidad, aceleracion)

        if ajustes != self._ajustes_aplicados:
            self.drive_base.settings(straight_speed=velocidad,
                                     straight_acceleration=aceleracion)
            self._ajustes_aplicados = ajustes

    def _terminar_movimiento_encadenado(self):
        """Micro-freno pasivo que libera tensión electromagnética para enlazar rápido."""
        self.motor_izquierda.brake()
        self.motor_derecha.brake()
        wait(4)
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        wait(2)

    def avanzar_recto(self, distancia_cm, velocidad=None, aceleracion=None, frenado=Stop.BRAKE, wait_after=True, margen_cm=0, encadenado=False):
        self._aplicar_velocidad(velocidad, aceleracion)
        distancia_mm = distancia_cm * 10
        
        if encadenado:
            frenado = Stop.NONE
            
        if wait_after and margen_cm > 0:
            distancia_inicial = self.drive_base.distance()
            margen_mm = abs(margen_cm * 10)
            # Si el margen se come toda la distancia, la condicion del while es
            # falsa en la primera vuelta y el movimiento se cancela a los pocos
            # milisegundos sin que nadie se entere.
            if margen_mm >= abs(distancia_mm):
                print("AVISO avanzar_recto: margen_cm=%d >= distancia=%d cm, el movimiento no espera nada"
                      % (margen_cm, abs(distancia_cm)))
            self.drive_base.straight(distancia_mm, then=frenado, wait=False)
            
            reloj_seg = StopWatch()
            while abs(self.drive_base.distance() - distancia_inicial) < (abs(distancia_mm) - margen_mm):
                if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                    print("TIMEOUT avanzar_recto")
                    break
                if self.drive_base.stalled():
                    break
                wait(2)
            if encadenado:
                self._terminar_movimiento_encadenado()
        else:
            self.drive_base.straight(distancia_mm, then=frenado, wait=wait_after)
            if wait_after and encadenado:
                self._terminar_movimiento_encadenado()
                
        if not encadenado:
            Utils.emitir_sonido_confirmacion(self.hub)
    
    def cuadrar_contra_pared(self, tiempo_ms=1000, potencia=30, angulo_referencia=0, reversa=True):
        self.drive_base.stop()
        potencia_aplicada = -potencia if reversa else potencia
        
        self.motor_izquierda.dc(potencia_aplicada)
        self.motor_derecha.dc(potencia_aplicada)
        wait(tiempo_ms) 
        
        self.motor_izquierda.brake()
        self.motor_derecha.brake()
        self.hub.imu.reset_heading(angulo_referencia) 
        
        Utils.emitir_sonido_confirmacion(self.hub)
    
    def mover_motor_izquierdo(self, grados, velocidad=500, wait_after=True, frenado=Stop.HOLD, margen_grados=0, encadenado=False):
        if encadenado: frenado = Stop.NONE
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_izquierda.angle() + grados
            self.motor_izquierda.run_angle(velocidad, grados, then=frenado, wait=False) 
            reloj_seg = StopWatch()
            while abs(angulo_meta - self.motor_izquierda.angle()) > margen_grados:
                if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                    print("TIMEOUT mover_motor_izquierdo")
                    break
                if self.motor_izquierda.stalled(): break
                wait(2)
            if encadenado:
                self.motor_izquierda.brake()
                wait(4)
                self.motor_izquierda.stop()
        else:
            self.motor_izquierda.run_angle(velocidad, grados, then=frenado, wait=wait_after) 
            if wait_after and encadenado:
                self.motor_izquierda.brake()
                wait(4)
                self.motor_izquierda.stop()
        if not encadenado: Utils.emitir_sonido_confirmacion(self.hub)

    def mover_motor_derecho(self, grados, velocidad=800, wait_after=True, frenado=Stop.HOLD, margen_grados=0, encadenado=False):
        if encadenado: frenado = Stop.NONE
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_derecha.angle() + grados
            self.motor_derecha.run_angle(velocidad, grados, then=frenado, wait=False)
            reloj_seg = StopWatch()
            while abs(angulo_meta - self.motor_derecha.angle()) > margen_grados:
                if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                    print("TIMEOUT mover_motor_derecho")
                    break
                if self.motor_derecha.stalled(): break
                wait(2)
            if encadenado:
                self.motor_derecha.brake()
                wait(4)
                self.motor_derecha.stop()
        else:
            self.motor_derecha.run_angle(velocidad, grados, then=frenado, wait=wait_after)
            if wait_after and encadenado:
                self.motor_derecha.brake()
                wait(4)
                self.motor_derecha.stop()
        if not encadenado: Utils.emitir_sonido_confirmacion(self.hub)

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
        Utils.emitir_sonido_confirmacion(self.hub)

    def compensar_voltaje(self, potencia_deseada):
        voltaje_actual = self.hub.battery.voltage()
        if voltaje_actual == 0: return potencia_deseada
        return max(-100, min(100, potencia_deseada * (8000 / voltaje_actual)))

    def avanzar_y_accionar_en_recorrido(self, distancia_total_cm, distancia_accion_cm, accion_callback, 
                                        frenado=Stop.BRAKE, margen_cm=0, encadenado=False,
                                        accion_sec_callback=None, delay_sec_ms=0):
        """
        Avanza (o retrocede) una distancia total y ejecuta una función (callback).
        Opcionalmente ejecuta una segunda función después de un delay en milisegundos, 
        sin detener la lectura de los sensores.
        """
        distancia_total_mm = distancia_total_cm * 10
        distancia_accion_mm = abs(distancia_accion_cm * 10) 
        margen_mm = abs(margen_cm * 10)

        if encadenado:
            frenado = Stop.NONE

        dist_inicial = self.drive_base.distance()
        meta_mm = abs(distancia_total_mm)
        
        # Variables de control de estado
        accion_ejecutada = False
        # Si no hay accion secundaria, la marcamos como ejecutada desde el inicio
        accion_secundaria_ejecutada = False if accion_sec_callback else True
        tiempo_accion_1 = 0

        self.drive_base.straight(distancia_total_mm, then=frenado, wait=False)

        reloj_seg = StopWatch()
        while True:
            if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                print("TIMEOUT avanzar_y_accionar_en_recorrido")
                break
                
            dist_actual = abs(self.drive_base.distance() - dist_inicial)

            # 1. Se ejecuta la primera acción al llegar a la distancia
            if not accion_ejecutada and dist_actual >= distancia_accion_mm:
                accion_callback()
                accion_ejecutada = True
                tiempo_accion_1 = reloj_seg.time() # Guardamos en qué momento exacto se ejecutó

            # 2. Se evalúa si ya pasó el tiempo para la segunda acción
            if accion_ejecutada and not accion_secundaria_ejecutada:
                if (reloj_seg.time() - tiempo_accion_1) >= delay_sec_ms:
                    accion_sec_callback()
                    accion_secundaria_ejecutada = True

            # 3. Condiciones de salida del bucle
            if dist_actual >= (meta_mm - margen_mm):
                break

            if self.drive_base.stalled():
                break

            wait(2) # Micro-pausa

        if encadenado:
            self._terminar_movimiento_encadenado()
        else:
            Utils.emitir_sonido_confirmacion(self.hub)
            
        # CASO EXTREMO: Si el robot terminó su recorrido ANTES de que el contador del delay
        # llegara a su fin, esperamos lo que falta y ejecutamos la acción secundaria.
        if accion_ejecutada and not accion_secundaria_ejecutada:
            tiempo_faltante = delay_sec_ms - (reloj_seg.time() - tiempo_accion_1)
            if tiempo_faltante > 0:
                wait(tiempo_faltante) # Aquí sí es seguro usar wait porque el chasis ya se detuvo
            accion_sec_callback()