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

        El tope es config.STRAIGHT_SPEED, que es un valor que el drive_base ya
        acepta hoy. Asi los movimientos rapidos quedan exactamente igual que
        antes y solo cambian los que pedian ir mas lento.
        """
        if velocidad is None:
            velocidad = self.velocidad_base
        if aceleracion is None:
            aceleracion = config.STRAIGHT_ACCEL

        velocidad = max(1, min(abs(velocidad), config.STRAIGHT_SPEED))
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
    
    def avance_milimetrico(self, milimetros, velocidad=400):
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        wait(100) 
        
        circunferencia = config.DIAMETRO_RUEDA * 3.1416
        grados_a_mover = (milimetros / circunferencia) * 360
        
        self.motor_izquierda.run_angle(velocidad, grados_a_mover, then=Stop.HOLD, wait=False)
        self.motor_derecha.run_angle(velocidad, grados_a_mover, then=Stop.HOLD, wait=True)
        
        wait(50)
        Utils.emitir_sonido_confirmacion(self.hub)

    def cuadrar_contra_pared(self, tiempo_ms=1000, potencia=30, angulo_referencia=0, reversa=True,
                             umbral_velocidad=40, arranque_ms=50, asentamiento_ms=50):
        """
        Empuja contra la pared hasta quedar plano y reinicia el rumbo.

        tiempo_ms es el TOPE, no una espera fija. El metodo sale en cuanto
        las dos ruedas se frenan contra la pared, que es la senal de que el
        robot quedo a escuadra. Antes se esperaba el tiempo completo: si era
        de mas se regalaba tiempo, y si era de menos se reiniciaba el rumbo
        con el robot todavia torcido, que es peor.

        arranque_ms: los motores parten de velocidad cero, asi que hay que
        darles un momento antes de empezar a mirar si estan frenados.
        asentamiento_ms: se sigue empujando un instante despues de detectar
        el tope, para que el robot termine de alinearse antes de fijar el cero.
        """
        self.drive_base.stop()
        potencia_aplicada = -potencia if reversa else potencia

        self.motor_izquierda.dc(potencia_aplicada)
        self.motor_derecha.dc(potencia_aplicada)

        margen = arranque_ms + asentamiento_ms
        if tiempo_ms <= margen:
            # Muy corto para detectar nada: se respeta la espera fija.
            wait(tiempo_ms)
        else:
            wait(arranque_ms)
            reloj_seg = StopWatch()
            while reloj_seg.time() < (tiempo_ms - margen):
                if (abs(self.motor_izquierda.speed()) < umbral_velocidad and
                        abs(self.motor_derecha.speed()) < umbral_velocidad):
                    break
                wait(5)
            wait(asentamiento_ms)

        self.motor_izquierda.brake()
        self.motor_derecha.brake()
        self.hub.imu.reset_heading(angulo_referencia)

        Utils.emitir_sonido_confirmacion(self.hub)
    def chocar_inteligente(self, distancia_acercamiento_cm, velocidad_acercamiento=800, potencia_choque=35, timeout_choque_ms=1500):
        es_reversa = distancia_acercamiento_cm < 0
        potencia_real = -abs(potencia_choque) if es_reversa else abs(potencia_choque)
        
        self.avanzar_recto(distancia_acercamiento_cm, velocidad=velocidad_acercamiento, wait_after=True)
        self.avanzar_hasta_choque(potencia=potencia_real, umbral_velocidad=20, tiempo_arranque_ms=150, timeout_ms=timeout_choque_ms)
        Utils.emitir_sonido_confirmacion(self.hub) 
            
    def mover_en_arco(self, radio_cm, angulo=None, velocidad=None, aceleracion=None, distancia_cm=None, stop=Stop.HOLD, wait_after=True, margen_grados=0, margen_cm=0, encadenado=False):
        self._aplicar_velocidad(velocidad, aceleracion)
        radio_mm = radio_cm * 10
        distancia_mm = distancia_cm * 10 if distancia_cm is not None else None
        if encadenado: stop = Stop.NONE

        if wait_after and (margen_grados > 0 or margen_cm > 0):
            self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=False)
            
            if distancia_cm is not None and margen_cm > 0:
                dist_inicial = self.drive_base.distance()
                margen_mm_real = abs(margen_cm * 10)
                meta_mm = abs(distancia_mm)
                reloj_seg = StopWatch()
                while abs(self.drive_base.distance() - dist_inicial) < (meta_mm - margen_mm_real):
                    if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                        print("TIMEOUT mover_en_arco (distancia)")
                        break
                    if self.drive_base.stalled(): break
                    wait(2)
                if encadenado: self._terminar_movimiento_encadenado()
            elif angulo is not None and margen_grados > 0:
                ang_inicial = self.drive_base.angle()
                meta_ang = abs(angulo)
                reloj_seg = StopWatch()
                while abs(self.drive_base.angle() - ang_inicial) < (meta_ang - margen_grados):
                    if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                        print("TIMEOUT mover_en_arco (angulo)")
                        break
                    if self.drive_base.stalled(): break
                    wait(2)
                if encadenado: self._terminar_movimiento_encadenado()
        else:
            self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=wait_after)
            if wait_after and encadenado: self._terminar_movimiento_encadenado()
            
        if not encadenado: Utils.emitir_sonido_confirmacion(self.hub)

    def girar_sobre_eje(self, grados, wait_after=True, margen_grados=0, encadenado=False):
        if wait_after and margen_grados > 0:
            angulo_inicial = self.drive_base.angle()
            self.drive_base.turn(grados, wait=False)
            reloj_seg = StopWatch()
            while abs(self.drive_base.angle() - angulo_inicial) < (abs(grados) - margen_grados):
                if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                    print("TIMEOUT girar_sobre_eje")
                    break
                if self.drive_base.stalled(): break
                wait(2)
            if encadenado: self._terminar_movimiento_encadenado()
        else:
            self.drive_base.turn(grados, wait=wait_after)
            if wait_after and encadenado: self._terminar_movimiento_encadenado()
            
        if not encadenado: Utils.emitir_sonido_confirmacion(self.hub)

    def giro_preciso(self, angulo_objetivo, kp_nuevo=2.5, tolerancia=1, margen_grados=0, encadenado=False):
        angulo_inicial = self.hub.imu.heading()
        angulo_meta = angulo_inicial + angulo_objetivo
        if abs(angulo_objetivo) < config.BANDA_MUERTA_GIRO:
            return
        kp = kp_nuevo
        min_speed = 50 
        reloj_seg = StopWatch()
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_meta - angulo_actual
            if abs(error) <= max(tolerancia, margen_grados):
                break
            if reloj_seg.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT giro_preciso: faltaban %d grados" % error)
                break
            turn_rate = error * kp
            turn_rate = max(turn_rate, min_speed) if turn_rate > 0 else min(turn_rate, -min_speed)
            self.drive_base.drive(0, turn_rate)
            wait(10)
            
        if encadenado:
            self._terminar_movimiento_encadenado()
        else:
            self.drive_base.stop()
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

    def avanzar_hasta_choque(self, potencia=100, umbral_velocidad=50, tiempo_arranque_ms=300, timeout_ms=None):
        self.drive_base.stop() 
        self.motor_izquierda.dc(potencia)
        self.motor_derecha.dc(potencia)
        wait(tiempo_arranque_ms)
        
        # Sin tope, si las ruedas patinan en el aire el bucle no sale nunca.
        if timeout_ms is None:
            timeout_ms = config.TIMEOUT_MOVIMIENTO_MS
        reloj_seg = StopWatch()
        paso_ms = 10 
        
        while True:
            vel_izq = abs(self.motor_izquierda.speed())
            vel_der = abs(self.motor_derecha.speed())
            if vel_izq < umbral_velocidad and vel_der < umbral_velocidad:
                break
            if reloj_seg.time() >= timeout_ms:
                print("TIMEOUT avanzar_hasta_choque")
                break
            wait(paso_ms)
            
        self.motor_izquierda.hold()
        self.motor_derecha.hold()
        Utils.emitir_sonido_confirmacion(self.hub)
    
    def latigazo(self, grados=45, velocidad=1000, wait_after=True, encadenado=False):
        self.drive_base.stop()
        
        pos_izq_inicial = self.motor_izquierda.angle()
        pos_der_inicial = self.motor_derecha.angle()
        
        grados_rueda = abs(grados * 3.0)
        
        dir_izq = 1 if grados > 0 else -1
        dir_der = -1 if grados > 0 else 1
        
        self.motor_izquierda.run(velocidad * dir_izq)
        self.motor_derecha.run(velocidad * dir_der)
        
        reloj_seg = StopWatch()
        while abs(self.motor_izquierda.angle() - pos_izq_inicial) < grados_rueda:
            if self.motor_izquierda.stalled() or reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                print("TIMEOUT latigazo")
                break
            wait(1)
            
        self.motor_izquierda.run_target(velocidad, pos_izq_inicial, wait=False)
        
        # El motor derecho determina si bloqueamos la ejecución o seguimos adelante
        self.motor_derecha.run_target(velocidad, pos_der_inicial, wait=wait_after)
        
        if wait_after:
            if encadenado:
                self._terminar_movimiento_encadenado()
            else:
                self.motor_izquierda.hold()
                self.motor_derecha.hold()
                Utils.emitir_sonido_confirmacion(self.hub)

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

    def avanzar_y_accionar_en_recorrido(self, distancia_total_cm, distancia_accion_cm, accion_callback, frenado=Stop.BRAKE, margen_cm=0, encadenado=False):
        """
        Avanza (o retrocede) una distancia total y ejecuta una función (callback) 
        al alcanzar un centímetro específico sin detener el movimiento del chasis.
        """
        distancia_total_mm = distancia_total_cm * 10
        # Usamos valor absoluto para soportar recorridos en reversa (distancia_total_cm negativo)
        distancia_accion_mm = abs(distancia_accion_cm * 10) 
        margen_mm = abs(margen_cm * 10)

        if encadenado:
            frenado = Stop.NONE

        dist_inicial = self.drive_base.distance()
        meta_mm = abs(distancia_total_mm)
        accion_ejecutada = False

        self.drive_base.straight(distancia_total_mm, then=frenado, wait=False)

        reloj_seg = StopWatch()
        while True:
            if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                print("TIMEOUT avanzar_y_accionar_en_recorrido")
                break
            dist_actual = abs(self.drive_base.distance() - dist_inicial)

            if not accion_ejecutada and dist_actual >= distancia_accion_mm:
                accion_callback()
                accion_ejecutada = True

            if dist_actual >= (meta_mm - margen_mm):
                break

            if self.drive_base.stalled():
                break

            wait(2) # Micro-pausa para no saturar el procesador del hub

        if encadenado:
            self._terminar_movimiento_encadenado()
        else:
            Utils.emitir_sonido_confirmacion(self.hub)
    
    def avanzar_indefinido(self, velocidad=None):
        """
        Inicia un avance recto indefinido y no bloqueante.
        No usa 'encadenado' al final ni sonido porque la acción queda en segundo plano.
        """
        if velocidad is None:
            velocidad = self.velocidad_base
            
        velocidad = max(min(velocidad, 976), -976)
        self.drive_base.drive(velocidad, 0)

    def detener(self, encadenado=False):
        """
        Detiene cualquier movimiento en progreso (como avanzar_indefinido).
        Si encadenado=True, usa tu micro-freno pasivo en lugar de un freno en seco.
        """
        if encadenado:
            self._terminar_movimiento_encadenado()
        else:
            self.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.hub)