from pybricks.parameters import Stop
from pybricks.tools import StopWatch, wait
import config
from Utils import Utils

class Chasis:
    """Movimientos del tren de traccion: rectas, cuadratura y motores sueltos.

    Los giros no viven aca: son de Navegacion, que trabaja contra el IMU.
    """

    def __init__(self, drive_base, motor_izquierdo, motor_derecho, hub, velocidad_base):
        """
        Argumentos:
            drive_base: DriveBase de Pybricks, ya configurado con el giroscopio.
            motor_izquierdo, motor_derecho: motores de traccion.
            hub: PrimeHub, de donde salen el IMU, la bateria y el parlante.
            velocidad_base: velocidad en mm/s de los movimientos que no piden
                una velocidad explicita.
        """
        self.drive_base = drive_base
        self.motor_izquierda = motor_izquierdo
        self.motor_derecha = motor_derecho
        self.hub = hub
        self.velocidad_base = velocidad_base
        # Ultimo (velocidad, aceleracion) aplicado al drive_base. Sirve para no
        # llamar a settings() en cada movimiento: solo cuando el valor cambia.
        self._ajustes_aplicados = None

    def _aplicar_velocidad(self, velocidad, aceleracion=None):
        """Deja la velocidad pedida cargada en el drive_base.

        straight() no acepta velocidad: la toma de settings(), asi que hay que
        escribirla antes de cada movimiento.

        Argumentos:
            velocidad: mm/s. None usa la velocidad base del chasis. Se acota a
                config.VELOCIDAD_RECTA, que es el techo que acepta el drive_base.
            aceleracion: mm/s². None usa config.ACELERACION_RECTA.
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
        """Micro-freno pasivo de los dos motores, para enlazar sin tiempo muerto."""
        self.motor_izquierda.brake()
        self.motor_derecha.brake()
        wait(4)
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        wait(2)

    def _frenar_motor_encadenado(self, motor):
        """Micro-freno pasivo de un solo motor."""
        motor.brake()
        wait(4)
        motor.stop()

    def avanzar_recto(self, distancia_cm, velocidad=None, aceleracion=None, frenado=Stop.BRAKE, wait_after=True, margen_cm=0, encadenado=False):
        """Avanza en linea recta con la correccion del giroscopio del drive_base.

        Argumentos:
            distancia_cm: distancia a recorrer. Negativa para ir en reversa.
            velocidad: mm/s. None usa la velocidad base del chasis.
            aceleracion: mm/s². None usa config.ACELERACION_RECTA.
            frenado: modo de frenado de Pybricks al llegar a la meta.
            wait_after: True bloquea hasta terminar; False deja el movimiento
                corriendo en segundo plano.
            margen_cm: corta la espera esa cantidad de centimetros antes de la
                meta, para arrancar el movimiento siguiente sin esperar la cola
                de desaceleracion. 0 espera el recorrido completo.
            encadenado: True reemplaza el frenado por el micro-freno pasivo y
                se salta el sonido de confirmacion.
        """
        self._aplicar_velocidad(velocidad, aceleracion)
        distancia_mm = distancia_cm * 10
        
        if encadenado:
            frenado = Stop.NONE
            
        if wait_after and margen_cm > 0:
            distancia_inicial = self.drive_base.distance()
            margen_mm = abs(margen_cm * 10)
            # Si el margen se come toda la distancia, la condicion del while es
            # falsa en la primera vuelta y el movimiento se cancela solo.
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
        """Empuja contra una pared para alinearse y reinicia el rumbo del IMU.

        Es la unica forma de matar el drift acumulado del giroscopio: la pared
        da una referencia fisica exacta.

        Argumentos:
            tiempo_ms: cuanto tiempo se empuja contra la pared.
            potencia: duty-cycle del empuje, 0-100.
            angulo_referencia: rumbo absoluto del mapa que corresponde a esa
                pared. Es el valor que queda cargado en el IMU al terminar.
            reversa: True cuadra de espaldas a la pared, False de frente.
        """
        self.drive_base.stop()
        potencia_aplicada = -potencia if reversa else potencia
        
        self.motor_izquierda.dc(potencia_aplicada)
        self.motor_derecha.dc(potencia_aplicada)
        wait(tiempo_ms) 
        
        self.motor_izquierda.brake()
        self.motor_derecha.brake()
        self.hub.imu.reset_heading(angulo_referencia) 
        
        Utils.emitir_sonido_confirmacion(self.hub)

    def _mover_motor_traccion(self, motor, grados, velocidad, wait_after,
                              frenado, margen_grados, encadenado, nombre):
        """Nucleo de mover_motor_izquierdo() y mover_motor_derecho().

        Argumentos:
            motor: motor de traccion a mover.
            grados: giro del motor, con signo.
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la
                meta. 0 espera el movimiento completo.
            encadenado: True reemplaza el frenado por el micro-freno pasivo.
            nombre: identificador para el mensaje de timeout.
        """
        if encadenado:
            frenado = Stop.NONE

        if wait_after and margen_grados > 0:
            angulo_meta = motor.angle() + grados
            motor.run_angle(velocidad, grados, then=frenado, wait=False)
            reloj_seg = StopWatch()
            while abs(angulo_meta - motor.angle()) > margen_grados:
                if reloj_seg.time() > config.TIMEOUT_MOVIMIENTO_MS:
                    print("TIMEOUT %s" % nombre)
                    break
                if motor.stalled():
                    break
                wait(2)
            if encadenado:
                self._frenar_motor_encadenado(motor)
        else:
            motor.run_angle(velocidad, grados, then=frenado, wait=wait_after)
            if wait_after and encadenado:
                self._frenar_motor_encadenado(motor)

        if not encadenado:
            Utils.emitir_sonido_confirmacion(self.hub)

    def mover_motor_izquierdo(self, grados, velocidad=500, wait_after=True, frenado=Stop.HOLD, margen_grados=0, encadenado=False):
        """Mueve solo el motor izquierdo, dejando el derecho libre.

        Argumentos:
            grados: giro del motor en grados, con signo.
            velocidad: velocidad en grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar a la meta.
            margen_grados: corta la espera esa cantidad de grados antes de la
                meta. 0 espera el recorrido completo.
            encadenado: True reemplaza el frenado por micro-freno pasivo.
        """
        self._mover_motor_traccion(self.motor_izquierda, grados, velocidad,
                                   wait_after, frenado, margen_grados,
                                   encadenado, "mover_motor_izquierdo")

    def mover_motor_derecho(self, grados, velocidad=800, wait_after=True, frenado=Stop.HOLD, margen_grados=0, encadenado=False):
        """Mueve solo el motor derecho, dejando el izquierdo libre.

        Argumentos:
            grados: giro del motor en grados, con signo.
            velocidad: velocidad en grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar a la meta.
            margen_grados: corta la espera esa cantidad de grados antes de la
                meta. 0 espera el recorrido completo.
            encadenado: True reemplaza el frenado por micro-freno pasivo.
        """
        self._mover_motor_traccion(self.motor_derecha, grados, velocidad,
                                   wait_after, frenado, margen_grados,
                                   encadenado, "mover_motor_derecho")

    def sacudir(self, iteraciones=5, potencia=100, tiempo_ms=60):
        """Sacude el chasis en el lugar para acomodar la carga de las garras.

        Argumentos:
            iteraciones: cantidad de sacudidas completas (ida y vuelta).
            potencia: duty-cycle de cada tiron, 0-100.
            tiempo_ms: duracion de cada medio tiron.
        """
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
        """Escala un duty-cycle para que rinda igual con la bateria descargada.

        La referencia son 8 V: con la bateria mas cargada baja el duty y con
        la bateria mas floja lo sube, asi los lazos DC se comportan parejo a
        lo largo de la corrida.

        Argumentos:
            potencia_deseada: duty-cycle pedido, -100 a 100.

        Devuelve el duty-cycle compensado, acotado a -100 a 100.
        """
        voltaje_actual = self.hub.battery.voltage()
        if voltaje_actual == 0: return potencia_deseada
        return max(-100, min(100, potencia_deseada * (8000 / voltaje_actual)))

    def avanzar_y_accionar_en_recorrido(self, distancia_total_cm, distancia_accion_cm, accion_callback, 
                                        frenado=Stop.BRAKE, margen_cm=0, encadenado=False,
                                        accion_secundaria_callback=None, retraso_secundaria_ms=0):
        """Avanza recto disparando una o dos acciones en pleno movimiento.

        Sirve para solapar los mecanismos con el desplazamiento en vez de
        pagar el tiempo de las dos cosas por separado.

        Argumentos:
            distancia_total_cm: distancia a recorrer. Negativa para reversa.
            distancia_accion_cm: distancia recorrida a la que se dispara
                accion_callback.
            accion_callback: funcion sin argumentos, tipicamente un movimiento
                de garra con wait_after=False.
            frenado: modo de frenado de Pybricks al llegar a la meta.
            margen_cm: corta la espera esa cantidad de centimetros antes de la
                meta. 0 espera el recorrido completo.
            encadenado: True reemplaza el frenado por el micro-freno pasivo.
            accion_secundaria_callback: segunda funcion, opcional.
            retraso_secundaria_ms: milisegundos a esperar desde la primera
                accion antes de disparar la segunda. Si el recorrido termina
                antes, se espera lo que falte con el chasis ya detenido.
        """
        distancia_total_mm = distancia_total_cm * 10
        distancia_accion_mm = abs(distancia_accion_cm * 10) 
        margen_mm = abs(margen_cm * 10)

        if encadenado:
            frenado = Stop.NONE

        dist_inicial = self.drive_base.distance()
        meta_mm = abs(distancia_total_mm)
        
        accion_ejecutada = False
        # Sin accion secundaria, la marcamos como ejecutada desde el inicio
        accion_secundaria_ejecutada = False if accion_secundaria_callback else True
        tiempo_accion_1 = 0

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
                tiempo_accion_1 = reloj_seg.time()

            if accion_ejecutada and not accion_secundaria_ejecutada:
                if (reloj_seg.time() - tiempo_accion_1) >= retraso_secundaria_ms:
                    accion_secundaria_callback()
                    accion_secundaria_ejecutada = True

            if dist_actual >= (meta_mm - margen_mm):
                break

            if self.drive_base.stalled():
                break

            wait(2)

        if encadenado:
            self._terminar_movimiento_encadenado()
        else:
            Utils.emitir_sonido_confirmacion(self.hub)
            
        # El recorrido puede terminar antes de que venza el retraso: aca el
        # chasis ya esta quieto, asi que esperar con wait() no cuesta nada.
        if accion_ejecutada and not accion_secundaria_ejecutada:
            tiempo_faltante = retraso_secundaria_ms - (reloj_seg.time() - tiempo_accion_1)
            if tiempo_faltante > 0:
                wait(tiempo_faltante)
            accion_secundaria_callback()
