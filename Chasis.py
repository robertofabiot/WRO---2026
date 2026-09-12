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
        # Cache de ajustes para aplicar settings() solo si cambian los valores
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
            # Validacion para advertir si el margen cancela la espera completa
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

    def _mover_motor_traccion(self, motor, grados, velocidad, wait_after, frenado, margen_grados, encadenado, nombre):
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

    def acomodar_estable(self, iteraciones=5, potencia=80, tiempo_ms=60, wait_after=True):
        """Sacude el chasis y lo devuelve exactamente al angulo de partida.

        Es la version fina de sacudir(): en vez de tirones a dc() abiertos,
        cada medio tiron es un run_target() contra un angulo anclado al
        arrancar, asi que el robot termina en el mismo punto y con el mismo
        rumbo con el que entro. Eso es lo que se necesita para asentar los
        bloques ya soltados dentro de la matriz sin correr el robot ni un
        milimetro: un sacudir() con dc() deja deriva acumulada y los bloques
        quedan fuera de la casilla.

        Argumentos:
            iteraciones: cantidad de sacudidas completas (ida y vuelta).
            potencia: gobierna la velocidad del tiron, 0-100. Se traduce a
                grados/s multiplicando por 8.
            tiempo_ms: duracion nominal de cada medio tiron, de la que sale la
                amplitud en grados.
            wait_after: True traba los motores y espera a que el PID del hub
                asiente las fuerzas antes de devolver el control.
        """
        self.drive_base.stop()

        velocidad = int(potencia * 8)
        amplitud = int((velocidad * tiempo_ms) / 1000)

        # Punto de anclaje: todo el movimiento es relativo a estos dos angulos.
        centro_izq = self.motor_izquierda.angle()
        centro_der = self.motor_derecha.angle()

        for _ in range(iteraciones):
            self.motor_izquierda.run_target(velocidad, centro_izq + amplitud, wait=False)
            self.motor_derecha.run_target(velocidad, centro_der - amplitud, wait=True)

            self.motor_izquierda.run_target(velocidad, centro_izq - amplitud, wait=False)
            self.motor_derecha.run_target(velocidad, centro_der + amplitud, wait=True)

        # El regreso al centro va al 40% de la velocidad: a velocidad plena el
        # overshoot del ultimo tramo deja el chasis torcido justo al final.
        velocidad_regreso = int(velocidad * 0.4)
        self.motor_izquierda.run_target(velocidad_regreso, centro_izq, wait=False)
        self.motor_derecha.run_target(velocidad_regreso, centro_der, wait=True)

        if wait_after:
            self.motor_izquierda.hold()
            self.motor_derecha.hold()
            wait(150)

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
        # Si no hay callback secundario, se considera completada
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
            
        # Esperar tiempo restante antes de la accion secundaria si el chasis llego antes
        if accion_ejecutada and not accion_secundaria_ejecutada:
            tiempo_faltante = retraso_secundaria_ms - (reloj_seg.time() - tiempo_accion_1)
            if tiempo_faltante > 0:
                wait(tiempo_faltante)
            accion_secundaria_callback()

    def giro_de_arco(self, radio_cm, angulo_deg, velocidad=None, aceleracion=None,
                     lado="derecha", frenado=Stop.BRAKE, wait_after=True, encadenado=False):
        """Recorre un arco de radio fijo, girando y avanzando a la vez.

        Es la unica primitiva de movimiento que no estaba: sirve para salir de
        una zona sin frenar a girar en el lugar. Se apoya en curve() del
        drive_base, que reparte solo la velocidad entre las dos ruedas y
        respeta el use_gyro(True) del chasis.

        Argumentos:
            radio_cm: radio del circulo que describe el centro del robot.
            angulo_deg: cuanto se recorre sobre ese circulo. Positivo avanza,
                negativo recorre el mismo arco en reversa.
            velocidad: mm/s sobre el arco. None usa la velocidad base.
            aceleracion: mm/s². None usa config.ACELERACION_RECTA.
            lado: "derecha" curva en sentido horario, "izquierda" antihorario.
            frenado: modo de frenado de Pybricks al llegar.
            wait_after: True bloquea hasta terminar el arco.
            encadenado: True reemplaza el frenado por el micro-freno pasivo.
        """
        if angulo_deg == 0 or radio_cm == 0:
            return

        self._aplicar_velocidad(velocidad, aceleracion)

        if encadenado:
            frenado = Stop.NONE

        # curve() toma el radio con signo: positivo curva a la derecha.
        radio_mm = abs(radio_cm) * 10
        if lado == "izquierda":
            radio_mm = -radio_mm
        elif lado != "derecha":
            raise ValueError("lado tiene que ser 'derecha' o 'izquierda'")

        self.drive_base.curve(radio_mm, angulo_deg, then=frenado, wait=wait_after)

        if wait_after and encadenado:
            self._terminar_movimiento_encadenado()

        if not encadenado:
            Utils.emitir_sonido_confirmacion(self.hub)
