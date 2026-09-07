from pybricks.tools import StopWatch, wait
from pybricks.parameters import Color
from Utils import Utils
import config

try:
    import math
except ImportError:
    import umath as math


class Navegacion:
    """Giros, seguidores de linea y avances guiados por sensor.

    Todo lo que necesita el IMU o el sensor de color para cerrar un lazo de
    control vive aca; los movimientos a ciegas son de Chasis.
    """

    # Ganancias del lazo PD de giro. Un giro pivote arrastra la rueda que se
    # queda quieta, asi que necesita mas empuje que un giro sobre el eje.
    PERFILES_GIRO = {
        "eje": {"kp": 3.5, "kd": 5.0, "min_potencia": 30},
        "pivote": {"kp": 4.0, "kd": 6.0, "min_potencia": 32},
    }

    # Techos de potencia para giros de poco angulo: con el techo alto el PD no
    # tiene espacio angular para frenar y el robot se pasa de largo.
    TECHO_GIRO_CORTO = 48
    TECHO_GIRO_MEDIO = 68

    def __init__(self, chasis):
        """
        Argumentos:
            chasis: instancia de Chasis, de donde salen el drive_base, los
                motores de traccion y el hub.
        """
        self.chasis = chasis
        
    def detectar_color_preciso(self, sensor):
        color_hsv = sensor.hsv()
        h, s, v = color_hsv.h, color_hsv.s, color_hsv.v
        
        # Colores acromáticos (Blanco, Gris, Negro)
        if s < 35:
            if v > 65: 
                return Color.WHITE
            elif v >= 40: 
                return Color.GRAY
            else: 
                return Color.BLACK
        # Colores cromáticos (Amarillo, Verde, Azul)
        else:
            if h < 95 or h > 310: return Color.YELLOW
            elif h < 185: return Color.GREEN
            else: return Color.BLUE

    def _preparar_giro(self, rueda_pivote):
        """Deja el chasis quieto y trabado antes de leer el rumbo.

        hold() bloquea los motores activamente, a diferencia de stop() que los
        deja rodar por inercia. Los 25 ms alcanzan para que el tren de
        engranajes se asiente y el IMU se lea con el robot realmente quieto.

        Argumentos:
            rueda_pivote: se acepta por simetria con el resto de los giros;
                las dos ruedas se traban igual antes de arrancar.
        """
        self.chasis.drive_base.stop()
        self.chasis.motor_izquierda.hold()
        self.chasis.motor_derecha.hold()
        wait(25)

    def _perfil_giro(self, rueda_pivote, kp, kd, min_potencia):
        """Completa con el perfil las ganancias que el llamador dejo en None.

        Argumentos:
            rueda_pivote: None para el perfil de giro sobre el eje, cualquier
                rueda para el de giro pivote.
            kp, kd, min_potencia: valores pedidos por el llamador; los que
                vengan en None se toman del perfil.

        Devuelve la terna (kp, kd, min_potencia) ya resuelta.
        """
        perfil = self.PERFILES_GIRO["eje" if rueda_pivote is None else "pivote"]
        return (perfil["kp"] if kp is None else kp,
                perfil["kd"] if kd is None else kd,
                perfil["min_potencia"] if min_potencia is None else min_potencia)

    def _rumbo_a_giro(self, rumbo_objetivo, ruta_corta):
        """Traduce un rumbo absoluto del mapa a los grados que hay que girar.

        Argumentos:
            rumbo_objetivo: rumbo absoluto del mapa, en grados del IMU.
            ruta_corta: True toma el camino mas corto (180° como maximo),
                False fuerza la vuelta larga por el otro lado.

        Devuelve la tupla (rumbo_actual, giro_requerido) en grados.
        """
        rumbo_actual = self.chasis.hub.imu.heading()
        giro_corto = (rumbo_objetivo - rumbo_actual + 180) % 360 - 180
        if ruta_corta or giro_corto == 0:
            return rumbo_actual, giro_corto
        return rumbo_actual, (giro_corto - 360 if giro_corto > 0 else giro_corto + 360)

    def _motores_del_giro(self, rueda_pivote):
        """Reparte los motores entre los que empujan el giro y el que se traba.

        El sentido de cada motor es fijo: el izquierdo suma y el derecho resta,
        asi un angulo positivo siempre gira en sentido horario. Un giro pivote
        es el mismo giro con uno de los dos motores apagado.

        Argumentos:
            rueda_pivote: None gira sobre el eje con las dos ruedas;
                "izquierda" o "derecha" nombra la rueda que se queda quieta.

        Devuelve la tupla (activos, rueda_fija), donde activos es una lista de
        pares (motor, sentido) y rueda_fija es el motor a trabar o None.
        """
        if rueda_pivote is None:
            return [(self.chasis.motor_izquierda, 1),
                    (self.chasis.motor_derecha, -1)], None
        if rueda_pivote == "izquierda":
            return [(self.chasis.motor_derecha, -1)], self.chasis.motor_izquierda
        if rueda_pivote == "derecha":
            return [(self.chasis.motor_izquierda, 1)], self.chasis.motor_derecha
        raise ValueError("rueda_pivote tiene que ser None, 'izquierda' o 'derecha'")

    def _lazo_giro_pd(self, meta, giro_requerido, rueda_pivote, max_potencia,
                      min_potencia, kp, kd, tolerancia, encadenado, nombre):
        """Lleva el rumbo del IMU hasta 'meta' con un lazo PD sobre dc() directo.

        Es el nucleo de giro_absoluto() y giro_relativo(); las misiones no lo
        llaman nunca en forma directa. Usa dc() en vez de drive_base.drive()
        para saltear el PID interno de Pybricks y correr a 500 Hz, con
        ganancias que se reescalan segun lo que falte y un filtro EMA en la
        derivada para no amplificar el ruido del IMU.

        Argumentos:
            meta: rumbo absoluto del IMU al que hay que llegar, en grados.
            giro_requerido: grados que faltan al entrar al lazo. Fijan el
                sentido de giro y el techo de potencia.
            rueda_pivote: ver _motores_del_giro().
            max_potencia: techo de duty-cycle, 0-100.
            min_potencia: piso de duty-cycle que vence la friccion estatica.
            kp, kd: ganancias base del PD.
            tolerancia: error en grados que ya se considera llegada.
            encadenado: True frena con el micro-freno pasivo para enlazar el
                movimiento siguiente; False sostiene la posicion con hold().
            nombre: identificador para el mensaje de timeout.
        """
        activos, rueda_fija = self._motores_del_giro(rueda_pivote)

        magnitud = abs(giro_requerido)
        if magnitud < 10:
            techo = min(max_potencia, self.TECHO_GIRO_CORTO)
        elif magnitud < 25:
            techo = min(max_potencia, self.TECHO_GIRO_MEDIO)
        else:
            techo = max_potencia

        sentido = 1 if giro_requerido > 0 else -1
        error_previo = giro_requerido
        derivada_filtrada = 0.0
        lecturas_en_tolerancia = 0

        rumbo_anterior = self.chasis.hub.imu.heading()
        odometria_anterior = self._odometria_giro(activos)
        ciclos_bloqueado = 0
        ciclos = 0

        reloj = StopWatch()
        while True:
            rumbo = self.chasis.hub.imu.heading()
            error = meta - rumbo
            error_abs = abs(error)

            # Se exigen dos lecturas seguidas dentro de la tolerancia: con una
            # sola, un pico de ruido del IMU corta el giro antes de llegar.
            if error_abs <= tolerancia:
                lecturas_en_tolerancia += 1
                if lecturas_en_tolerancia >= 2:
                    break
            else:
                lecturas_en_tolerancia = 0

            # El error cambio de signo: la inercia nos paso de la meta y
            # volver cuesta mas de lo que corrige. El hold() final aguanta.
            if error * sentido < 0:
                break

            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT %s: faltaban %d grados" % (nombre, error))
                break

            # Lejos manda el amortiguamiento, cerca manda la precision.
            if error_abs > 30:
                kp_efectivo, kd_efectivo = kp * 1.1, kd * 1.1
            elif error_abs > 10:
                kp_efectivo, kd_efectivo = kp * 1.3, kd * 0.6
            else:
                kp_efectivo, kd_efectivo = kp * 1.6, kd * 0.25

            derivada_filtrada = (error - error_previo) * 0.7 + derivada_filtrada * 0.3
            correccion = error * kp_efectivo + derivada_filtrada * kd_efectivo

            # Rampa de arranque de 80 ms contra el golpe mecanico del primer
            # instante.
            transcurrido = reloj.time()
            if transcurrido < 80:
                techo_efectivo = min_potencia + (techo - min_potencia) * transcurrido / 80.0
            else:
                techo_efectivo = techo

            potencia = max(-techo_efectivo, min(correccion, techo_efectivo))
            if abs(potencia) < min_potencia and error_abs > tolerancia:
                potencia = min_potencia if error > 0 else -min_potencia

            for motor, sentido_motor in activos:
                motor.dc(self.chasis.compensar_voltaje(potencia * sentido_motor))

            # Anti-patinaje: si el IMU dice que no giramos pero los encoders
            # dicen que si, hay una rueda girando en el aire o contra algo.
            ciclos += 1
            if ciclos % 10 == 0 and error_abs > 5:
                odometria = self._odometria_giro(activos)
                if (abs(rumbo - rumbo_anterior) < 1.0
                        and abs(odometria - odometria_anterior) > 6.0):
                    ciclos_bloqueado += 1
                    if ciclos_bloqueado > 3:
                        # Micro-freno invertido para recuperar traccion.
                        for motor, sentido_motor in activos:
                            motor.dc(self.chasis.compensar_voltaje(-18 * sentido * sentido_motor))
                        wait(15)
                        ciclos_bloqueado = 0
                else:
                    ciclos_bloqueado = max(0, ciclos_bloqueado - 1)

                rumbo_anterior = rumbo
                odometria_anterior = odometria
                if rueda_fija is not None:
                    rueda_fija.hold()

            error_previo = error
            wait(2)

        if encadenado:
            if rueda_fija is None:
                self.chasis._terminar_movimiento_encadenado()
            else:
                self.chasis._frenar_motor_encadenado(activos[0][0])
        else:
            for motor, _ in activos:
                motor.hold()
            if rueda_fija is not None:
                rueda_fija.hold()
            wait(20)
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def _odometria_giro(self, activos):
        """Promedio del angulo de los motores que empujan el giro, en grados.

        Argumentos:
            activos: lista de pares (motor, sentido) como la que devuelve
                _motores_del_giro().
        """
        total = 0
        for motor, _ in activos:
            total += motor.angle()
        return total / len(activos)

    def giro_absoluto(self, rumbo_objetivo, rueda_pivote=None, max_potencia=85,
                      min_potencia=None, kp=None, kd=None, tolerancia=1.5,
                      ruta_corta=True, encadenado=False):
        """Gira hasta un rumbo fijo del mapa, venga de donde venga el robot.

        Absorbe el derrape y el desfase que dejo el movimiento anterior, asi
        que el error no se acumula de un giro al siguiente. Necesita que el
        IMU este alineado con el mapa: eso lo da cuadrar_contra_pared().

        Argumentos:
            rumbo_objetivo: rumbo absoluto del mapa al que hay que quedar
                apuntando, en grados del IMU.
            rueda_pivote: None gira sobre el eje con las dos ruedas;
                "izquierda" o "derecha" nombra la rueda que se queda quieta.
            max_potencia: techo de duty-cycle, 0-100.
            min_potencia: piso de duty-cycle. None usa el del perfil.
            kp, kd: ganancias del PD. None usa las del perfil.
            tolerancia: error en grados que ya se considera llegada.
            ruta_corta: True va por el camino mas corto, False por la vuelta
                larga.
            encadenado: True frena con el micro-freno pasivo para enlazar el
                movimiento siguiente.
        """
        self._preparar_giro(rueda_pivote)
        rumbo_actual, giro_requerido = self._rumbo_a_giro(rumbo_objetivo, ruta_corta)
        if abs(giro_requerido) < config.BANDA_MUERTA_GIRO:
            return
        kp, kd, min_potencia = self._perfil_giro(rueda_pivote, kp, kd, min_potencia)
        self._lazo_giro_pd(rumbo_actual + giro_requerido, giro_requerido,
                           rueda_pivote, max_potencia, min_potencia, kp, kd,
                           tolerancia, encadenado, "giro_absoluto")

    def giro_relativo(self, angulo, rueda_pivote=None, max_potencia=85,
                      min_potencia=None, kp=None, kd=None, tolerancia=1.5,
                      encadenado=False):
        """Gira una cantidad de grados desde donde el robot este apuntando.

        No le importa como quedo el IMU respecto del mapa, pero el error de
        cada giro se hereda: dos giros relativos seguidos acumulan el desfase
        del primero. Para tramos largos conviene giro_absoluto().

        Argumentos:
            angulo: grados a girar. Positivo es horario (a la derecha),
                negativo antihorario (a la izquierda).
            rueda_pivote: None gira sobre el eje con las dos ruedas;
                "izquierda" o "derecha" nombra la rueda que se queda quieta.
            max_potencia: techo de duty-cycle, 0-100.
            min_potencia: piso de duty-cycle. None usa el del perfil.
            kp, kd: ganancias del PD. None usa las del perfil.
            tolerancia: error en grados que ya se considera llegada.
            encadenado: True frena con el micro-freno pasivo para enlazar el
                movimiento siguiente.
        """
        if abs(angulo) < config.BANDA_MUERTA_GIRO:
            return
        self._preparar_giro(rueda_pivote)
        kp, kd, min_potencia = self._perfil_giro(rueda_pivote, kp, kd, min_potencia)
        meta = self.chasis.hub.imu.heading() + angulo
        self._lazo_giro_pd(meta, angulo, rueda_pivote, max_potencia,
                           min_potencia, kp, kd, tolerancia, encadenado,
                           "giro_relativo")

    def desplazar_lateral(self, distancia_cm, max_potencia=85, min_potencia=None,
                          kp=None, kd=None, tolerancia=1.5,
                          compensar_avance=False, reversa=False, encadenado=False):
        """Corre el robot de costado dejandolo en la orientacion original.

        Encadena dos giros pivote opuestos: el primero saca al robot de su
        linea y el segundo lo devuelve al rumbo inicial, ya desplazado. Como
        los dos giros son arcos, el robot tambien avanza de paso; ese avance
        es delta_y y se puede cancelar con compensar_avance.

        Con L = ancho de via y d = desplazamiento pedido:
            alpha = arccos(1 - d/L)     angulo de cada giro pivote
            delta_y = L * sin(alpha)    avance longitudinal que sobra

        Argumentos:
            distancia_cm: desplazamiento lateral. Negativo a la izquierda,
                positivo a la derecha. El limite fisico son dos anchos de via;
                mas que eso se acota y se avisa por consola.
            max_potencia: techo de duty-cycle de los dos giros, 0-100.
            min_potencia: piso de duty-cycle. None usa el del perfil pivote.
            kp, kd: ganancias del PD. None usa las del perfil pivote.
            tolerancia: error en grados que ya se considera llegada.
            compensar_avance: True agrega una recta que cancela delta_y y deja
                al robot sobre su linea original.
            reversa: False pivota avanzando, True pivota retrocediendo.
            encadenado: True frena con el micro-freno pasivo al terminar.

        Devuelve (alpha_grados, delta_y_cm), utiles para calibrar en pista.
        """
        if distancia_cm == 0:
            return 0.0, 0.0

        ancho_via_cm = config.SEPARACION_RUEDAS / 10.0
        d = abs(distancia_cm)

        if d >= (2.0 * ancho_via_cm):
            d_max = 2.0 * ancho_via_cm * 0.99
            print("AVISO desplazar_lateral: distancia %.1f cm excede el limite fisico (%.1f cm), acotando a %.1f cm."
                  % (d, 2.0 * ancho_via_cm, d_max))
            d = d_max

        cos_alpha = max(-1.0, min(1.0, 1.0 - (d / ancho_via_cm)))
        alpha_rad = math.acos(cos_alpha)
        alpha_deg = alpha_rad * 180.0 / math.pi

        if alpha_deg < config.BANDA_MUERTA_GIRO:
            return 0.0, 0.0

        delta_y_cm = ancho_via_cm * math.sin(alpha_rad)
        rumbo_inicial = self.chasis.hub.imu.heading()

        hacia_izquierda = distancia_cm < 0
        angulo = -alpha_deg if hacia_izquierda else alpha_deg

        # Yendo de frente el robot sale de su linea pivotando sobre la rueda
        # del lado hacia el que se desplaza; en reversa, sobre la contraria.
        if hacia_izquierda != reversa:
            pivote_salida, pivote_regreso = "izquierda", "derecha"
        else:
            pivote_salida, pivote_regreso = "derecha", "izquierda"

        self.giro_relativo(angulo, rueda_pivote=pivote_salida,
                           max_potencia=max_potencia, min_potencia=min_potencia,
                           kp=kp, kd=kd, tolerancia=tolerancia,
                           encadenado=True)
        self.giro_absoluto(rumbo_inicial, rueda_pivote=pivote_regreso,
                           max_potencia=max_potencia, min_potencia=min_potencia,
                           kp=kp, kd=kd, tolerancia=tolerancia,
                           encadenado=(encadenado and not compensar_avance))

        if compensar_avance:
            avance_compensar = -delta_y_cm if not reversa else delta_y_cm
            self.chasis.avanzar_recto(avance_compensar, encadenado=encadenado)

        return alpha_deg, delta_y_cm

    def seguidor_linea_color(self, sensor_color, velocidad_max, color_objetivo, lado="derecha", tiempo_acomodo_ms=800, distancia_cm=None, lecturas_confirmacion=3, distancia_maxima_cm=None, encadenado=False):
        """
        Sigue la linea hasta ver un color.

        distancia_cm es la distancia ESPERADA hasta el color: el seguidor va
        frenando a medida que se acerca a ese punto, para llegar despacio y
        leer el color con precision. No corta el recorrido.

        distancia_maxima_cm si corta: es el tope duro de seguridad. Si el color
        no aparece antes, el metodo se detiene igual y avisa por consola.
        """
        cronometro = StopWatch()
        velocidad_max = min(velocidad_max, 70) if distancia_cm is None else velocidad_max
        last_error, contador_color = 0, 0
        multiplicador_lado = 1 if lado == "derecha" else -1
        
        grados_maximos = ((distancia_maxima_cm / (3.1416 * 5.6)) * 360
                          if distancia_maxima_cm is not None else None)
        grados_objetivo = None
        if distancia_cm is not None:
            grados_objetivo = (distancia_cm / (3.1416 * 5.6)) * 360
            velocidad_enfoque = min(50, velocidad_max)
            self.chasis.motor_izquierda.reset_angle(0)
            self.chasis.motor_derecha.reset_angle(0)
            
        cronometro.reset()
        cronometro.resume()
        
        while True:
            if cronometro.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT seguidor_linea_color")
                break
            if self.detectar_color_preciso(sensor_color) == color_objetivo:
                contador_color += 1
                if contador_color >= lecturas_confirmacion: break 
            else:
                contador_color = 0 
                
            velocidad_actual = 25 if cronometro.time() < tiempo_acomodo_ms else velocidad_max

            if grados_objetivo is not None:
                recorrido = (abs(self.chasis.motor_izquierda.angle())
                             + abs(self.chasis.motor_derecha.angle())) / 2

                # Tope duro: el color no aparecio donde tenia que aparecer.
                if grados_maximos is not None and recorrido >= grados_maximos:
                    print("AVISO seguidor_linea_color: no aparecio el color en %d cm" % distancia_maxima_cm)
                    break

                # Rampa: llegar despacio al punto esperado mejora la lectura del color.
                progreso = min(1.0, recorrido / grados_objetivo)
                velocidad_actual = min(velocidad_actual,
                                       max(velocidad_enfoque,
                                           velocidad_max - (velocidad_max - velocidad_enfoque) * progreso))
                
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
    
    def avanzar_manteniendo_rumbo(self, distancia_cm, velocidad=800, angulo_objetivo=None, kp=2.5, kd=10.0, margen_cm=0, encadenado=False):
        """
        Avanza una distancia manteniendo un rumbo fijo usando un Controlador PD.
        Si se le pasa un 'angulo_objetivo', corregirá su trayectoria hacia ese ángulo absoluto del mapa mientras avanza.
        Si no se le pasa, mantendrá exactamente el rumbo actual.
        """
        if angulo_objetivo is None:
            angulo_meta = self.chasis.hub.imu.heading()
        else:
            angulo_meta = angulo_objetivo

        dist_inicial = self.chasis.drive_base.distance()
        distancia_mm_objetivo = abs(distancia_cm * 10)
        margen_mm = abs(margen_cm * 10)
        
        # Ajustamos el signo de la velocidad (soporta ir en reversa si distancia_cm es negativo)
        velocidad_real = abs(velocidad) if distancia_cm > 0 else -abs(velocidad)
        
        error_previo = 0
        
        reloj_seg = StopWatch()
        while True:
            if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT avanzar_manteniendo_rumbo")
                break
            distancia_actual = abs(self.chasis.drive_base.distance() - dist_inicial)
            if distancia_actual >= max(1, distancia_mm_objetivo - margen_mm):
                break
                
            # Calculamos el error continuo (soporta cruces de 360 grados a negativos)
            error_bruto = angulo_meta - self.chasis.hub.imu.heading()
            error = (error_bruto + 180) % 360 - 180
            
            derivada = error - error_previo
            turn_rate = (error * kp) + (derivada * kd)
            
            # Tope de seguridad: limitamos el giro máximo para evitar que el robot 
            # sacrifique demasiado el avance lineal intentando girar de golpe
            turn_rate = min(max(turn_rate, -200), 200) 
            
            self.chasis.drive_base.drive(velocidad_real, turn_rate)
            
            error_previo = error
            wait(10)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)
    
    def avanzar_tiempo_luego_color(self, sensor_color, tiempo_ciego_s, color_objetivo, distancia_extra_cm=0, velocidad_alta=950, velocidad_escaneo=150, lecturas_confirmacion=2, encadenado=False):
        """
        Avanza a máxima velocidad durante un tiempo ciego (ignorando derrapes), 
        luego reduce la velocidad abruptamente, avanza hasta detectar un color específico
        y opcionalmente continúa una distancia extra en centímetros antes de finalizar.
        """
        from pybricks.tools import StopWatch, wait
        from Utils import Utils
        
        cronometro = StopWatch()
        contador_color = 0
        
        cronometro.reset()
        cronometro.resume()
        
        # Al tener use_gyro(True) en el drive_base, Pybricks corregirá automáticamente cualquier giro no deseado.
        self.chasis.drive_base.drive(velocidad_alta, 0)
        
        while cronometro.time() < (tiempo_ciego_s * 1000):
            wait(10)
            
        self.chasis.drive_base.drive(velocidad_escaneo, 0)
        
        reloj_seg = StopWatch()
        while True:
            if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT avanzar_tiempo_luego_color")
                break
            if self.detectar_color_preciso(sensor_color) == color_objetivo:
                contador_color += 1
                # Pedimos confirmaciones consecutivas para evitar falsos positivos
                if contador_color >= lecturas_confirmacion: 
                    break
            else:
                contador_color = 0 
                
            wait(5) # Frecuencia de escaneo
            
        if distancia_extra_cm > 0:
            distancia_inicial = self.chasis.drive_base.distance()
            distancia_mm_objetivo = distancia_extra_cm * 10
            
            # Mantiene el bucle de tracción activo para conservar la corrección del giroscopio
            while abs(self.chasis.drive_base.distance() - distancia_inicial) < distancia_mm_objetivo:
                if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                    print("TIMEOUT avanzar_tiempo_luego_color (distancia extra)")
                    break
                if self.chasis.drive_base.stalled():
                    break
                wait(5)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)  

    def avanzar_distancia_luego_color(self, sensor_color, distancia_ciega_cm, color_objetivo,
                                      distancia_maxima_cm, velocidad_alta=950, velocidad_escaneo=200,
                                      distancia_extra_cm=0, lecturas_confirmacion=2, encadenado=False):
        """
        Avanza a ciegas una distancia y recien despues empieza a buscar un color.

        Diferencia con avanzar_tiempo_luego_color: la fase ciega se mide por
        distancia recorrida, no por tiempo. El tiempo se corre con la bateria;
        la odometria no. Eso permite pasar de largo lineas intermedias sin
        riesgo de falso positivo, y terminar el movimiento sobre una marca
        fisica real en vez de sobre una distancia integrada mas deslizamiento.

        distancia_maxima_cm es el tope de seguridad medido desde el arranque:
        si el color no aparece antes, corta igual y devuelve False.

        Devuelve True si encontro el color, False si corto por tope o timeout.
        """
        if abs(distancia_maxima_cm) <= abs(distancia_ciega_cm):
            raise ValueError("distancia_maxima_cm tiene que ser mayor que distancia_ciega_cm")

        signo = 1 if distancia_ciega_cm >= 0 else -1
        ciega_mm = abs(distancia_ciega_cm) * 10
        maxima_mm = abs(distancia_maxima_cm) * 10
        extra_mm = abs(distancia_extra_cm) * 10

        inicio = self.chasis.drive_base.distance()
        reloj_seg = StopWatch()
        contador_color = 0
        encontrado = False

        # Fase 1: a fondo y sin mirar el sensor. Ninguna linea intermedia puede
        # confundir al robot porque directamente no se esta leyendo el color.
        self.chasis.drive_base.drive(signo * abs(velocidad_alta), 0)
        while abs(self.chasis.drive_base.distance() - inicio) < ciega_mm:
            if self.chasis.drive_base.stalled():
                break
            if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT avanzar_distancia_luego_color (fase ciega)")
                break
            wait(5)

        # Fase 2: busqueda del color, mas lento y con tope de seguridad.
        self.chasis.drive_base.drive(signo * abs(velocidad_escaneo), 0)
        while abs(self.chasis.drive_base.distance() - inicio) < maxima_mm:
            if self.detectar_color_preciso(sensor_color) == color_objetivo:
                contador_color += 1
                if contador_color >= lecturas_confirmacion:
                    encontrado = True
                    recorrido_cm = abs(self.chasis.drive_base.distance() - inicio) / 10.0
                    print("%s encontrado a %.1f cm (ventana %d-%d)"
                          % (color_objetivo, recorrido_cm, distancia_ciega_cm, distancia_maxima_cm))
                    break
            else:
                contador_color = 0

            if self.chasis.drive_base.stalled():
                break
            if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT avanzar_distancia_luego_color (busqueda)")
                break
            # 10 ms y no 5: leer mas rapido que el refresco del sensor devuelve
            # la misma muestra dos veces y las confirmaciones dejan de filtrar.
            wait(10)

        if not encontrado:
            print("AVISO: %s no aparecio entre %d y %d cm. Corregi la ventana."
                  % (color_objetivo, distancia_ciega_cm, distancia_maxima_cm))

        # Fase 3: distancia extra, medida desde donde aparecio el color.
        if extra_mm > 0:
            marca = self.chasis.drive_base.distance()
            while abs(self.chasis.drive_base.distance() - marca) < extra_mm:
                if self.chasis.drive_base.stalled():
                    break
                if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                    break
                wait(5)

        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            # brake() y no stop(): este metodo existe para posicionar con
            # precision, y coastear desde la velocidad de escaneo agrega
            # un centimetro de deriva que no hace falta regalar.
            self.chasis.drive_base.brake()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

        return encontrado

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
            if cronometro.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT seguidor_linea_cruces")
                break
            t = cronometro.time()
            velocidad_actual = 25 if t < tiempo_acomodo_ms else velocidad_max

            reflexion_actual = sensor_color.reflection()
            error = reflexion_actual - 35
            
            if reflexion_actual <= umbral_negro and not en_cruce:
                cruces_detectados += 1
                en_cruce = True
                print(f"Cruce {cruces_detectados}/{cruces_objetivo} detectado")
            elif reflexion_actual >= umbral_salida and en_cruce:
                en_cruce = False

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

    def seguidor_linea_cruces_y_distancia(self, sensor_color, velocidad_max, cruces_objetivo, distancia_extra_cm, distancia_inicial_cm=0, lado="derecha", tiempo_acomodo_ms=800, kp=0.85, kd=2.5, k_freno=0.6, margen_cm=0, encadenado=False):
        """
        Combina un avance inicial ciego a cruces, la detección de cruces y un avance extra por distancia 
        en un solo movimiento fluido. Mantiene el mismo lazo PID para evitar derrapes.
        """
        from pybricks.tools import StopWatch, wait
        from Utils import Utils
        
        cronometro = StopWatch()
        last_error = 0
        multiplicador_lado = 1 if lado == "derecha" else -1
        
        cruces_detectados = 0
        en_cruce = False
        umbral_negro = 15 
        umbral_salida = 25 
        
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
            if cronometro.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT seguidor_linea_cruces_y_distancia")
                break
            # Medida global de grados para las distancias (inicial y extra)
            grados_recorridos_totales = (abs(self.chasis.motor_izquierda.angle()) + abs(self.chasis.motor_derecha.angle())) / 2

            if en_distancia_inicial:
                if grados_recorridos_totales >= grados_objetivo_inicial:
                    en_distancia_inicial = False # Termina avance inicial, empieza a buscar cruces
            elif not buscando_cruces:
                grados_recorridos_extra = grados_recorridos_totales - grados_inicio_extra
                if grados_recorridos_extra >= grados_objetivo_extra:
                    break
                    
            t = cronometro.time()
            velocidad_actual = 25 if t < tiempo_acomodo_ms else velocidad_max

            reflexion_actual = sensor_color.reflection()
            error = reflexion_actual - 35
            
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

    def avanzar_contando_lineas(self, sensor_color, lineas_objetivo, color_linea, tiempo_ciego_s=0.0, distancia_extra_cm=0.0, velocidad=1000, velocidad_lenta=150, encadenado=False, debug=True):
        """
        Avanza recto ignorando líneas por un tiempo ciego inicial, luego cuenta cuántas 
        veces cruza una línea del color especificado. Al tocar la penúltima línea, 
        desacelera para mayor precisión. Finaliza avanzando una distancia extra opcional.
        
        debug: True para imprimir logs en consola, False para silenciarlos.
        """
        from pybricks.tools import StopWatch, wait
        from Utils import Utils
        
        # Si el objetivo es solo 1 línea, aplicamos la velocidad lenta desde el inicio 
        # para garantizar precisión, de lo contrario arrancamos a máxima velocidad.
        velocidad_actual = velocidad_lenta if lineas_objetivo == 1 else velocidad
        self.chasis.drive_base.drive(velocidad_actual, 0)
        
        if tiempo_ciego_s > 0:
            if debug:
                print(f"--- ETAPA 1: Avance ciego por {tiempo_ciego_s}s a {velocidad_actual} mm/s ---")
            
            cronometro = StopWatch()
            while cronometro.time() < (tiempo_ciego_s * 1000):
                if self.chasis.drive_base.stalled():
                    if debug:
                        print("ALERTA: Robot atascado en tiempo ciego.")
                    break
                wait(10)

        contador_lineas = 0
        en_linea = False
        
        if debug:
            print(f"--- ETAPA 2: Iniciando escaneo (Objetivo: {lineas_objetivo} líneas) ---")
        
        reloj_seg = StopWatch()
        while contador_lineas < lineas_objetivo:
            if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT avanzar_contando_lineas")
                break
            color_actual = self.detectar_color_preciso(sensor_color)
            
            if color_actual == color_linea:
                if not en_linea:
                    # Se detecta el flanco de entrada a la línea
                    en_linea = True
                    contador_lineas += 1
                    if debug:
                        distancia_actual = self.chasis.drive_base.distance()
                        print(f"Línea #{contador_lineas} detectada a los {distancia_actual} mm.")
                    
                    # Verificamos si acabamos de cruzar la penúltima línea
                    if contador_lineas == (lineas_objetivo - 1) and lineas_objetivo > 1:
                        self.chasis.drive_base.drive(velocidad_lenta, 0)
                        if debug:
                            print(f"--- Penúltima línea alcanzada. Reduciendo velocidad a {velocidad_lenta} mm/s ---")
            else:
                if en_linea:
                    # Se detecta el flanco de salida de la línea
                    en_linea = False
                    
            if self.chasis.drive_base.stalled():
                if debug:
                    print("ALERTA: Robot atascado durante el conteo.")
                break
                
            wait(5) # Frecuencia de escaneo para alta velocidad
            
        if debug:
            print(f"--- Conteo finalizado: {contador_lineas} líneas detectadas ---")
            
        if distancia_extra_cm > 0:
            if debug:
                print(f"--- ETAPA 3: Avanzando distancia extra de {distancia_extra_cm} cm ---")
                
            distancia_inicial = self.chasis.drive_base.distance()
            distancia_mm_objetivo = distancia_extra_cm * 10.0
            
            while abs(self.chasis.drive_base.distance() - distancia_inicial) < distancia_mm_objetivo:
                if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                    print("TIMEOUT avanzar_contando_lineas (distancia extra)")
                    break
                if self.chasis.drive_base.stalled():
                    if debug:
                        print("ALERTA: Robot atascado en distancia extra.")
                    break
                wait(5)
                
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)