from pybricks.tools import StopWatch, wait
from pybricks.parameters import Color
from Utils import Utils
import config

try:
    import math
except ImportError:
    try:
        import umath as math
    except ImportError:
        math = None

def _acos_safe(x):
    if math is not None and hasattr(math, 'acos'):
        return math.acos(x)
    x = max(-1.0, min(1.0, float(x)))
    neg = (x < 0)
    x_abs = -x if neg else x
    res = (1.0 - x_abs) ** 0.5 * (1.5707288 - 0.2121144 * x_abs + 0.0742610 * x_abs * x_abs - 0.0187293 * x_abs * x_abs * x_abs)
    return 3.1415926535 - res if neg else res

def _sin_safe(x):
    if math is not None and hasattr(math, 'sin'):
        return math.sin(x)
    PI = 3.1415926535
    TWO_PI = 6.283185307
    x = (x + PI) % TWO_PI - PI
    x2 = x * x
    return x - (x * x2) / 6.0 + (x2 * x2 * x) / 120.0 - (x2 * x2 * x2 * x) / 5040.0


class Navegacion:
    def __init__(self, chasis):
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

    def giro_preciso_pd(self, angulo_relativo, max_speed=800, min_speed=40, kp=4.0, kd=18.0, margen_grados=0, encadenado=False):
        if abs(angulo_relativo) < config.BANDA_MUERTA_GIRO:
            return
        angulo_meta = self.chasis.hub.imu.heading() + angulo_relativo
        error_previo = 0
        reloj = StopWatch()
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            if abs(error) <= max(1, margen_grados): break
            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT giro_preciso_pd (error %d grados)" % error)
                break
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
        if abs(angulo_relativo) < config.BANDA_MUERTA_GIRO:
            return
        angulo_meta = self.chasis.hub.imu.heading() + angulo_relativo
        error_previo = 0
        reloj = StopWatch()
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            if abs(error) <= max(1, margen_grados): break
            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT giro_eje_puro (error %d grados)" % error)
                break
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
        # Por debajo del ruido del IMU no vale la pena moverse: el PD se queda
        # pataleando con un turn_rate que no alcanza a vencer la friccion estatica.
        if abs(giro_requerido) < config.BANDA_MUERTA_GIRO:
            return
        
        # Guardamos la dirección inicial para detectar si la inercia nos hace cruzar la meta
        error_inicial_signo = 1 if giro_requerido > 0 else -1
        
        reloj = StopWatch()
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            
            if abs(error) <= max(1, margen_grados) or (error * error_inicial_signo < 0): 
                break
            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT giro_absoluto_pd: faltaban %d grados" % error)
                break
            
            derivada = error - error_previo
            turn_rate = (error * kp) + (derivada * kd)
            
            # Piso efectivo de velocidad. Antes esto era 0 por debajo de 5 grados: con un
            # error de 2 grados el turn_rate quedaba en ~6 grados/s, que no mueve el robot,
            # la derivada se iba a 0 y el lazo se quedaba trabado hasta que el ruido del
            # IMU lo sacaba de casualidad.
            if abs(error) > 15:
                min_speed_actual = min_speed
            elif abs(error) > 5:
                min_speed_actual = 40
            else:
                min_speed_actual = config.PISO_VELOCIDAD_GIRO
            
            turn_rate = min(max(turn_rate, min_speed_actual), max_speed) if turn_rate > 0 else max(min(turn_rate, -min_speed_actual), -max_speed)
            
            self.chasis.drive_base.drive(0, turn_rate)
            error_previo = error
            wait(10)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def giro_turbo(self, angulo_objetivo, max_potencia=85, min_potencia=30,
                   kp=3.5, kd=5.0, tolerancia=1.5, ruta_corta=True,
                   encadenado=False):
        """
        Giro absoluto hiperoptimizado con control DC directo a 500 Hz.

        Usa dc() en vez de drive_base.drive() para eliminar la latencia del
        PID interno de Pybricks. El lazo PD externo es el unico controlador,
        con ganancias adaptativas y filtro EMA en la derivada.

        Funciona para giros de 3 grados y de 180 grados con la misma precision:
        auto-escala la potencia maxima y las ganancias segun la magnitud.

        Parametros
        ----------
        angulo_objetivo : float
            Angulo absoluto del campo al que se quiere llegar (grados IMU).
        max_potencia : int
            Techo de duty-cycle (0-100). Default 85.
        min_potencia : int
            Piso que vence la friccion estatica. Default 30.
        kp : float
            Ganancia proporcional base. Default 3.5.
        kd : float
            Ganancia derivativa base. Default 5.0.
        tolerancia : float
            Error aceptable para terminar (grados). Default 1.5.
        ruta_corta : bool
            True = camino mas corto, False = vuelta larga.
        encadenado : bool
            True = micro-freno rapido para enlazar movimientos.
        """
        # ═══════════════════════════════════════════════════════════════
        # 1. PRE-ESTABILIZACIÓN
        #    hold() bloquea activamente los motores (PID interno mantiene
        #    posicion). stop() solo corta la señal y deja girar por inercia.
        #    Los 25 ms dejan que el tren de engranajes se asiente.
        # ═══════════════════════════════════════════════════════════════
        self.chasis.drive_base.stop()
        self.chasis.motor_izquierda.hold()
        self.chasis.motor_derecha.hold()
        wait(25)

        # ═══════════════════════════════════════════════════════════════
        # 2. CÁLCULO DE RUTA (shortest-path o vuelta larga)
        # ═══════════════════════════════════════════════════════════════
        angulo_actual = self.chasis.hub.imu.heading()
        error_bruto = angulo_objetivo - angulo_actual
        error_corto = (error_bruto + 180) % 360 - 180

        if ruta_corta:
            giro_req = error_corto
        else:
            giro_req = (error_corto - 360 if error_corto > 0
                        else error_corto + 360 if error_corto < 0
                        else 0)

        meta = angulo_actual + giro_req

        # Banda muerta: debajo del ruido del IMU no vale moverse
        if abs(giro_req) < config.BANDA_MUERTA_GIRO:
            return

        # ═══════════════════════════════════════════════════════════════
        # 3. AUTO-ESCALADO DE POTENCIA PARA GIROS CORTOS
        #    Un giro de 8° con 85 % de duty lo pasa de largo porque el PD
        #    no tiene espacio angular para frenar. Bajamos el techo para
        #    que la rampa de desaceleracion del controlador tenga margen.
        # ═══════════════════════════════════════════════════════════════
        mag = abs(giro_req)
        if mag < 10:
            techo = min(max_potencia, 45)
        elif mag < 25:
            techo = min(max_potencia, 65)
        else:
            techo = max_potencia

        # ═══════════════════════════════════════════════════════════════
        # 4. ESTADO INICIAL DEL CONTROLADOR
        # ═══════════════════════════════════════════════════════════════
        signo = 1 if giro_req > 0 else -1
        err_prev = giro_req          # error conocido en t = 0
        d_filt = 0.0                 # derivada filtrada (EMA)
        estables = 0                 # lecturas consecutivas dentro de tolerancia

        # Anti-patinaje
        imu_prev = angulo_actual
        odom_prev = (self.chasis.motor_izquierda.angle()
                     + self.chasis.motor_derecha.angle()) * 0.5
        bloqueo_cnt = 0
        it = 0

        reloj = StopWatch()

        # ═══════════════════════════════════════════════════════════════
        # 5. LAZO DE CONTROL  ─  500 Hz (wait 2 ms)
        # ═══════════════════════════════════════════════════════════════
        while True:
            imu = self.chasis.hub.imu.heading()
            err = meta - imu
            err_a = abs(err)

            # ── 5a. Salida por tolerancia (2 lecturas estables) ──────
            #    Exigir 2 lecturas evita salir por un pico de ruido del
            #    IMU que casualmente cae dentro de la ventana.
            if err_a <= tolerancia:
                estables += 1
                if estables >= 2:
                    break
            else:
                estables = 0

            # ── 5b. Salida por sobreimpulso ──────────────────────────
            #    Si el signo del error se invierte, la inercia nos llevo
            #    pasados de la meta. Mejor frenar ya que intentar volver
            #    (el hold() final mantiene la posicion).
            if err * signo < 0:
                break

            # ── 5c. Timeout de emergencia ────────────────────────────
            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT giro_turbo: faltaban %d°" % err)
                break

            # ── 5d. Ganancias dinámicas ──────────────────────────────
            #        Lejos  (>30°): crucero con amortiguamiento alto
            #        Medio  (10-30°): desaceleracion controlada
            #        Cerca  (<10°): precision quirurgica, kd minimo
            if err_a > 30:
                kp_e = kp * 1.1
                kd_e = kd * 1.1
            elif err_a > 10:
                kp_e = kp * 1.3
                kd_e = kd * 0.6
            else:
                kp_e = kp * 1.6
                kd_e = kd * 0.25

            # ── 5e. PD con filtro EMA en la derivada ─────────────────
            #    El filtro 70/30 suaviza los picos de ruido del IMU sin
            #    perder reactividad: la señal real pasa, el ruido no.
            d_raw = err - err_prev
            d_filt = d_raw * 0.7 + d_filt * 0.3
            corr = err * kp_e + d_filt * kd_e

            # ── 5f. Rampa de arranque (80 ms) ────────────────────────
            #    Previene el golpe mecanico del primer instante: el techo
            #    sube linealmente desde min_potencia hasta el techo real.
            t = reloj.time()
            t_eff = min_potencia + (techo - min_potencia) * t / 80.0 if t < 80 else techo

            # ── 5g. Saturacion + piso de friccion estatica ───────────
            if corr > t_eff:
                pot = t_eff
            elif corr < -t_eff:
                pot = -t_eff
            else:
                pot = corr

            if abs(pot) < min_potencia and err_a > tolerancia:
                pot = min_potencia if err > 0 else -min_potencia

            # ── 5h. Aplicar DC con compensacion de voltaje ───────────
            #    compensar_voltaje() normaliza a 8 V de referencia para
            #    que el comportamiento sea identico con bateria llena o
            #    medio descargada.
            self.chasis.motor_izquierda.dc(
                self.chasis.compensar_voltaje(pot))
            self.chasis.motor_derecha.dc(
                self.chasis.compensar_voltaje(-pot))

            # ── 5i. Deteccion de bloqueo (cada 10 ciclos, >5° error) ─
            #    Si el IMU dice "no giras" pero los encoders dicen "si
            #    giro", una rueda esta patinando en el aire o contra un
            #    obstaculo. Micro-freno inverso para recuperar traccion.
            it += 1
            if it % 10 == 0 and err_a > 5:
                d_imu = abs(imu - imu_prev)
                odom = (self.chasis.motor_izquierda.angle()
                        + self.chasis.motor_derecha.angle()) * 0.5
                d_odom = abs(odom - odom_prev)

                if d_imu < 1.0 and d_odom > 5.0:
                    bloqueo_cnt += 1
                    if bloqueo_cnt > 3:
                        self.chasis.motor_izquierda.dc(
                            self.chasis.compensar_voltaje(-15 * signo))
                        self.chasis.motor_derecha.dc(
                            self.chasis.compensar_voltaje(15 * signo))
                        wait(15)
                        bloqueo_cnt = 0
                else:
                    bloqueo_cnt = max(0, bloqueo_cnt - 1)

                imu_prev = imu
                odom_prev = odom

            err_prev = err
            wait(2)

        # ═══════════════════════════════════════════════════════════════
        # 6. FRENADO
        #    encadenado: micro-freno pasivo (brake → stop) para enlazar
        #                movimientos sin tiempo muerto.
        #    normal:     hold() mantiene posicion activamente + 20 ms
        #                de asentamiento mecanico.
        # ═══════════════════════════════════════════════════════════════
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.hold()
            self.chasis.motor_derecha.hold()
            wait(20)
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def giro_relativo_turbo(self, angulo, max_potencia=85, min_potencia=30,
                            kp=3.5, kd=5.0, tolerancia=1.5,
                            encadenado=False):
        """
        Giro RELATIVO hiperoptimizado. No necesita heading absoluto ni
        cuadrar_contra_pared: cada giro se mide desde la posicion actual.

        Positivo = horario (derecha), Negativo = antihorario (izquierda).

        Usa dc() directo a 500 Hz con PD adaptativo, filtro EMA, rampa
        de arranque, auto-escalado de potencia y deteccion de bloqueo.
        """
        if angulo == 0 or abs(angulo) < config.BANDA_MUERTA_GIRO:
            return

        # ─── PRE-ESTABILIZACIÓN ───────────────────────────────────────
        self.chasis.drive_base.stop()
        self.chasis.motor_izquierda.hold()
        self.chasis.motor_derecha.hold()
        wait(25)

        # ─── META RELATIVA ────────────────────────────────────────────
        # Leemos el heading DESPUÉS del hold para que la lectura sea
        # con el robot completamente quieto (maxima precision).
        inicio = self.chasis.hub.imu.heading()
        meta = inicio + angulo

        # ─── AUTO-ESCALADO PARA GIROS CORTOS ─────────────────────────
        mag = abs(angulo)
        if mag < 10:
            techo = min(max_potencia, 45)
        elif mag < 25:
            techo = min(max_potencia, 65)
        else:
            techo = max_potencia

        # ─── ESTADO DEL CONTROLADOR ──────────────────────────────────
        signo = 1 if angulo > 0 else -1
        err_prev = angulo
        d_filt = 0.0
        estables = 0

        # Anti-patinaje
        imu_prev = inicio
        odom_prev = (self.chasis.motor_izquierda.angle()
                     + self.chasis.motor_derecha.angle()) * 0.5
        bloqueo_cnt = 0
        it = 0

        reloj = StopWatch()

        # ─── LAZO DE CONTROL ─ 500 Hz ────────────────────────────────
        while True:
            imu = self.chasis.hub.imu.heading()
            err = meta - imu
            err_a = abs(err)

            # Salida por tolerancia (2 lecturas consecutivas)
            if err_a <= tolerancia:
                estables += 1
                if estables >= 2:
                    break
            else:
                estables = 0

            # Salida por sobreimpulso
            if err * signo < 0:
                break

            # Timeout
            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT giro_relativo_turbo: faltaban %d°" % err)
                break

            # Ganancias dinámicas
            if err_a > 30:
                kp_e = kp * 1.1
                kd_e = kd * 1.1
            elif err_a > 10:
                kp_e = kp * 1.3
                kd_e = kd * 0.6
            else:
                kp_e = kp * 1.6
                kd_e = kd * 0.25

            # PD con filtro EMA 70/30
            d_raw = err - err_prev
            d_filt = d_raw * 0.7 + d_filt * 0.3
            corr = err * kp_e + d_filt * kd_e

            # Rampa de arranque (80 ms)
            t = reloj.time()
            t_eff = min_potencia + (techo - min_potencia) * t / 80.0 if t < 80 else techo

            # Saturación + piso de fricción estática
            if corr > t_eff:
                pot = t_eff
            elif corr < -t_eff:
                pot = -t_eff
            else:
                pot = corr

            if abs(pot) < min_potencia and err_a > tolerancia:
                pot = min_potencia if err > 0 else -min_potencia

            # DC directo con compensación de voltaje
            self.chasis.motor_izquierda.dc(
                self.chasis.compensar_voltaje(pot))
            self.chasis.motor_derecha.dc(
                self.chasis.compensar_voltaje(-pot))

            # Detección de bloqueo (cada 10 ciclos, >5° error)
            it += 1
            if it % 10 == 0 and err_a > 5:
                d_imu = abs(imu - imu_prev)
                odom = (self.chasis.motor_izquierda.angle()
                        + self.chasis.motor_derecha.angle()) * 0.5
                d_odom = abs(odom - odom_prev)

                if d_imu < 1.0 and d_odom > 5.0:
                    bloqueo_cnt += 1
                    if bloqueo_cnt > 3:
                        self.chasis.motor_izquierda.dc(
                            self.chasis.compensar_voltaje(-15 * signo))
                        self.chasis.motor_derecha.dc(
                            self.chasis.compensar_voltaje(15 * signo))
                        wait(15)
                        bloqueo_cnt = 0
                else:
                    bloqueo_cnt = max(0, bloqueo_cnt - 1)

                imu_prev = imu
                odom_prev = odom

            err_prev = err
            wait(2)

        # ─── FRENADO ─────────────────────────────────────────────────
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.hold()
            self.chasis.motor_derecha.hold()
            wait(20)
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    # ═══════════════════════════════════════════════════════════════════
    # GIROS PIVOTE CON UN MOTOR (TURBO DC - 500 Hz)
    # ═══════════════════════════════════════════════════════════════════

    def _lazo_giro_un_motor_turbo(self, motor_activo, motor_fijo, sentido_motor,
                                  giro_req, meta, max_potencia=85, min_potencia=32,
                                  kp=4.0, kd=6.0, tolerancia=1.5, encadenado=False,
                                  nombre="giro_un_motor_turbo"):
        """
        Núcleo de ejecución para giros pivote sobre una rueda inmóvil.
        Frecuencia 500 Hz (wait 2ms), control DC directo al motor activo,
        PID adaptativo dinámico, filtro EMA en derivada y compensación de voltaje.
        """
        # Auto-escalado de potencia máxima para giros de poco ángulo
        mag = abs(giro_req)
        if mag < 10:
            techo = min(max_potencia, 48)
        elif mag < 25:
            techo = min(max_potencia, 68)
        else:
            techo = max_potencia

        signo = 1 if giro_req > 0 else -1
        err_prev = giro_req
        d_filt = 0.0
        estables = 0

        # Anti-patinaje y monitoreo
        imu_prev = self.chasis.hub.imu.heading()
        odom_prev = motor_activo.angle()
        bloqueo_cnt = 0
        it = 0

        reloj = StopWatch()

        while True:
            imu = self.chasis.hub.imu.heading()
            err = meta - imu
            err_a = abs(err)

            # 1. Criterio de parada por tolerancia (2 lecturas consecutivas)
            if err_a <= tolerancia:
                estables += 1
                if estables >= 2:
                    break
            else:
                estables = 0

            # 2. Criterio de corte por sobreimpulso inercial
            if err * signo < 0:
                break

            # 3. Timeout de seguridad
            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT %s: faltaban %d grados" % (nombre, err))
                break

            # 4. Ganancias dinámicas según proximidad al objetivo
            if err_a > 30:
                kp_e = kp * 1.1
                kd_e = kd * 1.1
            elif err_a > 10:
                kp_e = kp * 1.3
                kd_e = kd * 0.6
            else:
                kp_e = kp * 1.6
                kd_e = kd * 0.25

            # 5. PD con filtro EMA 70/30 en derivada (elimina ruido IMU)
            d_raw = err - err_prev
            d_filt = d_raw * 0.7 + d_filt * 0.3
            corr = err * kp_e + d_filt * kd_e

            # 6. Rampa de aceleración inicial (80 ms)
            t = reloj.time()
            t_eff = min_potencia + (techo - min_potencia) * t / 80.0 if t < 80 else techo

            # 7. Saturación y piso contra fricción estática
            if corr > t_eff:
                pot = t_eff
            elif corr < -t_eff:
                pot = -t_eff
            else:
                pot = corr

            if abs(pot) < min_potencia and err_a > tolerancia:
                pot = min_potencia if err > 0 else -min_potencia

            # 8. Aplicar DC directo al motor activo con sentido y compensación de batería
            pot_motor = pot * sentido_motor
            motor_activo.dc(self.chasis.compensar_voltaje(pot_motor))

            # 9. Detección de patinaje y reaseguro de rueda fija
            it += 1
            if it % 10 == 0 and err_a > 5:
                d_imu = abs(imu - imu_prev)
                odom_act = motor_activo.angle()
                d_odom = abs(odom_act - odom_prev)

                if d_imu < 1.0 and d_odom > 6.0:
                    bloqueo_cnt += 1
                    if bloqueo_cnt > 3:
                        motor_activo.dc(self.chasis.compensar_voltaje(-18 * signo * sentido_motor))
                        wait(15)
                        bloqueo_cnt = 0
                else:
                    bloqueo_cnt = max(0, bloqueo_cnt - 1)

                imu_prev = imu
                odom_prev = odom_act
                motor_fijo.hold()

            err_prev = err
            wait(2)

        # 10. Frenado
        if encadenado:
            motor_activo.brake()
            wait(4)
            motor_activo.stop()
            wait(2)
        else:
            motor_activo.hold()
            motor_fijo.hold()
            wait(20)
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def giro_relativo_motor_izquierdo_turbo(self, angulo, max_potencia=85, min_potencia=32,
                                            kp=4.0, kd=6.0, tolerancia=1.5,
                                            encadenado=False, desaceleracion=None):
        """
        Giro RELATIVO pivotando sobre la rueda DERECHA inmóvil.
        Mueve únicamente el motor IZQUIERDO.

        Ángulo positivo = gira a la derecha (horario, motor izquierdo hacia adelante).
        Ángulo negativo = gira a la izquierda (antihorario, motor izquierdo hacia atrás).
        """
        if angulo == 0 or abs(angulo) < config.BANDA_MUERTA_GIRO:
            return

        self.chasis.drive_base.stop()
        self.chasis.motor_derecha.hold()
        self.chasis.motor_izquierda.hold()
        wait(25)

        inicio = self.chasis.hub.imu.heading()
        meta = inicio + angulo

        self._lazo_giro_un_motor_turbo(
            motor_activo=self.chasis.motor_izquierda,
            motor_fijo=self.chasis.motor_derecha,
            sentido_motor=1,
            giro_req=angulo,
            meta=meta,
            max_potencia=max_potencia,
            min_potencia=min_potencia,
            kp=kp,
            kd=kd,
            tolerancia=tolerancia,
            encadenado=encadenado,
            nombre="giro_relativo_motor_izquierdo_turbo"
        )

    def giro_absoluto_motor_izquierdo_turbo(self, angulo_objetivo, max_potencia=85, min_potencia=32,
                                            kp=4.0, kd=6.0, tolerancia=1.5, ruta_corta=True,
                                            encadenado=False, desaceleracion=None):
        """
        Giro ABSOLUTO hacia rumbo fijo del mapa pivotando sobre la rueda DERECHA.
        Mueve únicamente el motor IZQUIERDO.
        """
        self.chasis.drive_base.stop()
        self.chasis.motor_derecha.hold()
        self.chasis.motor_izquierda.hold()
        wait(25)

        actual = self.chasis.hub.imu.heading()
        error_bruto = angulo_objetivo - actual
        error_corto = (error_bruto + 180) % 360 - 180

        if ruta_corta:
            giro_req = error_corto
        else:
            giro_req = (error_corto - 360 if error_corto > 0
                        else error_corto + 360 if error_corto < 0
                        else 0)

        if abs(giro_req) < config.BANDA_MUERTA_GIRO:
            return

        meta = actual + giro_req

        self._lazo_giro_un_motor_turbo(
            motor_activo=self.chasis.motor_izquierda,
            motor_fijo=self.chasis.motor_derecha,
            sentido_motor=1,
            giro_req=giro_req,
            meta=meta,
            max_potencia=max_potencia,
            min_potencia=min_potencia,
            kp=kp,
            kd=kd,
            tolerancia=tolerancia,
            encadenado=encadenado,
            nombre="giro_absoluto_motor_izquierdo_turbo"
        )

    def giro_relativo_motor_derecho_turbo(self, angulo, max_potencia=85, min_potencia=32,
                                          kp=4.0, kd=6.0, tolerancia=1.5,
                                          encadenado=False, desaceleracion=None):
        """
        Giro RELATIVO pivotando sobre la rueda IZQUIERDA inmóvil.
        Mueve únicamente el motor DERECHO.

        Ángulo positivo = gira a la derecha (horario, motor derecho hacia atrás).
        Ángulo negativo = gira a la izquierda (antihorario, motor derecho hacia adelante).
        """
        if angulo == 0 or abs(angulo) < config.BANDA_MUERTA_GIRO:
            return

        self.chasis.drive_base.stop()
        self.chasis.motor_izquierda.hold()
        self.chasis.motor_derecha.hold()
        wait(25)

        inicio = self.chasis.hub.imu.heading()
        meta = inicio + angulo

        self._lazo_giro_un_motor_turbo(
            motor_activo=self.chasis.motor_derecha,
            motor_fijo=self.chasis.motor_izquierda,
            sentido_motor=-1,
            giro_req=angulo,
            meta=meta,
            max_potencia=max_potencia,
            min_potencia=min_potencia,
            kp=kp,
            kd=kd,
            tolerancia=tolerancia,
            encadenado=encadenado,
            nombre="giro_relativo_motor_derecho_turbo"
        )

    def giro_absoluto_motor_derecho_turbo(self, angulo_objetivo, max_potencia=85, min_potencia=32,
                                          kp=4.0, kd=6.0, tolerancia=1.5, ruta_corta=True,
                                          encadenado=False, desaceleracion=None):
        """
        Giro ABSOLUTO hacia rumbo fijo del mapa pivotando sobre la rueda IZQUIERDA.
        Mueve únicamente el motor DERECHO.
        """
        self.chasis.drive_base.stop()
        self.chasis.motor_izquierda.hold()
        self.chasis.motor_derecha.hold()
        wait(25)

        actual = self.chasis.hub.imu.heading()
        error_bruto = angulo_objetivo - actual
        error_corto = (error_bruto + 180) % 360 - 180

        if ruta_corta:
            giro_req = error_corto
        else:
            giro_req = (error_corto - 360 if error_corto > 0
                        else error_corto + 360 if error_corto < 0
                        else 0)

        if abs(giro_req) < config.BANDA_MUERTA_GIRO:
            return

        meta = actual + giro_req

        self._lazo_giro_un_motor_turbo(
            motor_activo=self.chasis.motor_derecha,
            motor_fijo=self.chasis.motor_izquierda,
            sentido_motor=-1,
            giro_req=giro_req,
            meta=meta,
            max_potencia=max_potencia,
            min_potencia=min_potencia,
            kp=kp,
            kd=kd,
            tolerancia=tolerancia,
            encadenado=encadenado,
            nombre="giro_absoluto_motor_derecho_turbo"
        )

    # Alias directos para conveniencia de sintaxis
    giro_motor_izquierdo_turbo = giro_absoluto_motor_izquierdo_turbo
    giro_motor_derecho_turbo = giro_absoluto_motor_derecho_turbo

    def desplazar_lateral_turbo(self, distancia_cm, max_potencia=85, min_potencia=32,
                                kp=4.0, kd=6.0, tolerancia=1.5,
                                compensar_avance=False, reversa=False, encadenado=False):
        """
        Desplaza el robot lateralmente una cantidad de centímetros a la izquierda o derecha
        utilizando dos giros pivote consecutivos con un solo motor turbo, finalizando en la
        misma orientación original del robot.

        Geometría del desplazamiento:
        - d = abs(distancia_cm)
        - Ancho de vía: L = config.SEPARACION_RUEDAS / 10.0 (cm)
        - Ángulo de cada giro pivote: alpha = arccos(1 - d/L)
        - Desplazamiento longitudinal circular resultante: delta_y = L * sin(alpha)

        Parámetros:
        - distancia_cm (float): Desplazamiento lateral deseado en cm.
                                Negativo (-) para desplazar hacia la IZQUIERDA.
                                Positivo (+) para desplazar hacia la DERECHA.
        - max_potencia (int): Techo de potencia DC para los giros turbo (default 85).
        - min_potencia (int): Piso de potencia DC para vencer fricción estática (default 32).
        - kp, kd (float): Constantes de control PD dinámico para el giro con una sola rueda.
        - tolerancia (float): Tolerancia en grados para la parada con el giroscopio (default 1.5).
        - compensar_avance (bool): Si es True, ejecuta un tramo recto (avanzar_recto) para
                                  cancelar el avance longitudinal de los arcos circulares (delta_y),
                                  dejando al robot exactamente sobre su línea original.
        - reversa (bool): Si es False (default), los giros pivotan avanzando.
                          Si es True, los giros pivotan retrocediendo.
        - encadenado (bool): Si es True, no emite sonido ni frena en seco al concluir.

        Retorna:
        - (alpha_grados, delta_y_cm): Ángulo calculado para cada giro y avance longitudinal.
        """
        if distancia_cm == 0:
            return 0.0, 0.0

        ancho_via_cm = config.SEPARACION_RUEDAS / 10.0
        d = abs(distancia_cm)

        # Límite físico: dos giros pivote pueden desplazar a lo sumo 2 * L
        if d >= (2.0 * ancho_via_cm):
            d_max = 2.0 * ancho_via_cm * 0.99
            print("AVISO desplazar_lateral_turbo: distancia %.1f cm excede límite físico (%.1f cm), acotando a %.1f cm."
                  % (d, 2.0 * ancho_via_cm, d_max))
            d = d_max

        # Cálculo trigonométrico: cos(alpha) = 1 - d/L
        cos_alpha = 1.0 - (d / ancho_via_cm)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        alpha_rad = _acos_safe(cos_alpha)
        alpha_deg = alpha_rad * 180.0 / 3.1415926535

        if alpha_deg < config.BANDA_MUERTA_GIRO:
            return 0.0, 0.0

        delta_y_cm = ancho_via_cm * _sin_safe(alpha_rad)
        rumbo_inicial = self.chasis.hub.imu.heading()

        if distancia_cm < 0:
            # === DESPLAZAMIENTO A LA IZQUIERDA ===
            if not reversa:
                # Giro 1: Motor derecho avanza -> gira robot a la izquierda (-alpha)
                self.giro_relativo_motor_derecho_turbo(
                    -alpha_deg,
                    max_potencia=max_potencia,
                    min_potencia=min_potencia,
                    kp=kp,
                    kd=kd,
                    tolerancia=tolerancia,
                    encadenado=True
                )
                # Giro 2: Motor izquierdo avanza -> gira robot a la derecha hasta volver al rumbo inicial exacto
                self.giro_absoluto_motor_izquierdo_turbo(
                    rumbo_inicial,
                    max_potencia=max_potencia,
                    min_potencia=min_potencia,
                    kp=kp,
                    kd=kd,
                    tolerancia=tolerancia,
                    encadenado=(encadenado and not compensar_avance)
                )
            else:
                # Giro 1: Motor izquierdo retrocede -> gira robot a la izquierda (-alpha)
                self.giro_relativo_motor_izquierdo_turbo(
                    -alpha_deg,
                    max_potencia=max_potencia,
                    min_potencia=min_potencia,
                    kp=kp,
                    kd=kd,
                    tolerancia=tolerancia,
                    encadenado=True
                )
                # Giro 2: Motor derecho retrocede -> gira robot a la derecha hasta rumbo inicial
                self.giro_absoluto_motor_derecho_turbo(
                    rumbo_inicial,
                    max_potencia=max_potencia,
                    min_potencia=min_potencia,
                    kp=kp,
                    kd=kd,
                    tolerancia=tolerancia,
                    encadenado=(encadenado and not compensar_avance)
                )
        else:
            # === DESPLAZAMIENTO A LA DERECHA ===
            if not reversa:
                # Giro 1: Motor izquierdo avanza -> gira robot a la derecha (+alpha)
                self.giro_relativo_motor_izquierdo_turbo(
                    alpha_deg,
                    max_potencia=max_potencia,
                    min_potencia=min_potencia,
                    kp=kp,
                    kd=kd,
                    tolerancia=tolerancia,
                    encadenado=True
                )
                # Giro 2: Motor derecho avanza -> gira robot a la izquierda hasta volver al rumbo inicial exacto
                self.giro_absoluto_motor_derecho_turbo(
                    rumbo_inicial,
                    max_potencia=max_potencia,
                    min_potencia=min_potencia,
                    kp=kp,
                    kd=kd,
                    tolerancia=tolerancia,
                    encadenado=(encadenado and not compensar_avance)
                )
            else:
                # Giro 1: Motor derecho retrocede -> gira robot a la derecha (+alpha)
                self.giro_relativo_motor_derecho_turbo(
                    alpha_deg,
                    max_potencia=max_potencia,
                    min_potencia=min_potencia,
                    kp=kp,
                    kd=kd,
                    tolerancia=tolerancia,
                    encadenado=True
                )
                # Giro 2: Motor izquierdo retrocede -> gira robot a la izquierda hasta rumbo inicial
                self.giro_absoluto_motor_izquierdo_turbo(
                    rumbo_inicial,
                    max_potencia=max_potencia,
                    min_potencia=min_potencia,
                    kp=kp,
                    kd=kd,
                    tolerancia=tolerancia,
                    encadenado=(encadenado and not compensar_avance)
                )

        # Compensación del avance longitudinal si el usuario lo activa
        if compensar_avance:
            avance_compensar = -delta_y_cm if not reversa else delta_y_cm
            self.chasis.avanzar_recto(avance_compensar, encadenado=encadenado)

        return alpha_deg, delta_y_cm

    desplazamiento_lateral_turbo = desplazar_lateral_turbo



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
            if cronometro.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT seguidor_linea_distancia")
                break
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
    
    def giro_absoluto_motor_izquierdo(self, angulo_objetivo, max_speed=800, min_speed=120, kp=4.0, kd=18.0, margen_grados=0, ruta_corta=True, encadenado=False, desaceleracion=None):
        self.chasis.drive_base.stop()
        self.chasis.motor_derecha.hold() 
        
        error_previo = 0
        factor_conversion = 5.71 
        
        angulo_actual_inicial = self.chasis.hub.imu.heading()
        error_bruto_inicial = angulo_objetivo - angulo_actual_inicial
        error_corto_inicial = (error_bruto_inicial + 180) % 360 - 180
        
        if ruta_corta:
            giro_requerido = error_corto_inicial
        else:
            giro_requerido = error_corto_inicial - 360 if error_corto_inicial > 0 else (error_corto_inicial + 360 if error_corto_inicial < 0 else 0)
                
        angulo_meta = angulo_actual_inicial + giro_requerido
        # Por debajo del ruido del IMU no vale la pena moverse: el PD se queda
        # pataleando con un turn_rate que no alcanza a vencer la friccion estatica.
        if abs(giro_requerido) < config.BANDA_MUERTA_GIRO:
            return
        error_inicial_signo = 1 if giro_requerido > 0 else -1
        
        reloj = StopWatch()
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            
            if abs(error) <= max(1, margen_grados) or (error * error_inicial_signo < 0): 
                break
            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT giro_absoluto_motor_izquierdo: faltaban %d grados" % error)
                break
            
            if desaceleracion is None:
                # Control P clasico. Ojo: entre ~31 y ~8 grados de error el termino P
                # vale menos que min_speed, asi que el piso pasa a ser la velocidad real
                # y el giro se aplana. Por eso existe la rama de abajo.
                derivada = error - error_previo
                turn_rate = ((error * kp) + (derivada * kd)) * factor_conversion
                min_speed_actual = min_speed if abs(error) > 8 else (config.PISO_VELOCIDAD_GIRO * factor_conversion)
                velocidad_aplicar = min(max(turn_rate, min_speed_actual), max_speed) if turn_rate > 0 else max(min(turn_rate, -min_speed_actual), -max_speed)
            else:
                # Perfil de desaceleracion constante: omega = raiz(2 * alfa * error).
                # Corre a fondo hasta el ultimo momento y baja en rampa lineal hasta
                # clavar la meta, en vez de la cola exponencial que deja un control P.
                omega = (2.0 * desaceleracion * abs(error)) ** 0.5
                omega = max(omega, config.PISO_VELOCIDAD_GIRO)
                velocidad_aplicar = min(omega * factor_conversion, max_speed)
                if error < 0:
                    velocidad_aplicar = -velocidad_aplicar
                
            self.chasis.motor_izquierda.run(velocidad_aplicar)
            error_previo = error
            wait(10)
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.motor_izquierda.hold()
            Utils.emitir_sonido_confirmacion(self.chasis.hub)

    def giro_absoluto_motor_derecho(self, angulo_objetivo, max_speed=800, min_speed=120, kp=4.0, kd=18.0, margen_grados=0, ruta_corta=True, encadenado=False, desaceleracion=None):
        self.chasis.drive_base.stop()
        self.chasis.motor_izquierda.hold() 
        
        error_previo = 0
        factor_conversion = 5.71 
        
        angulo_actual_inicial = self.chasis.hub.imu.heading()
        error_bruto_inicial = angulo_objetivo - angulo_actual_inicial
        error_corto_inicial = (error_bruto_inicial + 180) % 360 - 180
        
        if ruta_corta:
            giro_requerido = error_corto_inicial
        else:
            giro_requerido = error_corto_inicial - 360 if error_corto_inicial > 0 else (error_corto_inicial + 360 if error_corto_inicial < 0 else 0)
                
        angulo_meta = angulo_actual_inicial + giro_requerido
        # Por debajo del ruido del IMU no vale la pena moverse: el PD se queda
        # pataleando con un turn_rate que no alcanza a vencer la friccion estatica.
        if abs(giro_requerido) < config.BANDA_MUERTA_GIRO:
            return
        error_inicial_signo = 1 if giro_requerido > 0 else -1
        
        reloj = StopWatch()
        while True:
            error = angulo_meta - self.chasis.hub.imu.heading()
            
            # Protección contra sobreimpulso
            if abs(error) <= max(1, margen_grados) or (error * error_inicial_signo < 0): 
                break
            if reloj.time() > config.TIMEOUT_GIRO_MS:
                print("TIMEOUT giro_absoluto_motor_derecho: faltaban %d grados" % error)
                break
            
            if desaceleracion is None:
                # Control P clasico. Ojo: entre ~31 y ~8 grados de error el termino P
                # vale menos que min_speed, asi que el piso pasa a ser la velocidad real
                # y el giro se aplana. Por eso existe la rama de abajo.
                derivada = error - error_previo
                turn_rate = ((error * kp) + (derivada * kd)) * factor_conversion
                min_speed_actual = min_speed if abs(error) > 8 else (config.PISO_VELOCIDAD_GIRO * factor_conversion)
                velocidad_aplicar = min(max(turn_rate, min_speed_actual), max_speed) if turn_rate > 0 else max(min(turn_rate, -min_speed_actual), -max_speed)
            else:
                # Perfil de desaceleracion constante: omega = raiz(2 * alfa * error).
                # Corre a fondo hasta el ultimo momento y baja en rampa lineal hasta
                # clavar la meta, en vez de la cola exponencial que deja un control P.
                omega = (2.0 * desaceleracion * abs(error)) ** 0.5
                omega = max(omega, config.PISO_VELOCIDAD_GIRO)
                velocidad_aplicar = min(omega * factor_conversion, max_speed)
                if error < 0:
                    velocidad_aplicar = -velocidad_aplicar
                
            self.chasis.motor_derecha.run(-velocidad_aplicar)
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

    def curva_coordenada_local(self, x_cm, y_cm, velocidad=600, kp_giro=4.5, tolerancia_cm=1.0, encadenado=False):
        """
        Hace que el robot dibuje una curva fluida hacia el punto relativo (x_cm, y_cm).
        Implementación 100% nativa sin depender del módulo 'math'.
        x_cm: Distancia hacia adelante (positivo) o atrás (negativo).
        y_cm: Desplazamiento lateral (positivo izquierda, negativo derecha).
        """
        PI = 3.14159265
        HALF_PI = 1.57079632
        TWO_PI = 6.28318530

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
        
        reloj_seg = StopWatch()
        while True:
            if reloj_seg.time() > config.TIMEOUT_LAZO_MS:
                print("TIMEOUT curva_coordenada_local")
                break
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
            
        if encadenado:
            self.chasis._terminar_movimiento_encadenado()
        else:
            self.chasis.drive_base.stop()
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