"""Módulo de navegación: giros con IMU, seguimiento de línea y detección de colores.

Preserva exactamente las funciones matemáticas, lazos PD, filtros y algoritmos
del equipo original, organizados dentro de una clase de navegación modular.
"""

from pybricks.parameters import Color, Stop
from pybricks.tools import wait, StopWatch
import config
from Utils import Utils


class Navegacion:
    """Controlador de giros IMU, seguidores de línea y navegación por sensor de color."""

    def __init__(self, chasis, sensor_color, torque=None):
        """
        Argumentos:
            chasis: Instancia de Chasis (para acceso a motores, drive_base y hub).
            sensor_color: Sensor de color principal (seguidor).
            torque: Mecanismo de torque para movimientos combinados.
        """
        self.chasis = chasis
        self.seguidor = sensor_color
        self.torque = torque

        # Accesos directos a componentes compartidos
        self.hub = chasis.hub
        self.motor_izquierdo = chasis.motor_izquierdo
        self.motor_derecho = chasis.motor_derecho
        self.drive_base = chasis.drive_base

        self.diametro_rueda = config.DIAMETRO_RUEDA
        self.circunferencia = config.CIRCUNFERENCIA
        self.grados_por_mm = config.GRADOS_POR_MM

    # =========================================================================
    # 1. GIROS SOBRE EL EJE Y DE ARCO (Preservados de navegacion.py)
    # =========================================================================

    def girar(
        self,
        angulo_deg,
        potencia_max=85,
        potencia_min=40,
        kp_base=3.5,
        kd_base=5.0,
        tiempo_curva_s_ms=100,
        tolerancia_fin=1.9,
        perfil="encadenado"
    ):
        """Giro con control de giroscopio y detección automática de bloqueos/patinaje."""
        if angulo_deg == 0:
            return

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(40)

        self.chasis.preparar_movimiento(
            reset_motores=False,
            reset_gyro=True,
            perfil=perfil
        )

        inicio = self.hub.imu.heading()
        objetivo = inicio + angulo_deg

        es_giro_180 = abs(abs(angulo_deg) - 180) < 0.01
        signo_giro_180 = 1 if angulo_deg > 0 else -1

        error_anterior = Utils.error_angular(objetivo, inicio)
        if es_giro_180 and abs(abs(error_anterior) - 180) < 0.5:
            error_anterior = 180 * signo_giro_180

        derivada_anterior = 0
        cronometro = StopWatch()
        cronometro.reset()

        ultimo_angulo_imu = inicio
        ultimo_angulo_motores = (self.motor_izquierdo.angle() + self.motor_derecho.angle()) / 2.0
        contador_sin_cambio = 0
        bloqueo_detectado = False

        while True:
            actual_imu = self.hub.imu.heading()
            error = Utils.error_angular(objetivo, actual_imu)

            if es_giro_180 and abs(abs(error) - 180) < 0.5:
                error = 180 * signo_giro_180

            angulo_motores = (self.motor_izquierdo.angle() + self.motor_derecho.angle()) / 2.0

            # Detección de bloqueo o patinaje
            if abs(error) > 5:
                delta_imu = abs(actual_imu - ultimo_angulo_imu)
                delta_motores = abs(angulo_motores - ultimo_angulo_motores)

                if delta_imu < 1.0 and delta_motores > 5.0:
                    contador_sin_cambio += 1
                    if contador_sin_cambio > 5:
                        bloqueo_detectado = True
                        movimiento_izquierdo = abs(self.motor_izquierdo.angle() - ultimo_angulo_motores)
                        movimiento_derecho = abs(self.motor_derecho.angle() - ultimo_angulo_motores)

                        if movimiento_izquierdo > movimiento_derecho:
                            self.motor_izquierdo.dc(-20)
                            wait(20)
                        else:
                            self.motor_derecho.dc(-20)
                            wait(20)

                        self.chasis.reset_motores()
                        angulo_motores = 0
                        bloqueo_detectado = False
                        contador_sin_cambio = 0
                else:
                    contador_sin_cambio = max(0, contador_sin_cambio - 1)

                ultimo_angulo_imu = actual_imu
                ultimo_angulo_motores = angulo_motores

            # Control normal del giro
            if abs(error) <= tolerancia_fin:
                break

            if abs(error) > 25:
                kp_dinamico = kp_base * 1.1
                kd_dinamico = kd_base * 1.1
            else:
                kp_dinamico = kp_base * 1.5
                kd_dinamico = kd_base * 0.3

            derivada_cruda = error - error_anterior
            derivada = derivada_cruda * 0.7 + derivada_anterior * 0.3

            correccion = error * kp_dinamico + derivada * kd_base
            tiempo_transcurrido = cronometro.time()

            if tiempo_transcurrido < tiempo_curva_s_ms:
                limite_potencia = potencia_min + (potencia_max - potencia_min) * (tiempo_transcurrido / tiempo_curva_s_ms)
            else:
                limite_potencia = potencia_max

            potencia_final = Utils.limitar(correccion, -limite_potencia, limite_potencia)

            if bloqueo_detectado:
                potencia_final *= 1.5
                potencia_final = Utils.limitar(potencia_final, -limite_potencia, limite_potencia)
                if abs(potencia_final) < potencia_min * 0.5:
                    potencia_final = potencia_min * 0.5 if potencia_final > 0 else -potencia_min * 0.5

            pot_izq = int(potencia_final)
            pot_der = int(-potencia_final)

            self.motor_izquierdo.dc(Utils.limitar(pot_izq, -100, 100))
            self.motor_derecho.dc(Utils.limitar(pot_der, -100, 100))

            error_anterior = error
            derivada_anterior = derivada
            wait(2)

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

    def giro_de_arco(
        self,
        radio_cm,
        angulo_deg,
        potencia_max=80,
        potencia_min=35,
        lado="derecha",
        distancia_ruedas_cm=12.3,
        kp_gyro=2.8,
        tolerancia_fin=1.5,
        perfil="encadenado"
    ):
        """Giro en arco con control de giroscopio para corregir desviaciones."""
        if angulo_deg == 0 or radio_cm == 0:
            return

        self.chasis.preparar_movimiento(reset_motores=False, reset_gyro=True, perfil=perfil)

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

        pi = 3.1416
        radio_interno = abs(radio_cm) - (distancia_ruedas_cm / 2.0)
        radio_externo = abs(radio_cm) + (distancia_ruedas_cm / 2.0)

        if radio_interno <= 0:
            radio_interno = 2.0

        distancia_interna = 2.0 * pi * radio_interno * (abs(angulo_deg) / 360.0)
        distancia_externa = 2.0 * pi * radio_externo * (abs(angulo_deg) / 360.0)
        relacion = distancia_interna / distancia_externa

        potencia_externa = potencia_max
        potencia_interna = potencia_max * relacion

        signo = 1 if angulo_deg > 0 else -1

        if (radio_cm > 0 and lado == "derecha") or (radio_cm < 0 and lado == "izquierda"):
            potencia_izq = potencia_externa * signo
            potencia_der = potencia_interna * signo
        else:
            potencia_izq = potencia_interna * signo
            potencia_der = potencia_externa * signo

        heading_inicial = self.hub.imu.heading()
        heading_objetivo = heading_inicial + angulo_deg

        potencia_interna = max(potencia_interna, potencia_min * 0.3)
        potencia_externa = max(potencia_externa, potencia_min)

        cronometro = StopWatch()
        cronometro.reset()

        while True:
            heading_actual = self.hub.imu.heading()
            error_actual = Utils.error_angular(heading_objetivo, heading_actual)

            if abs(error_actual) <= tolerancia_fin:
                break

            correccion_gyro = Utils.limitar(error_actual * kp_gyro, -25, 25)

            pot_izq_final = potencia_izq + correccion_gyro
            pot_der_final = potencia_der - correccion_gyro

            if abs(error_actual) < 10:
                factor_freno = abs(error_actual) / 10.0
                pot_izq_final *= factor_freno
                pot_der_final *= factor_freno
                pot_izq_final = max(pot_izq_final, potencia_min * signo * 0.5)
                pot_der_final = max(pot_der_final, potencia_min * signo * 0.5)

            pot_izq_final = Utils.limitar(pot_izq_final, -100, 100)
            pot_der_final = Utils.limitar(pot_der_final, -100, 100)

            self.motor_izquierdo.dc(pot_izq_final)
            self.motor_derecho.dc(pot_der_final)
            wait(2)

        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        wait(40)

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

        self.chasis.terminar_movimiento(perfil=perfil, modo="hold", pausa=None)

    def _giro_un_motor(
        self,
        motor_activo,
        motor_fijo,
        angulo_deg,
        sentido_motor,
        velocidad=1000,
        velocidad_min=180,
        zona_freno=22,
        tolerancia=1.2,
        tiempo_max_ms=None,
        perfil="seguro"
    ):
        """Función interna para girar usando una rueda activa mientras la otra queda bloqueada."""
        if angulo_deg == 0:
            return

        self.chasis.preparar_movimiento(reset_motores=False, reset_gyro=False, perfil=perfil)

        inicio = self.hub.imu.heading()
        objetivo = inicio + angulo_deg

        motor_fijo.hold()
        wait(3)

        if tiempo_max_ms is None:
            tiempo_max_ms = max(350, int(abs(angulo_deg) * 18))

        cronometro = StopWatch()
        cronometro.reset()
        lecturas_estables = 0

        while cronometro.time() < tiempo_max_ms:
            actual = self.hub.imu.heading()
            error = Utils.error_angular(objetivo, actual)
            error_abs = abs(error)

            if error_abs <= tolerancia:
                lecturas_estables += 1
                if lecturas_estables >= 2:
                    break
            else:
                lecturas_estables = 0

            if error_abs >= zona_freno:
                velocidad_actual = velocidad
            else:
                proporcion = error_abs / zona_freno
                velocidad_actual = velocidad_min + (velocidad - velocidad_min) * proporcion

            velocidad_actual = int(Utils.limitar(velocidad_actual, velocidad_min, velocidad))
            direccion_error = 1 if error > 0 else -1
            velocidad_motor = velocidad_actual * direccion_error * sentido_motor

            motor_activo.run(velocidad_motor)
            wait(2)

        motor_activo.brake()
        motor_fijo.hold()
        wait(12)

        if perfil == "encadenado":
            motor_activo.stop()
            wait(2)
        else:
            motor_activo.hold()
            motor_fijo.hold()
            wait(15)

    def giro_derecha(
        self,
        angulo_deg,
        velocidad=1000,
        velocidad_min=180,
        zona_freno=22,
        tolerancia=1.2,
        perfil="seguro"
    ):
        """Gira utilizando únicamente la rueda derecha (rueda izquierda fija)."""
        self._giro_un_motor(
            motor_activo=self.motor_derecho,
            motor_fijo=self.motor_izquierdo,
            angulo_deg=angulo_deg,
            sentido_motor=-1,
            velocidad=velocidad,
            velocidad_min=velocidad_min,
            zona_freno=zona_freno,
            tolerancia=tolerancia,
            perfil=perfil
        )

    def giro_izquierda(
        self,
        angulo_deg,
        velocidad=1000,
        velocidad_min=180,
        zona_freno=22,
        tolerancia=1.2,
        perfil="seguro"
    ):
        """Gira utilizando únicamente la rueda izquierda (rueda derecha fija)."""
        self._giro_un_motor(
            motor_activo=self.motor_izquierdo,
            motor_fijo=self.motor_derecho,
            angulo_deg=angulo_deg,
            sentido_motor=1,
            velocidad=velocidad,
            velocidad_min=velocidad_min,
            zona_freno=zona_freno,
            tolerancia=tolerancia,
            perfil=perfil
        )

    def girar_corto(
        self,
        angulo_deg,
        potencia_max=45,
        potencia_min=24,
        kp=4.0,
        tolerancia=1.8,
        tiempo_max_ms=500,
        perfil="encadenado"
    ):
        """Realiza giros pequeños y rápidos sin bloquearse."""
        if angulo_deg == 0:
            return

        self.hub.imu.reset_heading(0)
        wait(10)

        objetivo = angulo_deg
        cronometro = StopWatch()
        cronometro.reset()
        lecturas_estables = 0

        while cronometro.time() < tiempo_max_ms:
            angulo_actual = self.hub.imu.heading()
            error = Utils.error_angular(objetivo, angulo_actual)

            if abs(error) <= tolerancia:
                lecturas_estables += 1
                if lecturas_estables >= 2:
                    break
            else:
                lecturas_estables = 0

            potencia = Utils.limitar(error * kp, -potencia_max, potencia_max)
            if abs(potencia) < potencia_min:
                potencia = potencia_min if error > 0 else -potencia_min

            self.motor_izquierdo.dc(potencia)
            self.motor_derecho.dc(-potencia)
            wait(2)

        self.motor_izquierdo.brake()
        self.motor_derecho.brake()

        if perfil == "encadenado":
            wait(5)
        else:
            self.motor_izquierdo.hold()
            self.motor_derecho.hold()
            wait(20)

    def establecer_norte(self, rumbo_inicial=0):
        """Establece la orientación actual como el rumbo absoluto 0°."""
        self.drive_base.stop()
        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(100)
        self.hub.imu.reset_heading(rumbo_inicial)
        wait(50)

    def girar_a_rumbo(
        self,
        rumbo_objetivo,
        potencia_max=100,
        potencia_media=55,
        potencia_min=45,
        kp=2.6,
        kd=4.2,
        zona_media=35,
        zona_precisa=9,
        tolerancia=1.0,
        ciclos_estables=5,
        tiempo_maximo_ms=3000,
        perfil="encadenado"
    ):
        """Gira hacia un rumbo absoluto en coordenadas de mapa usando el IMU."""
        potencia_max = abs(potencia_max)
        potencia_media = abs(potencia_media)
        potencia_min = abs(potencia_min)

        self.chasis.preparar_movimiento(reset_motores=False, reset_gyro=False, perfil=perfil)

        self.drive_base.stop()
        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

        actual = self.hub.imu.heading()
        error_anterior = Utils.error_angular_absoluto(rumbo_objetivo, actual)
        derivada_filtrada_anterior = 0
        contador_estable = 0

        cronometro = StopWatch()
        cronometro.reset()

        while True:
            actual = self.hub.imu.heading()
            error = Utils.error_angular_absoluto(rumbo_objetivo, actual)

            if abs(error) <= tolerancia:
                contador_estable += 1
                self.motor_izquierdo.brake()
                self.motor_derecho.brake()

                if contador_estable >= ciclos_estables:
                    break
                wait(4)
                continue
            else:
                contador_estable = 0

            if cronometro.time() >= tiempo_maximo_ms:
                break

            derivada_cruda = error - error_anterior
            derivada_filtrada = derivada_cruda * 0.7 + derivada_filtrada_anterior * 0.3
            correccion = error * kp + derivada_filtrada * kd
            distancia_al_objetivo = abs(error)

            if distancia_al_objetivo > zona_media:
                limite_potencia = potencia_max
            elif distancia_al_objetivo > zona_precisa:
                proporcion = (distancia_al_objetivo - zona_precisa) / (zona_media - zona_precisa)
                limite_potencia = potencia_media + (potencia_max - potencia_media) * proporcion
            else:
                proporcion = distancia_al_objetivo / zona_precisa
                limite_potencia = potencia_min + (potencia_media - potencia_min) * proporcion

            potencia_final = Utils.limitar(correccion, -limite_potencia, limite_potencia)
            if abs(potencia_final) < potencia_min:
                potencia_final = potencia_min if error > 0 else -potencia_min

            pot_izq = int(potencia_final)
            pot_der = int(-potencia_final)

            self.motor_izquierdo.dc(Utils.limitar(pot_izq, -100, 100))
            self.motor_derecho.dc(Utils.limitar(pot_der, -100, 100))

            error_anterior = error
            derivada_filtrada_anterior = derivada_filtrada
            wait(4)

        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        wait(30)

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

    # =========================================================================
    # 2. SEGUIMIENTO DE LÍNEA (Preservado de seguimiento_linea.py)
    # =========================================================================

    def seguir_linea(
        self,
        sensor_color=None,
        velocidad_max=100,
        distancia_cm=70,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=120,
        kp=1.15,
        kd=2.6,
        k_freno=0.15,
        objetivo_reflexion=27,
        correccion_max=100,
        margen_cm=0,
        perfil_salida="encadenado",
        captura_inicial=True,
        tiempo_captura_ms=260,
        potencia_captura=55,
        kp_captura=2.4,
        margen_captura=5,
        lecturas_estables_captura=2
    ):
        """Sigue el borde de una línea durante una distancia determinada."""
        if sensor_color is None:
            sensor_color = self.seguidor

        diametro_rueda_cm = self.diametro_rueda / 10.0
        circunferencia_cm = 3.14159 * diametro_rueda_cm
        grados_objetivo = (distancia_cm / circunferencia_cm) * 360.0
        grados_margen = (margen_cm / circunferencia_cm) * 360.0 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)
        multiplicador_lado = 1 if lado == "derecha" else -1

        self.chasis.reset_motores()

        # Fase 1: Captura inicial
        if captura_inicial:
            reloj_captura = StopWatch()
            reloj_captura.reset()
            estables = 0

            while reloj_captura.time() < tiempo_captura_ms:
                lectura = sensor_color.reflection()
                error = lectura - objetivo_reflexion

                if abs(error) <= margen_captura:
                    estables += 1
                    if estables >= lecturas_estables_captura:
                        break
                else:
                    estables = 0

                correccion = Utils.limitar(error * kp_captura * multiplicador_lado, -correccion_max, correccion_max)
                velocidad_base = 28 if abs(error) > 22 else potencia_captura

                potencia_izq = velocidad_base - correccion
                potencia_der = velocidad_base + correccion

                self.motor_izquierdo.dc(Utils.limitar(potencia_izq, -100, 100))
                self.motor_derecho.dc(Utils.limitar(potencia_der, -100, 100))
                wait(2)

            self.motor_izquierdo.brake()
            self.motor_derecho.brake()
            wait(8)
            self.chasis.reset_motores()

        # Fase 2: Seguimiento por distancia
        cronometro = StopWatch()
        cronometro.reset()

        velocidad_minima = 75
        error_anterior = 0
        derivada_anterior = 0

        while True:
            grados_actuales = self.chasis.distancia_promedio_grados()
            if grados_actuales >= grados_objetivo_real:
                break

            tiempo_actual = cronometro.time()
            if tiempo_actual < tiempo_acomodo_ms:
                velocidad_actual = velocidad_minima
            elif tiempo_actual < tiempo_acomodo_ms + tiempo_aceleracion_ms:
                progreso = (tiempo_actual - tiempo_acomodo_ms) / float(tiempo_aceleracion_ms)
                velocidad_actual = velocidad_minima + (velocidad_max - velocidad_minima) * progreso
            else:
                velocidad_actual = velocidad_max

            lectura = sensor_color.reflection()
            error = lectura - objetivo_reflexion
            derivada = (error - error_anterior) * 0.82 + derivada_anterior * 0.18
            correccion = Utils.limitar(((error * kp) + (derivada * kd)) * multiplicador_lado, -correccion_max, correccion_max)

            velocidad_base = max(55, velocidad_actual - abs(error) * k_freno)
            potencia_izq = velocidad_base - correccion
            potencia_der = velocidad_base + correccion

            self.motor_izquierdo.dc(Utils.limitar(potencia_izq, -100, 100))
            self.motor_derecho.dc(Utils.limitar(potencia_der, -100, 100))

            error_anterior = error
            derivada_anterior = derivada
            wait(2)

        self.chasis.terminar_movimiento(perfil=perfil_salida, modo="brake")

    def seguir_linea_hasta_color(
        self,
        color_objetivo,
        sensor_color=None,
        velocidad_max=100,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=120,
        kp=1.15,
        kd=2.6,
        k_freno=0.15,
        objetivo_reflexion=27,
        correccion_max=100,
        perfil_salida="encadenado",
        captura_inicial=True,
        tiempo_captura_ms=260,
        potencia_captura=55,
        kp_captura=2.4,
        margen_captura=5,
        lecturas_estables_captura=2,
        lecturas_color_estables=3
    ):
        """Sigue el borde de una línea hasta que el sensor detecte el color objetivo."""
        if sensor_color is None:
            sensor_color = self.seguidor

        multiplicador_lado = 1 if lado == "derecha" else -1
        self.chasis.reset_motores()

        # Fase 1: Captura inicial
        if captura_inicial:
            reloj_captura = StopWatch()
            reloj_captura.reset()
            estables = 0

            while reloj_captura.time() < tiempo_captura_ms:
                lectura = sensor_color.reflection()
                error = lectura - objetivo_reflexion

                if abs(error) <= margen_captura:
                    estables += 1
                    if estables >= lecturas_estables_captura:
                        break
                else:
                    estables = 0

                correccion = Utils.limitar(error * kp_captura * multiplicador_lado, -correccion_max, correccion_max)
                velocidad_base = 28 if abs(error) > 22 else potencia_captura

                potencia_izq = velocidad_base - correccion
                potencia_der = velocidad_base + correccion

                self.motor_izquierdo.dc(Utils.limitar(potencia_izq, -100, 100))
                self.motor_derecho.dc(Utils.limitar(potencia_der, -100, 100))
                wait(2)

            self.motor_izquierdo.brake()
            self.motor_derecho.brake()
            wait(8)
            self.chasis.reset_motores()

        # Fase 2: Seguimiento hasta color
        cronometro = StopWatch()
        cronometro.reset()

        velocidad_minima = 75
        error_anterior = 0
        derivada_anterior = 0
        lecturas_color = 0

        while True:
            hsv = sensor_color.hsv()
            reflexion = sensor_color.reflection()
            encontrado = False

            if color_objetivo in config.HSV_RANGOS:
                limites_color = config.HSV_RANGOS[color_objetivo]
                encontrado = True
                for componente, limites in limites_color.items():
                    minimo, maximo = limites
                    if componente == "h":
                        valor = hsv.h
                    elif componente == "s":
                        valor = hsv.s
                    elif componente == "v":
                        valor = hsv.v
                    elif componente == "reflection":
                        valor = reflexion
                    else:
                        encontrado = False
                        break
                    if not (minimo <= valor <= maximo):
                        encontrado = False
                        break
            else:
                encontrado = (sensor_color.color() == color_objetivo)

            if encontrado:
                lecturas_color += 1
            else:
                lecturas_color = 0

            if lecturas_color >= lecturas_color_estables:
                self.motor_izquierdo.dc(-100)
                self.motor_derecho.dc(-100)
                wait(30)
                self.motor_izquierdo.hold()
                self.motor_derecho.hold()
                wait(20)
                break

            tiempo_actual = cronometro.time()
            if tiempo_actual < tiempo_acomodo_ms:
                velocidad_actual = velocidad_minima
            elif tiempo_actual < tiempo_acomodo_ms + tiempo_aceleracion_ms:
                progreso = (tiempo_actual - tiempo_acomodo_ms) / float(tiempo_aceleracion_ms)
                velocidad_actual = velocidad_minima + (velocidad_max - velocidad_minima) * progreso
            else:
                velocidad_actual = velocidad_max

            lectura = reflexion
            error = lectura - objetivo_reflexion
            derivada = (error - error_anterior) * 0.82 + derivada_anterior * 0.18
            correccion = Utils.limitar(((error * kp) + (derivada * kd)) * multiplicador_lado, -correccion_max, correccion_max)

            velocidad_base = max(55, velocidad_actual - abs(error) * k_freno)
            potencia_izq = velocidad_base - correccion
            potencia_der = velocidad_base + correccion

            self.motor_izquierdo.dc(Utils.limitar(potencia_izq, -100, 100))
            self.motor_derecho.dc(Utils.limitar(potencia_der, -100, 100))

            error_anterior = error
            derivada_anterior = derivada
            wait(2)

    def seguir_linea_y_mover_torque(
        self,
        sensor_color=None,
        velocidad_max=100,
        distancia_cm=70,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=120,
        kp=1.15,
        kd=2.6,
        k_freno=0.15,
        objetivo_reflexion=27,
        correccion_max=100,
        margen_cm=0,
        perfil_salida="encadenado",
        captura_inicial=True,
        tiempo_captura_ms=260,
        potencia_captura=55,
        kp_captura=2.4,
        margen_captura=5,
        lecturas_estables_captura=2,
        torque_grados=None,
        torque_velocidad=180,
        distancia_torque_cm=0,
        retraso_ms=0,
        modo_torque="distancia",
        torque_porcentaje=None
    ):
        """Sigue línea y activa el motor de torque durante el recorrido."""
        if sensor_color is None:
            sensor_color = self.seguidor

        diametro_rueda_cm = self.diametro_rueda / 10.0
        circunferencia_cm = 3.14159 * diametro_rueda_cm
        grados_objetivo = (distancia_cm / circunferencia_cm) * 360.0
        grados_margen = (margen_cm / circunferencia_cm) * 360.0 if margen_cm > 0 else 0
        grados_objetivo_real = max(0, grados_objetivo - grados_margen)
        grados_activacion_torque = (distancia_torque_cm / circunferencia_cm) * 360.0

        multiplicador_lado = 1 if lado == "derecha" else -1
        self.chasis.reset_motores()

        if captura_inicial:
            reloj_captura = StopWatch()
            reloj_captura.reset()
            estables = 0
            while reloj_captura.time() < tiempo_captura_ms:
                lectura = sensor_color.reflection()
                error = lectura - objetivo_reflexion
                if abs(error) <= margen_captura:
                    estables += 1
                    if estables >= lecturas_estables_captura:
                        break
                else:
                    estables = 0
                correccion = Utils.limitar(error * kp_captura * multiplicador_lado, -correccion_max, correccion_max)
                velocidad_base = 28 if abs(error) > 22 else potencia_captura
                self.motor_izquierdo.dc(Utils.limitar(velocidad_base - correccion, -100, 100))
                self.motor_derecho.dc(Utils.limitar(velocidad_base + correccion, -100, 100))
                wait(2)
            self.motor_izquierdo.brake()
            self.motor_derecho.brake()
            wait(8)
            self.chasis.reset_motores()

        cronometro = StopWatch()
        cronometro.reset()
        velocidad_minima = 75
        error_anterior = 0
        derivada_anterior = 0
        torque_iniciado = False

        while True:
            grados_actuales = self.chasis.distancia_promedio_grados()
            if grados_actuales >= grados_objetivo_real:
                break

            tiempo_actual = cronometro.time()

            # Disparo de torque coordinado
            if (torque_porcentaje is not None or torque_grados is not None) and not torque_iniciado:
                disparar = False
                if modo_torque == "distancia" and grados_actuales >= grados_activacion_torque:
                    disparar = True
                elif modo_torque == "tiempo" and tiempo_actual >= retraso_ms:
                    disparar = True

                if disparar and self.torque is not None:
                    if torque_porcentaje is not None:
                        self.torque.ir_a_porcentaje(
                            porcentaje=torque_porcentaje,
                            velocidad=torque_velocidad,
                            wait_after=False
                        )
                    else:
                        self.torque.mover_grados(
                            grados_torque=torque_grados,
                            velocidad_torque=torque_velocidad,
                            esperar=False
                        )
                    torque_iniciado = True

            if tiempo_actual < tiempo_acomodo_ms:
                velocidad_actual = velocidad_minima
            elif tiempo_actual < tiempo_acomodo_ms + tiempo_aceleracion_ms:
                progreso = (tiempo_actual - tiempo_acomodo_ms) / float(tiempo_aceleracion_ms)
                velocidad_actual = velocidad_minima + (velocidad_max - velocidad_minima) * progreso
            else:
                velocidad_actual = velocidad_max

            lectura = sensor_color.reflection()
            error = lectura - objetivo_reflexion
            derivada = (error - error_anterior) * 0.82 + derivada_anterior * 0.18
            correccion = Utils.limitar(((error * kp) + (derivada * kd)) * multiplicador_lado, -correccion_max, correccion_max)

            velocidad_base = max(55, velocidad_actual - abs(error) * k_freno)
            potencia_izq = velocidad_base - correccion
            potencia_der = velocidad_base + correccion

            self.motor_izquierdo.dc(Utils.limitar(potencia_izq, -100, 100))
            self.motor_derecho.dc(Utils.limitar(potencia_der, -100, 100))

            error_anterior = error
            derivada_anterior = derivada
            wait(2)

        self.chasis.terminar_movimiento(perfil=perfil_salida, modo="brake")

    # =========================================================================
    # 3. DETECCIÓN DE COLOR Y CRUCES (Preservados de deteccion_color.py)
    # =========================================================================

    def _es_color(self, color):
        """Comprueba si la lectura actual está dentro de los rangos HSV calibrados."""
        if color not in config.HSV_RANGOS:
            raise ValueError("No existe calibración en HSV_RANGOS para {}".format(color))

        hsv = self.seguidor.hsv()
        lectura = {
            "h": hsv.h,
            "s": hsv.s,
            "v": hsv.v,
            "reflection": self.seguidor.reflection()
        }

        for componente, limites in config.HSV_RANGOS[color].items():
            minimo, maximo = limites
            valor = lectura[componente]
            if not (minimo <= valor <= maximo):
                return False
        return True

    def avanzar_hasta_color(
        self,
        color_objetivo,
        velocidad=300,
        kp_gyro=20.0,
        cruces=1,
        perfil="seguro"
    ):
        """Avanza recto con giroscopio hasta detectar una cantidad de cruces del color objetivo."""
        self.chasis.preparar_movimiento(reset_motores=False, reset_gyro=False, perfil=perfil)
        self.drive_base.reset()

        heading_objetivo = self.hub.imu.heading()
        signo = 1 if velocidad > 0 else -1
        conteo_cruces = 0
        viendo_color = False

        while True:
            color_actual = self.seguidor.color()
            hsv = self.seguidor.hsv()
            reflexion = self.seguidor.reflection()
            es_color_objetivo = False

            if color_objetivo == Color.BLUE:
                if hsv.s > 70:
                    es_color_objetivo = True
            elif color_objetivo == Color.BLACK:
                if hsv.s < 30 and reflexion < 15:
                    es_color_objetivo = True
            else:
                if color_actual == color_objetivo:
                    es_color_objetivo = True

            if es_color_objetivo:
                if not viendo_color:
                    viendo_color = True
                    conteo_cruces += 1
                    if conteo_cruces >= cruces:
                        break
            else:
                viendo_color = False

            actual_heading = self.hub.imu.heading()
            error_gyro = Utils.error_angular(heading_objetivo, actual_heading)
            self.drive_base.drive(abs(velocidad) * signo, error_gyro * kp_gyro)
            wait(2)

        self.drive_base.stop()
        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

    def avanzar_cruzando_lineas(
        self,
        cruces_objetivo=1,
        velocidad=300,
        escape_inicial_cm=0,
        margen_linea_cm=3.5,
        kp_gyro=20.0,
        perfil="seguro",
        retraso_freno_ms=0,
        distancia_extra_cm=0
    ):
        """Avanza recto contando líneas negras cruzadas con memoria de escape."""
        self.chasis.preparar_movimiento(reset_motores=False, reset_gyro=False, perfil=perfil)
        self.drive_base.reset()

        heading_objetivo = self.hub.imu.heading()
        signo = 1 if velocidad > 0 else -1
        conteo_cruces = 0
        distancia_desbloqueo_mm = escape_inicial_cm * 10.0

        while True:
            distancia_actual_mm = abs(self.drive_base.distance())
            actual_heading = self.hub.imu.heading()
            error_gyro = Utils.error_angular(heading_objetivo, actual_heading)

            self.drive_base.drive(abs(velocidad) * signo, error_gyro * kp_gyro)

            if distancia_actual_mm < distancia_desbloqueo_mm:
                wait(2)
                continue

            hsv = self.seguidor.hsv()
            reflexion = self.seguidor.reflection()
            es_linea_negra = (hsv.s < 30 and reflexion < 18)

            if es_linea_negra:
                conteo_cruces += 1
                if conteo_cruces >= cruces_objetivo:
                    break
                distancia_desbloqueo_mm = distancia_actual_mm + (margen_linea_cm * 10.0)

            wait(2)

        # Avance extra por distancia
        if abs(distancia_extra_cm) > 0:
            self.drive_base.reset()
            distancia_extra_mm = abs(distancia_extra_cm) * 10.0
            signo_extra = signo if distancia_extra_cm > 0 else -signo

            while abs(self.drive_base.distance()) < distancia_extra_mm:
                actual_heading = self.hub.imu.heading()
                error_gyro = Utils.error_angular(heading_objetivo, actual_heading)
                self.drive_base.drive(abs(velocidad) * signo_extra, error_gyro * kp_gyro)
                wait(2)

        # Retraso opcional adicional por tiempo
        if retraso_freno_ms > 0:
            reloj_extra = StopWatch()
            reloj_extra.reset()
            while reloj_extra.time() < retraso_freno_ms:
                actual_heading = self.hub.imu.heading()
                error_gyro = Utils.error_angular(heading_objetivo, actual_heading)
                self.drive_base.drive(abs(velocidad) * signo, error_gyro * kp_gyro)
                wait(2)

        self.drive_base.stop()
        if perfil == "encadenado":
            self.motor_izquierdo.brake()
            self.motor_derecho.brake()
            wait(10)
        else:
            self.motor_izquierdo.brake()
            self.motor_derecho.brake()
            wait(60)
            self.motor_izquierdo.hold()
            self.motor_derecho.hold()
            wait(20)

    def avanzar_hibrido(
        self,
        distancia_inicial_cm,
        color_objetivo,
        velocidad_max=400,
        velocidad_min=150,
        kp_gyro=20.0,
        zona_transicion_cm=3,
        perfil="encadenado",
        cruces=1,
        **kwargs
    ):
        """Combina un avance por distancia con una búsqueda posterior por color."""
        if distancia_inicial_cm == 0:
            return self.avanzar_hasta_color(
                color_objetivo=color_objetivo,
                velocidad=velocidad_max,
                kp_gyro=kp_gyro,
                cruces=cruces,
                perfil=perfil
            )

        self.chasis.preparar_movimiento(reset_motores=False, reset_gyro=False, perfil=perfil)
        self.drive_base.reset()

        heading_objetivo = self.hub.imu.heading()
        signo = 1 if velocidad_max > 0 else -1
        zona_transicion_cm = min(zona_transicion_cm, distancia_inicial_cm / 2.0)

        fase_busqueda_activa = False
        conteo_cruces = 0
        viendo_color = False

        cronometro = StopWatch()
        cronometro.reset()

        velocidad_actual = velocidad_min

        while True:
            distancia_actual = abs(self.drive_base.distance()) / 10.0
            distancia_restante = distancia_inicial_cm - distancia_actual

            if color_objetivo in config.HSV_RANGOS:
                es_color_objetivo = self._es_color(color_objetivo)
            else:
                es_color_objetivo = (self.seguidor.color() == color_objetivo)

            if fase_busqueda_activa and es_color_objetivo:
                if not viendo_color:
                    viendo_color = True
                    conteo_cruces += 1
                    if conteo_cruces >= cruces:
                        break
            else:
                viendo_color = False

            if distancia_actual >= distancia_inicial_cm and not fase_busqueda_activa:
                fase_busqueda_activa = True

            if not fase_busqueda_activa:
                tiempo_actual = cronometro.time()
                if tiempo_actual < 150:
                    factor = tiempo_actual / 150.0
                    velocidad_actual = velocidad_min + (velocidad_max - velocidad_min) * factor
                else:
                    velocidad_actual = velocidad_max

                if 0 < distancia_restante < zona_transicion_cm:
                    progreso = distancia_restante / zona_transicion_cm
                    velocidad_actual *= (0.5 + 0.5 * progreso)
                    if velocidad_actual < 60:
                        velocidad_actual = 60

                if distancia_restante <= 0:
                    velocidad_actual = velocidad_max * 0.9
            else:
                if conteo_cruces < cruces:
                    if conteo_cruces > 0:
                        factor = 1.0 - (conteo_cruces / float(cruces)) * 0.3
                        velocidad_actual = velocidad_max * 0.6 * factor
                    else:
                        velocidad_actual = velocidad_max * 0.6
                    velocidad_actual = max(velocidad_actual, 100)
                else:
                    velocidad_actual = velocidad_max * 0.5

            actual_heading = self.hub.imu.heading()
            error_gyro = Utils.error_angular(heading_objetivo, actual_heading)
            velocidad_final = max(velocidad_actual, 80)

            self.drive_base.drive(velocidad_final * signo, error_gyro * kp_gyro)
            wait(3)

        self.drive_base.stop()
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        wait(60)
        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

    def avanzar_hasta_salir_negro(
        self,
        velocidad_max=900,
        velocidad_min=100,
        kp_gyro=20.0,
        zona_rampa_cm=8,
        perfil="encadenado",
        rumbo_esperado=None,
        sensor_color=None,
        objetivo_reflexion=15,
        lecturas_salida=5,
        torque_grados=None,
        torque_velocidad=180,
        torque_retraso_ms=0
    ):
        """Avanza recto manteniendo rumbo hasta dejar de detectar reflexión negra."""
        if sensor_color is None:
            sensor_color = self.seguidor

        self.chasis.preparar_movimiento(reset_motores=False, reset_gyro=False, perfil=perfil)
        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(90)
        self.drive_base.reset()

        if rumbo_esperado is not None:
            heading_objetivo = rumbo_esperado
        else:
            heading_objetivo = self.hub.imu.heading()

        grados_por_cm = config.GRADOS_POR_CM
        grados_rampa = zona_rampa_cm * grados_por_cm

        cronometro = StopWatch()
        cronometro.reset()
        torque_iniciado = False
        contador_salida = 0

        while True:
            reflexion = sensor_color.reflection()

            if reflexion > objetivo_reflexion:
                contador_salida += 1
            else:
                contador_salida = 0

            if contador_salida >= lecturas_salida:
                break

            recorrido = abs(self.drive_base.distance()) * self.grados_por_mm

            if recorrido < grados_rampa:
                proporcion = recorrido / grados_rampa
                vel_base = velocidad_min + (velocidad_max - velocidad_min) * proporcion
            else:
                vel_base = velocidad_max

            tiempo_ms = cronometro.time()

            if torque_grados is not None and not torque_iniciado and tiempo_ms >= torque_retraso_ms:
                if self.torque is not None:
                    self.torque.mover_grados(
                        grados_torque=torque_grados,
                        velocidad_torque=torque_velocidad,
                        esperar=False
                    )
                torque_iniciado = True

            if tiempo_ms < 150:
                vel = vel_base * (tiempo_ms / 150.0)
            else:
                vel = vel_base

            vel = Utils.limitar(vel, 25, velocidad_max)
            actual_heading = self.hub.imu.heading()
            error_gyro = Utils.error_angular(heading_objetivo, actual_heading)
            correccion_giro = error_gyro * kp_gyro

            self.drive_base.drive(vel, correccion_giro)
            wait(2)

        self.drive_base.stop()
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        wait(60)
        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)
