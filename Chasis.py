"""Módulo de control del chasis y tren de tracción.

Contiene las operaciones de bajo nivel (frenado, reseteo, preparación) y
el avance recto con control de giroscopio y perfil de aceleración/desaceleración
proveniente del equipo original.
"""

from pybricks.tools import wait, StopWatch
import config
from Utils import Utils


class Chasis:
    """Controlador de tracción y desplazamientos rectos."""

    def __init__(self, drive_base, motor_izquierdo, motor_derecho, hub, torque=None):
        """
        Argumentos:
            drive_base: Instancia de DriveBase configurada.
            motor_izquierdo: Motor de la rueda izquierda.
            motor_derecho: Motor de la rueda derecha.
            hub: Instancia de PrimeHub.
            torque: Mecanismo de torque opcional para acciones coordinadas.
        """
        self.drive_base = drive_base
        self.motor_izquierdo = motor_izquierdo
        self.motor_derecho = motor_derecho
        self.hub = hub
        self.torque = torque

        self.diametro_rueda = config.DIAMETRO_RUEDA
        self.circunferencia = config.CIRCUNFERENCIA
        self.grados_por_mm = config.GRADOS_POR_MM

    # =========================================================================
    # UTILIDADES DE BAJO NIVEL (Preservadas de utilidades.py)
    # =========================================================================

    def frenar(self):
        """Frena inmediatamente ambos motores de tracción."""
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        wait(3)

    def reset_motores(self):
        """Reinicia los contadores de grados de ambos motores de tracción."""
        self.motor_izquierdo.reset_angle(0)
        self.motor_derecho.reset_angle(0)

    def distancia_promedio_grados(self):
        """Calcula el promedio absoluto de grados recorridos por ambas ruedas."""
        return (abs(self.motor_izquierdo.angle()) + abs(self.motor_derecho.angle())) / 2.0

    def preparar_movimiento(self, reset_motores=True, reset_gyro=True, perfil="seguro", pausa=None):
        """Prepara sensores y motores antes de iniciar un movimiento."""
        if reset_motores:
            self.reset_motores()
            self.drive_base.reset()

        if reset_gyro:
            self.hub.imu.reset_heading(0)
            wait(15)

        if pausa is not None:
            wait(pausa)

    def terminar_movimiento(self, perfil="seguro", modo="brake", pausa=None, soltar=True):
        """Aplica el frenado y las pausas correspondientes según el perfil."""
        if modo == "brake":
            self.motor_izquierdo.brake()
            self.motor_derecho.brake()
        elif modo == "stop":
            self.motor_izquierdo.stop()
            self.motor_derecho.stop()
        elif modo == "hold":
            self.motor_izquierdo.hold()
            self.motor_derecho.hold()

        if pausa is not None:
            wait(pausa)
        elif perfil == "seguro":
            wait(18)
        elif perfil == "encadenado":
            wait(6)
        else:
            wait(15)

        if soltar and modo == "brake":
            self.motor_izquierdo.stop()
            self.motor_derecho.stop()
            wait(2)

    # =========================================================================
    # AVANCE RECTO CON CONTROL DE IMU Y RAMPAS (Preservado de navegacion.py)
    # =========================================================================

    def avanzar_recto(
        self,
        distancia_cm,
        velocidad_max=900,
        velocidad_min=100,
        kp_gyro=20.0,
        zona_rampa_cm=8,
        perfil="encadenado",
        rumbo_esperado=None,
        torque_grados=None,
        torque_velocidad=180,
        torque_retraso_ms=0,
        accion_torque_callback=None,
        torque_porcentaje=None
    ):
        """Avanza o retrocede una distancia manteniendo el rumbo con IMU y rampas de velocidad."""
        if distancia_cm == 0:
            return

        self.preparar_movimiento(
            reset_motores=False,
            reset_gyro=False,
            perfil=perfil
        )

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(90)

        self.drive_base.reset()

        # Memoria de rumbo
        if rumbo_esperado is not None:
            heading_objetivo = rumbo_esperado
        else:
            heading_objetivo = self.hub.imu.heading()

        grados_por_cm = config.GRADOS_POR_CM
        grados_objetivo = abs(distancia_cm) * grados_por_cm
        rampa_efectiva_cm = min(zona_rampa_cm, abs(distancia_cm) / 2.0)
        grados_rampa = rampa_efectiva_cm * grados_por_cm
        signo = 1 if distancia_cm > 0 else -1

        cronometro = StopWatch()
        cronometro.reset()

        torque_iniciado = False

        while True:
            recorrido = abs(self.drive_base.distance()) * self.grados_por_mm
            restante = grados_objetivo - recorrido

            if restante <= 1.5:
                break

            actual_heading = self.hub.imu.heading()
            error_gyro = Utils.error_angular(heading_objetivo, actual_heading)

            # Rampa de aceleración
            if recorrido < grados_rampa:
                proporcion = recorrido / grados_rampa
                vel_base = velocidad_min + (velocidad_max - velocidad_min) * proporcion
            # Rampa de desaceleración
            elif restante < grados_rampa:
                proporcion = restante / grados_rampa
                vel_base = velocidad_min * 0.4 + (velocidad_max - velocidad_min * 0.4) * proporcion
            else:
                vel_base = velocidad_max

            tiempo_ms = cronometro.time()

            # Torque retrasado durante el avance
            if (torque_porcentaje is not None or torque_grados is not None) and not torque_iniciado and tiempo_ms >= torque_retraso_ms:
                if accion_torque_callback is not None:
                    accion_torque_callback()
                elif self.torque is not None:
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

            # Rampa temporal inicial original
            if tiempo_ms < 150:
                vel = vel_base * (tiempo_ms / 150.0)
            else:
                vel = vel_base

            vel = Utils.limitar(vel, 25, velocidad_max)
            correccion_giro = error_gyro * kp_gyro

            self.drive_base.drive(vel * signo, correccion_giro)

        # Frenado original de dos fases
        self.drive_base.stop()
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        wait(60)

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

    def avanzar_con_torque(
        self,
        distancia_cm,
        distancia_activacion_torque_cm,
        torque_grados=None,
        torque_velocidad=900,
        velocidad_max=900,
        velocidad_min=100,
        kp_gyro=20.0,
        zona_rampa_cm=8,
        perfil="encadenado",
        rumbo_esperado=None,
        accion_torque_callback=None,
        torque_porcentaje=None
    ):
        """Avanza recto activando el torque en función de la distancia recorrida."""
        if distancia_cm == 0:
            return

        self.preparar_movimiento(
            reset_motores=False,
            reset_gyro=False,
            perfil=perfil
        )

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(90)

        self.drive_base.reset()

        if rumbo_esperado is not None:
            heading_objetivo = rumbo_esperado
        else:
            heading_objetivo = self.hub.imu.heading()

        grados_por_cm = config.GRADOS_POR_CM
        grados_objetivo = abs(distancia_cm) * grados_por_cm
        grados_activacion_torque = abs(distancia_activacion_torque_cm) * grados_por_cm
        rampa_efectiva_cm = min(zona_rampa_cm, abs(distancia_cm) / 2.0)
        grados_rampa = rampa_efectiva_cm * grados_por_cm
        signo = 1 if distancia_cm > 0 else -1

        cronometro = StopWatch()
        cronometro.reset()

        torque_iniciado = False

        while True:
            recorrido = abs(self.drive_base.distance()) * self.grados_por_mm
            restante = grados_objetivo - recorrido

            if restante <= 1.5:
                break

            actual_heading = self.hub.imu.heading()
            error_gyro = Utils.error_angular(heading_objetivo, actual_heading)

            if recorrido < grados_rampa:
                proporcion = recorrido / grados_rampa
                vel_base = velocidad_min + (velocidad_max - velocidad_min) * proporcion
            elif restante < grados_rampa:
                proporcion = restante / grados_rampa
                vel_base = velocidad_min * 0.4 + (velocidad_max - velocidad_min * 0.4) * proporcion
            else:
                vel_base = velocidad_max

            tiempo_ms = cronometro.time()

            if (torque_porcentaje is not None or torque_grados is not None) and not torque_iniciado and recorrido >= grados_activacion_torque:
                if accion_torque_callback is not None:
                    accion_torque_callback()
                elif self.torque is not None:
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

            if tiempo_ms < 150:
                vel = vel_base * (tiempo_ms / 150.0)
            else:
                vel = vel_base

            vel = Utils.limitar(vel, 25, velocidad_max)
            correccion_giro = error_gyro * kp_gyro

            self.drive_base.drive(vel * signo, correccion_giro)

        self.drive_base.stop()
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        wait(60)

        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

    def cuadrar_contra_pared(self, tiempo_ms=1000, potencia=30, angulo_referencia=0, reversa=True):
        """Empuja contra una pared para alinearse físicamente y reiniciar el rumbo del IMU."""
        self.drive_base.stop()
        potencia_aplicada = -abs(potencia) if reversa else abs(potencia)

        self.motor_izquierdo.dc(potencia_aplicada)
        self.motor_derecho.dc(potencia_aplicada)
        wait(tiempo_ms)

        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        self.hub.imu.reset_heading(angulo_referencia)
        wait(20)
