"""Módulos y controladores de mecanismos y actuadores del robot.

Estructurado con la arquitectura de clases orientada a objetos de MIO,
proporcionando control absoluto por porcentaje (ir_a_porcentaje) y
conservando las funciones originales en grados para facilitar la transición.
"""

from pybricks.parameters import Stop
from pybricks.tools import wait, StopWatch
import config
from Utils import Utils


class Mecanismo:
    """Clase base para cualquier mecanismo accionado por un motor."""

    def __init__(self, motor, rango_maximo=None):
        """
        Argumentos:
            motor: Instancia de Motor de Pybricks.
            rango_maximo: Rango total en grados calibrado para el 100% de carrera.
        """
        self.motor = motor
        self.rango_maximo = rango_maximo

    def reset_angle(self, angulo=0):
        """Reinicia el encoder del motor al ángulo indicado."""
        self.motor.reset_angle(angulo)

    def angle(self):
        """Devuelve la posición angular actual del motor."""
        return self.motor.angle()

    def brake(self):
        """Frena el motor de forma pasiva."""
        self.motor.brake()

    def hold(self):
        """Mantiene activamente la posición actual del motor."""
        self.motor.hold()

    def stop(self):
        """Detiene el motor dejando el eje libre."""
        self.motor.stop()

    def mover(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD):
        """Mueve el motor una cantidad relativa de grados."""
        self.motor.run_angle(velocidad, grados, then=frenado, wait=wait_after)

    def llevar_al_tope(self, direccion="positivo", velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        """Lleva el mecanismo hasta trabarse contra su tope físico."""
        vel_real = abs(velocidad) if direccion == "positivo" else -abs(velocidad)
        return self.motor.run_until_stalled(vel_real, then=frenado, duty_limit=limite_potencia)

    def grados_a_porcentaje(self, grados):
        """Convierte un ángulo en grados al porcentaje correspondiente (0 a 100%)."""
        if self.rango_maximo is None or self.rango_maximo == 0:
            return 0.0
        return (float(grados) / self.rango_maximo) * 100.0

    def ir_a_porcentaje(self, porcentaje, velocidad=800, wait_after=True, frenado=Stop.HOLD):
        """Lleva el mecanismo a una posición proporcional a su rango calibrado (0 a 100%).

        Argumentos:
            porcentaje: Posición deseada del 0.0 al 100.0%.
            velocidad: Grados/s del movimiento.
            wait_after: Si es True, bloquea hasta terminar; si es False, continúa en segundo plano.
            frenado: Modo de frenado al alcanzar la meta.
        """
        if self.rango_maximo is None:
            raise ValueError("Debes configurar el rango_maximo del mecanismo en config.py")

        porcentaje_seguro = Utils.limitar(float(porcentaje), 0.0, 100.0)
        angulo_objetivo = (porcentaje_seguro / 100.0) * self.rango_maximo

        self.motor.run_target(
            speed=abs(velocidad),
            target_angle=angulo_objetivo,
            then=frenado,
            wait=wait_after
        )


class MecanismoTorque(Mecanismo):
    """Mecanismo de torque (Port D): levanta, toma o suelta cementos y pala."""

    def __init__(self, motor, rango_maximo=None):
        if rango_maximo is None:
            rango_maximo = config.RANGO_MAXIMO_TORQUE
        super().__init__(motor, rango_maximo=rango_maximo)

    def establecer_cero(self):
        """Fija la posición actual como el origen 0°."""
        self.motor.reset_angle(0)

    def subir_al_tope(self, velocidad=600):
        """Lleva el torque a su posición superior guardada (0%)."""
        self.ir_a_porcentaje(0, velocidad=velocidad, wait_after=True)

    def bajar_al_tope(self, velocidad=600):
        """Lleva el torque a su posición inferior máxima (100%)."""
        self.ir_a_porcentaje(100, velocidad=velocidad, wait_after=True)

    def mover_grados(self, grados_torque, velocidad_torque=180, esperar=True, modo_final=Stop.HOLD, retraso_inicial_ms=0):
        """Mueve el mecanismo una cantidad relativa de grados (interfaz original de mover_torque)."""
        if retraso_inicial_ms > 0:
            wait(retraso_inicial_ms)

        if grados_torque == 0:
            return

        self.motor.run_angle(
            velocidad_torque,
            grados_torque,
            then=modo_final,
            wait=esperar
        )

    def mover_seguro(self, grados_torque, velocidad_torque=180, esperar=True, modo_final=Stop.HOLD,
                     retraso_inicial_ms=0, duty_limit=50, detener_en_tope=True):
        """Mueve el mecanismo hasta el ángulo o hasta encontrar resistencia mecánica (tope)."""
        if retraso_inicial_ms > 0:
            wait(retraso_inicial_ms)

        if grados_torque == 0:
            return False

        self.motor.stop()
        wait(10)
        self.motor.reset_angle(0)

        velocidad_real = abs(velocidad_torque) if grados_torque > 0 else -abs(velocidad_torque)
        modo_tope = Stop.HOLD if detener_en_tope else Stop.COAST

        angulo_final = self.motor.run_until_stalled(
            speed=velocidad_real,
            then=modo_tope,
            duty_limit=duty_limit
        )

        angulo_real = abs(angulo_final)
        angulo_objetivo = abs(grados_torque)
        tope_detectado = angulo_real < angulo_objetivo

        if not detener_en_tope:
            if modo_final == Stop.HOLD:
                self.motor.hold()
            elif modo_final == Stop.BRAKE:
                self.motor.brake()
            else:
                self.motor.stop()

        if esperar:
            wait(20)

        return tope_detectado


class GarraDelantera(Mecanismo):
    """Mecanismo de la garra delantera (Port B): 0° arriba, 353° abajo."""

    def __init__(self, motor, rango_maximo=None):
        if rango_maximo is None:
            rango_maximo = config.RANGO_MAXIMO_GARRA_DELANTERA
        super().__init__(motor, rango_maximo=rango_maximo)

    def establecer_cero(self):
        """Fija la posición actual como el origen 0°."""
        self.motor.reset_angle(0)

    def mover_grados(self, posicion, velocidad=700, simultaneo=False, modo_final=Stop.HOLD,
                     limite_minimo=0, limite_maximo=None):
        """Mueve la garra a una posición absoluta en grados (interfaz original de mover_garra_delantera)."""
        if limite_maximo is None:
            limite_maximo = abs(self.rango_maximo) if self.rango_maximo else abs(config.RANGO_MAXIMO_GARRA_DELANTERA)
        posicion_acotada = Utils.limitar(posicion, limite_minimo, limite_maximo)
        self.motor.run_target(
            speed=abs(velocidad),
            target_angle=posicion_acotada,
            then=modo_final,
            wait=not simultaneo
        )
        return posicion_acotada

    def subir_al_tope(self, velocidad=700):
        """Lleva la garra a su posición superior (0°)."""
        self.ir_a_porcentaje(0, velocidad=velocidad, wait_after=True)

    def bajar_al_tope(self, velocidad=700):
        """Lleva la garra a su posición inferior máxima (100%)."""
        self.ir_a_porcentaje(100, velocidad=velocidad, wait_after=True)

    def ir_a_porcentaje_pinza(self, *args, **kwargs):
        """Delegación de compatibilidad para código estilo MIO (donde la pinza colgaba de la delantera)."""
        if hasattr(self, "pinza") and self.pinza is not None:
            return self.pinza.ir_a_porcentaje(*args, **kwargs)

    def cerrar_al_tope_pinza(self, *args, **kwargs):
        """Delegación de compatibilidad para cerrar pinza."""
        if hasattr(self, "pinza") and self.pinza is not None:
            return self.pinza.cerrar_al_tope(*args, **kwargs)


class GarraPrincipal(Mecanismo):
    """Garra principal / pinza (Port F): apriete de objetos, apertura y cierre."""

    def __init__(self, motor, rango_maximo=None):
        if rango_maximo is None:
            rango_maximo = config.RANGO_MAXIMO_GARRA_PRINCIPAL
        super().__init__(motor, rango_maximo=rango_maximo)

    def ir_a_porcentaje(self, porcentaje, velocidad=800, wait_after=True, frenado=Stop.HOLD, duty_cierre=100):
        """Lleva la garra a un porcentaje de apertura (0% = cerrada, 100% = abierta al máximo).

        Argumentos:
            porcentaje: Grado de apertura del 0.0 al 100.0%.
            velocidad: Grados/s de desplazamiento.
            wait_after: True bloquea hasta alcanzar la posición; False continúa en paralelo.
            frenado: Modo de detención al finalizar.
            duty_cierre: Límite de potencia aplicado al trabarse en 0% si wait_after=True.
        """
        if self.rango_maximo is None:
            raise ValueError("Debes configurar el rango_maximo del mecanismo en config.py")

        porcentaje_seguro = Utils.limitar(float(porcentaje), 0.0, 100.0)

        # Cierre absoluto a tope con re-calibración de cero cuando se pide 0% y es bloqueante
        if porcentaje_seguro == 0 and wait_after:
            self.motor.run_until_stalled(
                -abs(velocidad),
                then=frenado,
                duty_limit=duty_cierre
            )
            self.motor.reset_angle(0)
        else:
            angulo_objetivo = (porcentaje_seguro / 100.0) * self.rango_maximo
            self.motor.run_target(
                speed=abs(velocidad),
                target_angle=angulo_objetivo,
                then=frenado,
                wait=wait_after
            )

    def apretar(self, potencia=100, velocidad=300, frenado=Stop.HOLD):
        """Cierra la pinza hasta encontrar resistencia mecánica (agarre de objetos de tamaño variable)."""
        potencia_apriete = Utils.limitar(potencia, -100, 100)
        return self.motor.run_until_stalled(
            -abs(velocidad),
            then=frenado,
            duty_limit=abs(potencia_apriete)
        )

    def abrir_al_tope(self, velocidad=800):
        """Abre la garra a su posición máxima (100%)."""
        self.ir_a_porcentaje(100, velocidad=velocidad, wait_after=True)

    def cerrar_al_tope(self, velocidad=800, duty_limit=100):
        """Cierra la garra hasta trabarse al tope físico y reinicia el encoder a 0°."""
        self.motor.run_until_stalled(
            -abs(velocidad),
            then=Stop.HOLD,
            duty_limit=duty_limit
        )
        self.motor.reset_angle(0)

    def establecer_cero(self):
        """Fija la posición actual como el origen 0°."""
        self.motor.reset_angle(0)

    def mover_grados(self, velocidad, grados=0, esperar=True, potencia_apriete=150,
                     tiempo_apriete_ms=120, apretar=False, modo_soltar=None, duty_cierre=100):
        """Control manual, por posición o por apriete (interfaz original de mover_garra_principal)."""
        if modo_soltar == "hold":
            self.motor.hold()
            return
        elif modo_soltar == "stop":
            self.motor.stop()
            return
        elif modo_soltar == "brake":
            self.motor.brake()
            return

        velocidad = abs(velocidad) if velocidad != 0 else 200

        # Modo apriete
        if apretar:
            self.apretar(potencia=potencia_apriete, velocidad=velocidad)
            return

        # Modo posición
        if grados <= 0:
            self.ir_a_porcentaje(0, velocidad=velocidad, wait_after=esperar, duty_cierre=duty_cierre)
        else:
            porcentaje = self.grados_a_porcentaje(grados)
            self.ir_a_porcentaje(porcentaje, velocidad=velocidad, wait_after=esperar)

    def mover_rapida(self, grados=130, potencia=100, tiempo_max_ms=1200, abrir=False):
        """Movimiento directo DC por tiempo o grados para expulsar/soltar piezas."""
        if grados <= 0:
            return

        potencia_lim = Utils.limitar(abs(potencia), 10, 100)
        potencia_real = potencia_lim if not abrir else -potencia_lim

        self.motor.stop()
        wait(10)
        self.motor.reset_angle(0)

        cronometro = StopWatch()
        cronometro.reset()

        self.motor.dc(potencia_real)

        while True:
            if abs(self.motor.angle()) >= grados:
                break
            if cronometro.time() >= tiempo_max_ms:
                break
            wait(2)

        self.motor.hold()
