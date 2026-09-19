"""Secuencia de misiones autónomas del Reto 1 para WRO 2026.

Estructura modular dividida en misiones atómicas individuales que pueden
ejecutarse por separado para calibración y pruebas en pista.
"""

from pybricks.parameters import Color
from pybricks.tools import wait
import gc
import config
from robot import Robot


class Misiones:
    """Misiones del recorrido secuencial del Reto 1."""

    def __init__(self, robot: Robot, sensor_frente):
        """
        Argumentos:
            robot: instancia de Robot con el chasis y los mecanismos armados.
            sensor_frente: ColorSensor delantero (el seguidor).
        """
        self.robot = robot
        self.sensor = sensor_frente

    # =========================================================================
    # AUXILIARES
    # =========================================================================

    def _seguir_linea_distancia(
        self,
        distancia_cm,
        velocidad_max=100,
        lado="derecha",
        kp=1.15,
        kd=3.8,
        k_freno=0.05,
        tiempo_acomodo_ms=50,
        encadenado=True):
        """Sigue el borde de la línea una distancia fija utilizando seguidor_linea_cruces."""
        self.robot.navegacion.seguidor_linea_cruces(
            self.sensor,
            velocidad_max=velocidad_max,
            cruces_objetivo=0,
            distancia_extra_cm=distancia_cm,
            distancia_inicial_cm=0,
            lado=lado,
            tiempo_acomodo_ms=tiempo_acomodo_ms,
            kp=kp,
            kd=kd,
            k_freno=k_freno,
            margen_cm=0,
            encadenado=encadenado
        )

    def _identificar_combinacion(self, distancia_verificacion_cm=5):
        """Lee el mosaico a armar y devuelve su número según config.MOSAICOS."""
        color_principal = self.robot.navegacion.detectar_color_preciso(self.sensor)
        if color_principal not in config.MOSAICOS:
            return -1

        decision = config.MOSAICOS[color_principal]

        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_verificacion_cm)
            wait(50)
            color_anterior = self.robot.navegacion.detectar_color_preciso(self.sensor)
            self.robot.chasis.avanzar_recto(-distancia_verificacion_cm)

            if color_anterior not in decision:
                return -1
            return decision[color_anterior]

        return decision

    # =========================================================================
    # MISIONES ATÓMICAS (RETO 1)
    # =========================================================================

    def agarrar_cemento(self):
        """Salida inicial, seguidor de línea y captura del cemento con la jaula trasera."""
        self.robot.navegacion.giro_relativo(90, rueda_pivote="derecha", max_potencia=100, min_potencia=100, encadenado=True)

        self._seguir_linea_distancia(
            distancia_cm=79,
            velocidad_max=100,
            lado="derecha",
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            tiempo_acomodo_ms=50,
            encadenado=True
        )
        gc.collect()

        self.robot.navegacion.giro_relativo(-90, max_potencia=100, encadenado=True)
        self.robot.garra_trasera.ir_a_porcentaje(94.4, velocidad=600, wait_after=False)
        self.robot.chasis.avanzar_recto(-12, velocidad=1000, encadenado=False)

    def dejar_llana(self):
        """Empuja la llana hacia su zona y regresa a la línea cruzando intersecciones."""
        self.robot.chasis.avanzar_recto(8, velocidad=1000, encadenado=False)
        self.robot.navegacion.giro_relativo(70, max_potencia=100, encadenado=True)

        self.robot.chasis.avanzar_recto(-32, velocidad=1000)

        self.robot.chasis.avanzar_recto(8, velocidad=900, encadenado=True)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, 5, Color.WHITE, 20, velocidad_escaneo=300)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, 5, Color.BLACK, 15, velocidad_alta=300, velocidad_escaneo=300, distancia_extra_cm=5)
        self.robot.navegacion.giro_relativo(10, rueda_pivote="derecha", max_potencia=100, min_potencia=80, encadenado=True)

    def dejar_cemento(self):
        """Sigue la línea hasta el cruce, encara el parqueo y suelta ahí el cemento."""
        self.robot.navegacion.seguidor_linea_cruces(
            self.sensor,
            velocidad_max=100,
            cruces_objetivo=1,
            distancia_extra_cm=32,
            distancia_inicial_cm=18,
            tiempo_acomodo_ms=0
        )

        self.robot.navegacion.giro_relativo(90, max_potencia=85, encadenado=True)
        self.robot.chasis.avanzar_recto(-10, velocidad=1000)

        self.robot.garra_trasera.ir_a_porcentaje(0, wait_after=False)
        self.robot.chasis.avanzar_recto(5, velocidad=1000)

    def mover_pala_al_camino(self, distancia_pala_cm=24):
        """Recoge la pala con la jaula trasera y la deja sobre el camino de salida.

        Aprovecha que la jaula queda libre apenas se suelta el cemento. La pala
        queda apoyada donde el robot vuelve a pasar al ir a dejar los amarillos.
        Deja al robot perfilado hacia el pasillo de salida.

        Argumentos:
            distancia_pala_cm: cuánto retrocede para calzarle la jaula a la pala.
        """
        self.robot.navegacion.giro_relativo(-130)
        self.robot.chasis.avanzar_recto(-distancia_pala_cm, wait_after=False)
        wait(350)
        self.robot.garra_trasera.ir_a_porcentaje(90, velocidad=800)
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            distancia_total_cm=40,
            distancia_accion_cm=20,
            margen_cm=2,
            accion_callback=lambda: self.robot.garra_trasera.ir_a_porcentaje(
                0, wait_after=False
            ),
        )
        gc.collect()

    def agarrar_verdes(self):
        """Gira hacia la línea, detecta la intersección (blanco/negro), encara y retrocede atrapando los verdes con la jaula."""
        self.robot.navegacion.giro_relativo(-20)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, 5, Color.WHITE, 20, velocidad_escaneo=300)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, 5, Color.BLACK, 15, velocidad_alta=300, velocidad_escaneo=300, distancia_extra_cm=5)
        
        self.robot.navegacion.giro_relativo(-110, max_potencia=90, encadenado=True)

        self.robot.garra_trasera.ir_a_porcentaje(95.0, velocidad=250, wait_after=False)
        self.robot.chasis.avanzar_recto(-17, velocidad=600, encadenado=True)

    def escanear_mosaico(self, distancia_verificacion_cm=7):
        """Sigue la línea por la izquierda, entra a la matriz y escanea el mosaico."""
        self.robot.garra_trasera.ir_a_porcentaje(97, wait_after=False)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=150, potencia=80)
        self.robot.navegacion.seguidor_linea_color(
            self.sensor,
            velocidad_max=100,
            color_objetivo=Color.GREEN,
            lado="izquierda",
            tiempo_acomodo_ms=0,
            distancia_cm=70,
            encadenado=True
        )

        self.robot.navegacion.desplazar_lateral(2)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.chasis.avanzar_recto(9, velocidad=700)

        mosaico = self._identificar_combinacion(distancia_verificacion_cm)
        print("Mosaico detectado:", mosaico)
        return mosaico

    def dejar_verdes(self):
        """Sale de la matriz en reversa, gira 180° y deposita los bloques verdes."""
        self.robot.chasis.avanzar_recto(-32, velocidad=900)
        self.robot.navegacion.giro_relativo(-180, max_potencia=90, encadenado=True)
        gc.collect()

        self.robot.chasis.avanzar_recto(-16, velocidad=900, encadenado=True)
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=350, wait_after=False)

    def agarrar_amarillos(self):
        """Sigue la línea y navega el pasillo en zigzag recolectando los bloques amarillos."""
        self.robot.navegacion.giro_relativo(20, min_potencia=70, encadenado=True)
        self.robot.navegacion.seguidor_linea_color(
            self.sensor,
            100,
            Color.GREEN,
            tiempo_acomodo_ms=100,
            distancia_cm=70
        )
        self.robot.chasis.avanzar_recto(2, encadenado=True)
        self.robot.navegacion.giro_absoluto(90, rueda_pivote="izquierda")
        self.robot.chasis.avanzar_recto(18, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(-90, max_potencia=80, encadenado=False)
        self.robot.chasis.avanzar_recto(20, velocidad=900, encadenado=False)
        self.robot.navegacion.giro_relativo(-90, max_potencia=80, encadenado=False)
        wait(100)

    def agarrar_azules(self):
        """Avanza recto, gira hacia los bloques azules y los encierra con la jaula."""
        self.robot.chasis.avanzar_recto(55, encadenado=False)
        self.robot.navegacion.giro_relativo(90, max_potencia=80, encadenado=True)

        self.robot.garra_trasera.ir_a_porcentaje(95.0, velocidad=300, wait_after=False)
        self.robot.chasis.avanzar_recto(-11, velocidad=600, encadenado=True)

    def agarrar_pala(self):
        """Posiciona la garra delantera y pinza abierta, encara la pala y avanza para calzarla."""
        self.robot.garra_delantera.ir_a_porcentaje(65, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(65, wait_after=False)
        self.robot.chasis.avanzar_recto(4)
        self.robot.navegacion.giro_absoluto(325)
        self.robot.chasis.avanzar_recto(40)

    def dejar_amarillos(self):
        """Alinea a la línea, avanza hacia la zona amarilla, retrocede y posiciona la garra delantera."""
        self.robot.navegacion.giro_absoluto(0)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor, 100, 1, distancia_extra_cm=5, distancia_inicial_cm=30, lado="izquierda", tiempo_acomodo_ms=0)
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)
        wait(300)
        self.robot.navegacion.giro_absoluto(90, max_potencia=80)
        self.robot.navegacion.avanzar_distancia_luego_color(
            self.sensor,
            distancia_ciega_cm=10,
            color_objetivo=Color.YELLOW,
            distancia_maxima_cm=40,
            velocidad_alta=400,
            velocidad_escaneo=240,
            distancia_extra_cm=6
        )
        self.robot.chasis.avanzar_recto(-20.5, velocidad=900)
        self.robot.navegacion.giro_relativo(-90, max_potencia=90, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(70)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(72)

    def dejar_pala_y_azules(self):
        """Sigue la línea transportando la carga y se posiciona hasta detectar la zona azul frente a la zona de inicio."""
        self._seguir_linea_distancia(
            distancia_cm=40,
            velocidad_max=100,
            lado="izquierda",
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            tiempo_acomodo_ms=100,
            encadenado=True
        )
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.BLUE, lado="izquierda", tiempo_acomodo_ms=0, distancia_cm=45, distancia_maxima_cm=55)