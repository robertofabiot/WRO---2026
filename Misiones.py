"""Secuencia de misiones autónomas y armado de matriz para WRO 2026.

Estructura modular dividida en misiones atómicas individuales que pueden
ejecutarse por separado para calibración y pruebas, o encadenadas en recorrido_completo().
"""

from pybricks.parameters import Color
from pybricks.tools import wait
import gc
import config
from robot import Robot


class Misiones:
    """Misiones del recorrido secuencial del Reto 1."""

    def __init__(self, robot: Robot, sensor_frente, armador=None):
        """
        Argumentos:
            robot: instancia de Robot con el chasis y los mecanismos armados.
            sensor_frente: ColorSensor delantero (el seguidor).
            armador: instancia opcional de ArmadorMosaicos.
        """
        self.robot = robot
        self.sensor = sensor_frente
        self.armador = armador
        self.matriz_detectada = None

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
        color_principal = self.sensor.color()
        if color_principal not in config.MOSAICOS:
            return -1

        decision = config.MOSAICOS[color_principal]

        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_verificacion_cm)
            wait(50)
            color_anterior = self.sensor.color()
            self.robot.chasis.avanzar_recto(-distancia_verificacion_cm)

            if color_anterior not in decision:
                return -1
            return decision[color_anterior]
        return decision

    def agarrar_cemento(self):
        """Salida inicial, seguidor de línea y captura del cemento con la jaula trasera."""
        self.robot.garra_trasera.establecer_cero()
        self.robot.garra_delantera.establecer_cero()
        self.robot.garra_delantera.establecer_cero_pinza()

        self.robot.navegacion.giro_relativo(90, rueda_pivote="derecha", max_potencia=100, min_potencia=100, encadenado=True)
        gc.collect()

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
        self.robot.navegacion.avanzar_contando_lineas(
            self.sensor,
            lineas_objetivo=2,
            color_linea=Color.BLACK,
            velocidad=900,
            velocidad_lenta=800,
            debug=False
        )
        gc.collect()

        self.robot.chasis.giro_de_arco(
            radio_cm=13,
            angulo_deg=19,
            lado="derecha",
            encadenado=False
        )

    def dejar_cemento(self):
        """Sigue la línea, gira y descarga el cemento levantando la jaula en recorrido."""
        self.robot.navegacion.seguidor_linea_cruces(self.sensor, 100, 1, distancia_extra_cm=20, distancia_inicial_cm=15, tiempo_acomodo_ms=0)

        self.robot.navegacion.giro_relativo(90, max_potencia=85, encadenado=True)
        self.robot.chasis.avanzar_recto(-20, 1000)
        self.robot.garra_trasera.ir_a_porcentaje(0, wait_after=False)

        self.robot.chasis.avanzar_recto(25, 1000)
        self.robot.navegacion.giro_relativo(-225)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.garra_trasera.ir_a_porcentaje(90, wait_after=False)
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            distancia_total_cm=40,
            distancia_accion_cm=20,
            accion_callback = lambda: self.robot.garra_trasera.ir_a_porcentaje(0, wait_after=False),
            margen_cm=2,
        )

    def agarrar_verdes(self):
        """Sigue la línea hasta ver verde, gira y retrocede atrapando los bloques verdes."""
        self.robot.navegacion.giro_relativo(-45)
        self.robot.navegacion.avanzar_distancia_luego_color(
            self.sensor,
            distancia_ciega_cm=0,
            color_objetivo=Color.WHITE,
            distancia_maxima_cm=10,
            velocidad_escaneo=900,
            encadenado=True
        )

        self.robot.navegacion.avanzar_distancia_luego_color(
            self.sensor,
            distancia_ciega_cm=2,
            color_objetivo=Color.BLACK,
            distancia_maxima_cm=10,
            velocidad_escaneo=150,
            distancia_extra_cm=5,
            accion_callback= lambda: None
        )

        self.robot.navegacion.giro_relativo(90, max_potencia=85, encadenado=True)

        self.robot.navegacion.seguidor_linea_color(
            self.sensor,
            velocidad_max=95,
            color_objetivo=Color.GREEN,
            lado="derecha",
            tiempo_acomodo_ms=0,
            distancia_cm=45,
            distancia_maxima_cm=55,
            encadenado=True,
        )

        self.robot.chasis.avanzar_recto(-7, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(-183, max_potencia=90, encadenado=True)

        self.robot.garra_trasera.ir_a_porcentaje(95.0, velocidad=250, wait_after=False)
        self.robot.chasis.avanzar_recto(-14, velocidad=400, encadenado=True)

    def escanear_mosaico(self, distancia_verificacion_cm=5):
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

        self.robot.navegacion.giro_absoluto(0)
        self.robot.chasis.avanzar_recto(20, velocidad=700)

        mosaico = self._identificar_combinacion(distancia_verificacion_cm)
        self.matriz_detectada = mosaico
        print("Mosaico detectado:", mosaico)
        return mosaico

    def detectar_mosaico(self, distancia_verificacion_cm=5):
        """Alias para compatibilidad con ArmadorMosaicos."""
        return self.escanear_mosaico(distancia_verificacion_cm)

    def escanear_matriz(self, distancia_verificacion_cm=5):
        """Alias para compatibilidad."""
        return self.escanear_mosaico(distancia_verificacion_cm)

    def dejar_verdes(self):
        """Sale de la matriz en reversa, gira 180° y deposita los bloques verdes."""
        self.robot.chasis.avanzar_recto(-37.5, velocidad=900)
        self.robot.navegacion.giro_relativo(-180, max_potencia=90, encadenado=True)
        gc.collect()

        self.robot.chasis.avanzar_recto(-16, velocidad=900, encadenado=True)
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=350, wait_after=False)

    def agarrar_amarillos(self):
        """Sigue la línea y navega el pasillo en zigzag recolectando los bloques amarillos."""
        self.robot.navegacion.seguidor_linea_color(
            self.sensor,
            100,
            Color.GREEN,
            tiempo_acomodo_ms=100,
            distancia_cm=70
        )
        self.robot.chasis.avanzar_recto(18, velocidad=900, encadenado=False)

        self.robot.navegacion.giro_relativo(-90, max_potencia=90, encadenado=True)
        self.robot.chasis.avanzar_recto(25, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(-90, max_potencia=80, encadenado=False)
        self.robot.chasis.avanzar_recto(20, velocidad=900, encadenado=False)
        self.robot.navegacion.giro_relativo(-90, max_potencia=80, encadenado=False)
        wait(100)

    def agarrar_azules(self):
        """Avanza contando líneas, gira hacia los bloques azules y los encierra con la jaula."""
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, 35, Color.BLACK, distancia_maxima_cm=50, velocidad_escaneo=150, distancia_extra_cm=4, encadenado=True)
        self.robot.navegacion.giro_relativo(90, max_potencia=80, encadenado=True)

        self.robot.garra_trasera.ir_a_porcentaje(95.0, velocidad=500, wait_after=False)
        self.robot.chasis.avanzar_recto(-13, velocidad=400, encadenado=True)

    def agarrar_pala(self):
        """Avanza hacia la pala, posiciona la garra/pinza y la sujeta firmemente."""
        self.robot.navegacion.giro_relativo(-25)
        self.robot.chasis.avanzar_recto(35)
        self.robot.navegacion.giro_relativo(25)

    def dejar_amarillos(self):
        """Descarga los bloques amarillos abriendo la pinza, los acomoda y sale de la sección."""
        self.robot.navegacion.seguidor_linea_cruces(self.sensor, 100, 1, distancia_extra_cm=0, distancia_inicial_cm=20,lado="izquierda", tiempo_acomodo_ms=0)

        self.robot.garra_delantera.ir_a_porcentaje_pinza(0, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)
        wait(300) 
        self.robot.navegacion.giro_relativo(90, max_potencia=80)
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
        self.robot.garra_delantera.ir_a_porcentaje_pinza(70, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje(70, wait_after=False)
        wait(200)

    def dejar_pala_y_azules(self):
        """Retorna con la pala, descarga los bloques azules y se posiciona en la matriz."""
        self._seguir_linea_distancia(
            distancia_cm=65,
            velocidad_max=100,
            lado="izquierda",
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            tiempo_acomodo_ms=100,
            encadenado=True
        )
        gc.collect()