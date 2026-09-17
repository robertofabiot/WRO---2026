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

    # =========================================================================
    # MISIONES ATÓMICAS (RETO 1)
    # =========================================================================

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
        """Sigue la línea hasta el cruce, encara el parqueo y suelta ahí el cemento."""
        self.robot.navegacion.seguidor_linea_cruces(
            self.sensor,
            velocidad_max=100,
            cruces_objetivo=1,
            distancia_extra_cm=30,
            distancia_inicial_cm=10,
            tiempo_acomodo_ms=0
        )

        self.robot.navegacion.giro_relativo(90, max_potencia=85, encadenado=True)
        self.robot.chasis.avanzar_recto(-10, velocidad=1000)

        # La jaula sube bloqueando a propósito: si el robot arranca con ella a
        # medio subir se lleva el cuenco puesto y se pierden los 15 puntos.
        self.robot.garra_trasera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(15, velocidad=1000)

    def mover_pala_al_camino(self, distancia_pala_cm=20):
        """Recoge la pala con la jaula trasera y la deja tirada sobre el camino de salida.

        Aprovecha que la jaula queda libre apenas se suelta el cemento. La pala
        no viaja el resto del recorrido: queda apoyada donde el robot vuelve a
        pasar al ir a dejar los amarillos, así que después solo hay que empujarla.

        Arranca encarando el parqueo, que es donde termina dejar_cemento(), y
        devuelve el robot a la línea en la misma pose que espera agarrar_verdes().

        Argumentos:
            distancia_pala_cm: cuánto retrocede para calzarle la jaula a la pala.
                Es el número a tocar si la barrera de la jaula cae encima de la
                pala en vez de quedar por detrás de ella.
        """
        # Todas las maniobras del rodeo se miden contra el rumbo con el que
        # arranca la misión, así el error de un giro no se hereda al siguiente.
        rumbo_parqueo = self.robot.navegacion.rumbo()
        rumbo_linea = rumbo_parqueo - 90

        # La jaula baja recién cuando el robot ya está en posición: la pala es
        # plana y una jaula que baja antes la arrastra en vez de encerrarla.
        self.robot.navegacion.giro_absoluto(rumbo_parqueo - 100)
        self.robot.chasis.avanzar_recto(-distancia_pala_cm)
        self.robot.garra_trasera.ir_a_porcentaje(90)

        self.robot.chasis.avanzar_recto(15)
        self.robot.navegacion.giro_absoluto(rumbo_parqueo - 120)
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            distancia_total_cm=40,
            distancia_accion_cm=20,
            margen_cm=2,
            accion_callback=lambda: self.robot.garra_trasera.ir_a_porcentaje(
                0, wait_after=False
            ),
        )
        gc.collect()

        # Vuelta a la línea. Este bloque estaba metido al principio de
        # agarrar_verdes: vive acá porque es parte del rodeo de la pala.
        self.robot.navegacion.giro_absoluto(rumbo_parqueo - 165)
        self.robot.navegacion.avanzar_distancia_luego_color(
            self.sensor,
            distancia_ciega_cm=30,
            color_objetivo=Color.WHITE,
            distancia_maxima_cm=45,
            velocidad_escaneo=900,
            encadenado=True
        )
        self.robot.navegacion.avanzar_distancia_luego_color(
            self.sensor,
            distancia_ciega_cm=2,
            color_objetivo=Color.BLACK,
            distancia_maxima_cm=10,
            velocidad_escaneo=150,
            distancia_extra_cm=5
        )

        # Absoluto y no relativo: deja el robot sobre el rumbo de la línea sin
        # arrastrar el error acumulado de los cuatro giros del rodeo.
        self.robot.navegacion.giro_absoluto(rumbo_linea, max_potencia=85, encadenado=True)

    def agarrar_verdes(self):
        """Sigue la línea hasta ver verde, gira y retrocede atrapando los bloques verdes."""
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
        self.matriz_detectada = mosaico
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
        """Avanza contando líneas, gira hacia los bloques azules y los encierra con la jaula."""
        self.robot.chasis.avanzar_recto(55, encadenado=False)
        self.robot.navegacion.giro_relativo(90, max_potencia=80, encadenado=True)

        self.robot.garra_trasera.ir_a_porcentaje(95.0, velocidad=300, wait_after=False)
        self.robot.chasis.avanzar_recto(-11, velocidad=600, encadenado=True)

    def agarrar_pala(self):
        """Avanza hacia la pala, posiciona la garra/pinza y la sujeta firmemente."""
        self.robot.chasis.avanzar_recto(4)
        self.robot.navegacion.giro_absoluto(325)
        self.robot.garra_delantera.ir_a_porcentaje(65, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(65, wait_after=False)
        self.robot.chasis.avanzar_recto(52.5)

    def dejar_amarillos(self):
        """Descarga los bloques amarillos abriendo la pinza, los acomoda y sale de la sección."""
        self.robot.navegacion.giro_absoluto(10)
        self.robot.chasis.avanzar_recto(15)
        self.robot.garra_delantera.ir_a_porcentaje(60, wait_after=False)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(70)
        self.robot.garra_delantera.ir_a_porcentaje(70, wait_after=True)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor, 100, 1, distancia_extra_cm=5, distancia_inicial_cm=20,lado="izquierda", tiempo_acomodo_ms=0)
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
        """Retorna con la pala, descarga los bloques azules y se posiciona en la matriz."""
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