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
    """Misiones del recorrido secuencial y rutinas de armado de matriz."""

    def __init__(self, robot: Robot, sensor_frente):
        """
        Argumentos:
            robot: instancia de Robot con el chasis y los mecanismos armados.
            sensor_frente: ColorSensor delantero (el seguidor).
        """
        self.robot = robot
        self.sensor = sensor_frente
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

        self.robot.navegacion.giro_relativo(90, rueda_pivote="derecha", min_potencia=60, encadenado=True)
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

    def pruebas(self):
        self.robot.navegacion.desplazar_lateral(-1)
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
        self._seguir_linea_distancia(
            distancia_cm=50,
            velocidad_max=100,
            lado="derecha",
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            tiempo_acomodo_ms=50,
            encadenado=True
        )
        wait(100)

        self.robot.navegacion.giro_relativo(-90, max_potencia=85, encadenado=True)

        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            distancia_total_cm=28,
            distancia_accion_cm=13,
            margen_cm=2,
            accion_callback=lambda: self.robot.garra_trasera.ir_a_porcentaje(
                0, velocidad=900, wait_after=False
            ),
        )
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
            distancia_extra_cm=5
        )

        self.robot.navegacion.giro_relativo(90, max_potencia=85, encadenado=True)

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
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, 40, Color.BLACK, distancia_maxima_cm=60, velocidad_escaneo=400, distancia_extra_cm=4, encadenado=True)
        self.robot.navegacion.giro_relativo(90, max_potencia=80, encadenado=True)

        self.robot.garra_trasera.ir_a_porcentaje(95.0, velocidad=500, wait_after=False)
        self.robot.chasis.avanzar_recto(-17, velocidad=600, encadenado=True)

    def agarrar_pala(self):
        """Avanza hacia la pala, posiciona la garra/pinza y la sujeta firmemente."""
        self.robot.chasis.avanzar_recto(9)
        self.robot.navegacion.giro_absoluto(325)
        self.robot.garra_delantera.ir_a_porcentaje(75, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(65, wait_after=False)
        self.robot.chasis.avanzar_recto(39)
        self.robot.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)

    def dejar_amarillos(self):
        """Descarga los bloques amarillos abriendo la pinza, los acomoda y sale de la sección."""
        self.robot.navegacion.giro_absoluto(10)
        self.robot.chasis.avanzar_recto(15)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor, 100, 1, distancia_extra_cm=0, distancia_inicial_cm=20,lado="izquierda", tiempo_acomodo_ms=0)

        self.robot.garra_delantera.ir_a_porcentaje_pinza(0, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)
        wait(300) 
        self.robot.navegacion.giro_relativo(90, max_potencia=80)
        self.robot.navegacion.avanzar_distancia_luego_color(
            self.sensor,
            distancia_ciega_cm=4,
            color_objetivo=Color.YELLOW,
            distancia_maxima_cm=40,
            velocidad_alta=400,
            velocidad_escaneo=240,
            distancia_extra_cm=6
        )
        self.robot.chasis.avanzar_recto(-20.5, velocidad=900)
        self.robot.navegacion.giro_relativo(-90, max_potencia=90, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(70, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje(80)

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

    # =========================================================================
    # RUTINAS DE ARMA DE MATRIZ
    # =========================================================================

    def dejar_bloques_matriz(self):
        """Secuencia de entrega de bloques en la matriz (primera entrega)."""
        self._seguir_linea_distancia(
            distancia_cm=15,
            velocidad_max=65,
            lado="derecha",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )

        self.robot.garra_delantera.ir_a_porcentaje_pinza(54.8, velocidad=900, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje(65.2, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-14, velocidad=400)

        self.robot.garra_delantera.ir_a_porcentaje(76.5, velocidad=700, wait_after=True)
        self._seguir_linea_distancia(
            distancia_cm=13,
            velocidad_max=100,
            lado="derecha",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )

        self.robot.garra_delantera.cerrar_al_tope(velocidad=300, limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)

        self.robot.navegacion.seguidor_linea_color(
            self.sensor,
            velocidad_max=100,
            color_objetivo=Color.BLUE,
            lado="derecha",
            tiempo_acomodo_ms=0
        )
        wait(400)

        self.robot.navegacion.giro_relativo(-10, max_potencia=50, min_potencia=35)
        self.robot.chasis.avanzar_recto(12.5, velocidad=650, encadenado=True)

        self.robot.garra_delantera.ir_a_porcentaje(62.3, velocidad=700, wait_after=True)
        self.robot.garra_delantera.abrir(130, velocidad=1000)

        self.robot.chasis.avanzar_recto(-0.6, velocidad=650, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(1.8, velocidad=650, encadenado=True)

        self.robot.chasis.sacudir(iteraciones=4, potencia=50, tiempo_ms=70)

        self.robot.chasis.avanzar_recto(-1, velocidad=500)
        self.robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-17, velocidad=500)
        self.robot.navegacion.giro_relativo(180, max_potencia=80, min_potencia=70)

    def dejar_bloques_matriz2(self):
        """Secuencia auxiliar de entrega en matriz 2 (segunda entrega)."""
        self.robot.chasis.avanzar_recto(8, velocidad=900, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(
            self.sensor,
            lineas_objetivo=1,
            color_linea=Color.BLACK,
            velocidad=900,
            distancia_extra_cm=8.1,
            debug=False
        )

        self.robot.navegacion.giro_relativo(95, max_potencia=85, min_potencia=35, encadenado=True)
        self._seguir_linea_distancia(
            distancia_cm=7,
            velocidad_max=50,
            lado="derecha",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )

        self.robot.garra_delantera.ir_a_porcentaje_pinza(54.8, velocidad=900, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje(65.2, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-17, velocidad=400)

        self.robot.garra_delantera.ir_a_porcentaje(76.5, velocidad=700, wait_after=True)
        self._seguir_linea_distancia(
            distancia_cm=16,
            velocidad_max=100,
            lado="derecha",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )

        self.robot.garra_delantera.cerrar_al_tope(velocidad=300, limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)
        self.robot.navegacion.seguidor_linea_color(
            self.sensor,
            velocidad_max=100,
            color_objetivo=Color.BLUE,
            lado="derecha",
            tiempo_acomodo_ms=0
        )
        wait(200)

        self.robot.navegacion.giro_relativo(-11.5, max_potencia=50, min_potencia=35)
        self.robot.chasis.avanzar_recto(3, velocidad=650, encadenado=True)

        self.robot.garra_delantera.ir_a_porcentaje(62.3, velocidad=700, wait_after=True)
        self.robot.garra_delantera.abrir(130, velocidad=1000)

        self.robot.chasis.avanzar_recto(-0.6, velocidad=650, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(1.8, velocidad=750, encadenado=True)

        self.robot.chasis.sacudir(iteraciones=4, potencia=50, tiempo_ms=70)

        self.robot.chasis.avanzar_recto(-1, velocidad=900, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-29, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(180, max_potencia=85, min_potencia=35, encadenado=True)
        self.robot.chasis.avanzar_recto(-21, velocidad=900, encadenado=True)

    def ejecutar_matriz_2(self):
        """Ejecuta la secuencia de navegación y manipulación para el armado de la matriz 2."""
        print("Voltaje Hub:", self.robot.hub.battery.voltage(), "mV")
        print("Ejecutando recorrido de matriz 2...")
        gc.collect()

        # Primera fase: Bloques azules
        self.robot.garra_delantera.subir_al_tope()
        self.robot.garra_delantera.abrir_al_tope()

        self.robot.chasis.avanzar_recto(-9, velocidad=900)
        self.robot.navegacion.giro_relativo(90, max_potencia=65, min_potencia=45, encadenado=True)
        wait(200)

        self.robot.chasis.avanzar_recto(8, velocidad=900, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(
            self.sensor,
            lineas_objetivo=2,
            color_linea=Color.BLACK,
            velocidad=900,
            distancia_extra_cm=8.2,
            debug=False
        )
        gc.collect()
        wait(200)

        self.robot.navegacion.giro_relativo(-90, max_potencia=65, min_potencia=45, encadenado=True)
        self.robot.garra_delantera.abrir_al_tope(velocidad=900)

        self.robot.navegacion.avanzar_hasta_salir_negro(
            self.sensor,
            velocidad=900,
            umbral_reflexion=15,
            lecturas_salida=4,
            encadenado=True
        )

        self.robot.chasis.avanzar_recto(4.3, velocidad=900, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-20.7, velocidad=900)
        wait(200)

        self.robot.navegacion.giro_relativo(-90, max_potencia=75, min_potencia=35, encadenado=True)
        self._seguir_linea_distancia(
            distancia_cm=8,
            velocidad_max=60,
            lado="izquierda",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )
        gc.collect()
        wait(200)

        # Primera fase: Bloques amarillos
        self.robot.navegacion.giro_relativo(90, max_potencia=65, min_potencia=45, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(72.2, velocidad=700, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(33.3, velocidad=900, wait_after=True)
        wait(200)

        self.robot.chasis.avanzar_recto(12.5, velocidad=750, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(73.7, velocidad=700, wait_after=True)
        self.robot.garra_delantera.cerrar_al_tope(velocidad=300, limite_potencia=100)

        self.robot.chasis.avanzar_recto(-13, velocidad=550)
        wait(200)

        self.robot.navegacion.giro_relativo(90, max_potencia=65, min_potencia=45, encadenado=True)
        self._seguir_linea_distancia(
            distancia_cm=12,
            velocidad_max=90,
            lado="derecha",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )
        gc.collect()
        wait(200)

        self.robot.navegacion.giro_relativo(90, max_potencia=90, min_potencia=35, encadenado=True)
        self.dejar_bloques_matriz()
        gc.collect()

        # Segunda fase: Recolección y descarga
        self.robot.garra_delantera.ir_a_porcentaje_pinza(59.5, velocidad=900, wait_after=True)
        self._seguir_linea_distancia(
            distancia_cm=25,
            velocidad_max=80,
            lado="izquierda",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )
        gc.collect()
        wait(300)

        self.robot.navegacion.giro_relativo(-45, max_potencia=75, min_potencia=35, encadenado=True)
        self.robot.chasis.avanzar_recto(4.5, velocidad=500)
        wait(200)
        self.robot.navegacion.giro_relativo(45, max_potencia=80, min_potencia=35, encadenado=True)

        self._seguir_linea_distancia(
            distancia_cm=8,
            velocidad_max=60,
            lado="derecha",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )
        gc.collect()

        self.robot.chasis.avanzar_recto(20, velocidad=350)
        self.robot.garra_delantera.ir_a_porcentaje(85.0, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-25.5, velocidad=600)
        self.robot.navegacion.giro_relativo(-90, max_potencia=90, min_potencia=35, encadenado=True)

        self._seguir_linea_distancia(
            distancia_cm=13,
            velocidad_max=70,
            lado="izquierda",
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            tiempo_acomodo_ms=140,
            encadenado=True
        )
        gc.collect()
        wait(200)

        self.robot.navegacion.giro_relativo(90, max_potencia=90, min_potencia=35, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(75.1, velocidad=700, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(35.7, velocidad=900, wait_after=True)
        wait(300)

        self.robot.chasis.avanzar_recto(12.5, velocidad=750, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(77.9, velocidad=700, wait_after=True)
        self.robot.garra_delantera.cerrar_al_tope(velocidad=300, limite_potencia=80)
        self.robot.garra_delantera.ir_a_porcentaje(53.8, velocidad=700, wait_after=True)
        gc.collect()

        self.robot.chasis.avanzar_recto(16, velocidad=350)
        self.robot.garra_delantera.ir_a_porcentaje(73.7, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-38.5, velocidad=600)
        gc.collect()
        wait(300)

        self.robot.navegacion.giro_relativo(89.9, max_potencia=70, min_potencia=35, encadenado=True)
        self.robot.chasis.avanzar_recto(-18, velocidad=600)

        self.robot.garra_trasera.ir_a_porcentaje(97.8, velocidad=900, wait_after=False)
        gc.collect()

        self.dejar_bloques_matriz2()
        gc.collect()


       # Acomoda su elevador y garra delantera
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=1000, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(0, wait_after=False)

        # Intentar hacer el recorrido con la garra abajo para llevar los cementos
        # azules seria un dolor de cabeza, debido a que la friccion generada de
        # esta hace que el funcionamiento del robot sea distinto mayormente en
        # los giros.
        # self.robot.garra_trasera.bajar_al_tope()

        # Camino a los 4 primeros bloques azules
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=100, cruces_objetivo=0, distancia_extra_cm=39)
        self.robot.navegacion.giro_relativo(-90)

        # Agarra los 4 azules
        self.robot.garra_delantera.ir_a_porcentaje(75.9, velocidad=500, wait_after=False)  # bajar 510
        self.robot.chasis.avanzar_recto(24, velocidad=930)  # pedia 2000: el chasis de origen topaba en 930 y este topa en VELOCIDAD_RECTA

        # Camino al amarillo
        self.robot.chasis.avanzar_recto(-24)
        self.robot.navegacion.giro_relativo(-90)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=100, cruces_objetivo=0, distancia_extra_cm=21, lado="izquierda", tiempo_acomodo_ms=0)
        self.robot.garra_delantera.cerrar(490, velocidad=500, wait_after=False)

        self.robot.navegacion.giro_relativo(90)

        # Agarra un bloque amarillo
        self.robot.garra_delantera.cerrar_presionando(50)  # el original cerraba sin bloquear: agarra mientras avanza
        self.robot.chasis.avanzar_recto(10.8)
        self.robot.chasis.avanzar_recto(-10.8)

        # Camino al bloque verdecito jeje
        self.robot.navegacion.giro_relativo(90)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=100, cruces_objetivo=0, distancia_extra_cm=41, tiempo_acomodo_ms=50)
        self.robot.navegacion.giro_relativo(-90)
        self.robot.garra_delantera.abrir(100, wait_after=False)

        # Agarra el bloque verde alaverga
        self.robot.chasis.avanzar_recto(9.5)
        self.robot.garra_delantera.cerrar_al_tope(velocidad=1000)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.navegacion.giro_relativo(-80)
        self.robot.chasis.mover_motor_derecho(800, velocidad=1200)

        # Sigue la linea hacia el mosaico
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=100, cruces_objetivo=0, distancia_extra_cm=14, tiempo_acomodo_ms=0)

        # Abre la garra para dejar los 4 bloques azules
        self.robot.garra_delantera.abrir(300, wait_after=False)

        # Levanta la garra para meter los 6 bloques
        self.robot.garra_delantera.ir_a_porcentaje(16.4, velocidad=900, wait_after=False)  # subir 400
        self.robot.chasis.avanzar_recto(-15)
        self.robot.garra_delantera.ir_a_porcentaje(72.9, velocidad=600)                    # bajar 380
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=90, cruces_objetivo=0, distancia_extra_cm=25, tiempo_acomodo_ms=0)

        # Aprieta la garra y agarra los 6 bloques
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=600, wait_after=False)  # subir 300

        self.robot.chasis.mover_motor_derecho(30, velocidad=1200)
        self.robot.chasis.mover_motor_izquierdo(30, velocidad=1200)

        # Se mete a la matriz a dejar los 6 bloques
        self.robot.chasis.avanzar_recto(25, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje(67.0, velocidad=600)                    # bajar 260

        # Tira los bloques a la matriz y acomoda los bloques
        self.robot.garra_delantera.abrir(160, velocidad=2000)

        # Avance y retroceso para acomodar los bloques
        self.robot.chasis.avanzar_recto(2)
        self.robot.chasis.avanzar_recto(-4)

        # Acomodos que gira a la izquierda y derecha para terminar de acomodar
        # los 6 bloques
        self.robot.chasis.acomodar_estable(iteraciones=3, tiempo_ms=60)
        self.robot.garra_delantera.abrir(50, wait_after=False)

        # Termina de acomodar
        self.robot.garra_delantera.ir_a_porcentaje(81.8, velocidad=600)                    # bajar 100
        self.robot.chasis.acomodar_estable(iteraciones=3, tiempo_ms=40)
        gc.collect()

        # Levanta la garra para que no choque con las barreras alrededor de la matriz
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=1000, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(0, wait_after=False)

        # Camino al resto de los bloques
        self.robot.chasis.avanzar_recto(-12)
        self.robot.navegacion.giro_relativo(-178)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=100, cruces_objetivo=0, distancia_extra_cm=39, lado="derecha", tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_relativo(-90)
        self.robot.chasis.avanzar_recto(9.5)
        self.robot.navegacion.giro_relativo(90)

        # Se mete a agarrar los dos bloques azules
        self.robot.garra_delantera.ir_a_porcentaje(73.7, velocidad=300, wait_after=False)  # bajar 495
        self.robot.chasis.avanzar_recto(31)
        self.robot.chasis.avanzar_recto(-31)

        # Camino a los 3 bloques amarillos
        self.robot.navegacion.giro_relativo(-90)

        self.robot.chasis.avanzar_recto(16)
        self.robot.navegacion.giro_relativo(90)
        self.robot.garra_delantera.abrir(110, wait_after=False)
        self.robot.chasis.avanzar_recto(10)

        # Agarra el primer bloque amarillo
        self.robot.garra_delantera.cerrar_al_tope(velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje(29.0, velocidad=600)                    # subir 300
        self.robot.garra_delantera.ir_a_porcentaje(76.6, velocidad=500, wait_after=False)  # bajar 320
        self.robot.chasis.avanzar_recto(16)
        self.robot.chasis.avanzar_recto(-26)

        # Camino al bloque verde
        self.robot.navegacion.giro_relativo(90)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=100, cruces_objetivo=0, distancia_extra_cm=30, tiempo_acomodo_ms=100)
        gc.collect()

        # Se mete a agarrar el ultimo bloque verde
        self.robot.navegacion.giro_relativo(-90)
        self.robot.garra_delantera.abrir(110, wait_after=False)

        # Agarra el SEGUNDO bloque verde alaverga
        self.robot.chasis.avanzar_recto(13.5)
        self.robot.garra_delantera.cerrar_presionando(100)  # el original cerraba sin bloquear
        self.robot.chasis.avanzar_recto(-9.5)
        self.robot.navegacion.giro_relativo(-120)
        self.robot.chasis.mover_motor_derecho(500, velocidad=1200)

        # Sigue la linea hacia el mosaico
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=100, cruces_objetivo=0, distancia_extra_cm=14, tiempo_acomodo_ms=0)

        # Abre la garra para dejar los 2 amarillos, 3 azules y un verde
        self.robot.garra_delantera.abrir(300, wait_after=False)

        # Levanta la garra para meter los 6 bloques
        self.robot.garra_delantera.ir_a_porcentaje(17.1, velocidad=900, wait_after=False)  # subir 400
        self.robot.chasis.avanzar_recto(-15)
        self.robot.garra_delantera.ir_a_porcentaje(73.7, velocidad=600)                    # bajar 380
        self.robot.navegacion.seguidor_linea_cruces(self.sensor.color, velocidad_max=90, cruces_objetivo=0, distancia_extra_cm=25, tiempo_acomodo_ms=0)

        # Aprieta la garra y agarra los 6 bloques
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(29.0, velocidad=600)                    # subir 300

        self.robot.chasis.mover_motor_derecho(30, velocidad=1200)
        self.robot.chasis.mover_motor_izquierdo(30, velocidad=1200)

        # Se mete a la matriz a dejar los 6 bloques
        self.robot.chasis.avanzar_recto(13)
        self.robot.garra_delantera.ir_a_porcentaje(67.7, velocidad=600)                    # bajar 260

        # Tira los bloques a la matriz y acomoda los bloques
        self.robot.garra_delantera.abrir(160, velocidad=2000)

        # Avance y retroceso para acomodar los bloques
        self.robot.chasis.avanzar_recto(2)
        self.robot.chasis.avanzar_recto(-4)

        # Acomodos que gira a la izquierda y derecha para terminar de acomodar
        # los 6 bloques
        self.robot.chasis.acomodar_estable(iteraciones=3, potencia=50, tiempo_ms=60)
        self.robot.garra_delantera.abrir(50, wait_after=False)

        # Termina de acomodar
        self.robot.garra_delantera.ir_a_porcentaje(82.6, velocidad=600)                    # bajar 100
        self.robot.chasis.acomodar_estable(iteraciones=3, tiempo_ms=40)
        gc.collect()

    # =========================================================================
    # CORRIDA COMPLETA
    # =========================================================================

    def recorrido_completo(self, forzar_matriz=None):
        """Encadena todas las misiones del reto y ejecuta la matriz correspondiente.

        Argumentos:
            forzar_matriz: Si se especifica un número (ej. 2), omite el resultado
                del escáner y fuerza esa matriz. Si es None, ejecuta la detectada.
        """
        print("=== INICIANDO RECORRIDO COMPLETO (RETO 1 + MATRIZ) ===")
        self.agarrar_cemento()
        self.dejar_llana()
        self.dejar_cemento()
        self.agarrar_verdes()
        matriz = self.escanear_mosaico()
        self.dejar_verdes()
        self.agarrar_amarillos()
        self.agarrar_azules()
        self.agarrar_pala()
        self.dejar_amarillos()
        self.dejar_pala_y_azules()
        gc.collect()

        if forzar_matriz is not None:
            matriz = forzar_matriz

        print("Matriz seleccionada:", matriz)
        if matriz == 2 or True:
            print("Iniciando recorrido de la matriz 2...")
            self.ejecutar_matriz_2()
        else:
            print("Matriz Predeterminada", matriz)
            self.ejecutar_matriz_2()
