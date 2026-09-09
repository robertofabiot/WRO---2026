"""Recorrido de competencia del otro equipo (prueba_reto_1 + matriz_2) adaptado a las funciones propias de MIO.

Conserva con exactitud la secuencia, distancias, ángulos, tiempos de espera,
llamadas a gc.collect() y lógica de control del archivo prueba_reto_1 y matriz_2,
pero ejecutados íntegramente mediante las funciones y subsistemas de MIO:
- Chasis: avanzar_recto, giro_de_arco, avanzar_y_accionar_en_recorrido, sacudir
- Navegacion: giro_relativo, seguidor_linea_cruces, seguidor_linea_color,
  avanzar_contando_lineas, avanzar_distancia_luego_color, avanzar_hasta_salir_negro
- Mecanismos: garra_trasera (ir_a_porcentaje), garra_delantera (ir_a_porcentaje,
  cerrar_al_tope, abrir_al_tope, ir_a_porcentaje_pinza)
"""

from pybricks.parameters import Color
from pybricks.tools import wait
import gc
import config
from robot import Robot


class Misiones:
    """Las misiones del recorrido secuencial del reto 1 y la matriz 2."""

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
        """Sigue el borde de la línea una distancia fija utilizando seguidor_linea_cruces.

        Al pasar cruces_objetivo=0 se saltea el conteo y recorre exactamente
        distancia_extra_cm (= distancia_cm).

        Argumentos:
            distancia_cm: centímetros a recorrer sobre la línea.
            velocidad_max: duty-cycle de crucero, 0-100.
            lado: "derecha" o "izquierda", qué lado del sensor sigue la línea.
            kp, kd: ganancias del PD sobre la reflexión.
            k_freno: reducción de velocidad por error de reflexión.
            tiempo_acomodo_ms: tiempo de arranque suave para engancharse.
            encadenado: True frena con el micro-freno pasivo.
        """
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
        """Lee el mosaico que toca armar y devuelve su número.

        La tabla está en config.MOSAICOS. El verde aparece en dos mosaicos
        distintos, así que cuando sale verde hay que avanzar a leer la celda
        de al lado para desempatar, y después volver.

        Argumentos:
            distancia_verificacion_cm: centímetros a avanzar para leer la segunda
                celda cuando la primera sale verde.

        Devuelve el número de mosaico (1 a 5), o -1 si la lectura no cierra.
        """
        color_principal = self.sensor.color()
        if color_principal not in config.MOSAICOS:
            return -1

        decision = config.MOSAICOS[color_principal]

        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_verificacion_cm)
            wait(50)  # Micro-pausa para que el sensor lea sin vibraciones del motor
            color_anterior = self.sensor.color()

            self.robot.chasis.avanzar_recto(-distancia_verificacion_cm)

            if color_anterior not in decision:
                return -1
            return decision[color_anterior]

        return decision

    def detectar_mosaico(self, distancia_verificacion_cm=5):
        """Escanea la matriz de mosaicos y guarda el resultado."""
        mosaico = self._identificar_combinacion(distancia_verificacion_cm)
        self.matriz_detectada = mosaico
        return mosaico

    def escanear_matriz(self, distancia_verificacion_cm=5):
        """Alias de detectar_mosaico."""
        return self.detectar_mosaico(distancia_verificacion_cm)


    # =========================================================================
    # SECCIÓN 1: Giro de salida y seguidor hasta el cemento
    # =========================================================================

    def seccion_1_salida_y_cemento(self):
        """Salida en arco, seguidor hasta el cemento, agarre y empuje de la llana."""
        # 1. Calibrar las garras contra sus topes físicos al inicio
        self.robot.garra_trasera.establecer_cero()
        self.robot.garra_delantera.establecer_cero()
        self.robot.garra_delantera.establecer_cero_pinza()

        # 2. Giro de arco inicial de salida
        self.robot.navegacion.giro_relativo(90, rueda_pivote="derecha", min_potencia=60, encadenado=True)
        gc.collect()

        # 3. Seguidor de línea de 79 cm
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

        # 4. Giro de -90° para encarar el cemento
        self.robot.navegacion.giro_relativo(-90, max_potencia=100, encadenado=True)

        # 5. Bajar jaula trasera para tomar el cemento (-169° = 94.4%) y retroceder
        self.robot.garra_trasera.ir_a_porcentaje(94.4, velocidad=600, wait_after=False)
        self.robot.chasis.avanzar_recto(-12, velocidad=1000, encadenado=False)

        # 6. Pequeño avance y giro hacia la llana
        self.robot.chasis.avanzar_recto(8, velocidad=1000, encadenado=False)
        self.robot.navegacion.giro_relativo(70, max_potencia=100, encadenado=True)

        # 7. Retroceso para dejar la llana y regresar cruzando 2 líneas
        self.robot.chasis.avanzar_recto(-32, velocidad=1000)

        # Escape inicial de 8 cm y conteo de 2 líneas negras
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

        # 8. Giro de arco para posicionarse sobre la línea
        self.robot.chasis.giro_de_arco(
            radio_cm=13,
            angulo_deg=19,
            lado="derecha",
            encadenado=False
        )

    # =========================================================================
    # SECCIÓN 2: Dejar cemento y tomar los cementos verdes
    # =========================================================================

    def seccion_2_dejar_cemento_y_tomar_verdes(self):
        """Seguidor hasta la zona de cemento, depósito coordinado y agarre de verdes."""
        # 1. Seguidor de línea 50 cm
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

        # 2. Giro de -90°
        self.robot.navegacion.giro_relativo(-90, max_potencia=85, encadenado=True)

        # 3. Avanzar 39.5 cm levantando la jaula a los 13 cm de recorrido
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            distancia_total_cm=28,
            distancia_accion_cm=13,
            margen_cm=2,
            accion_callback=lambda: self.robot.garra_trasera.ir_a_porcentaje(
                0, velocidad=900, wait_after=False
            ),
        )
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, distancia_ciega_cm=0, color_objetivo=Color.WHITE, distancia_maxima_cm=10, velocidad_escaneo=900, encadenado=True)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, distancia_ciega_cm=2, color_objetivo=Color.BLACK, distancia_maxima_cm=10, velocidad_escaneo=150, distancia_extra_cm=5)

        # 4. Giro de 90°
        self.robot.navegacion.giro_relativo(90, max_potencia=85, encadenado=True)

        # 5. Seguir línea hasta ver verde
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

        # 6. Reacomodo y giro de -188°
        self.robot.chasis.avanzar_recto(-7, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(-183, max_potencia=90, encadenado=True)

        # 7. Bajar jaula (-170° = 95.0%) y retroceder sobre los cementos verdes
        self.robot.garra_trasera.ir_a_porcentaje(95.0, velocidad=250, wait_after=False)
        self.robot.chasis.avanzar_recto(-20, velocidad=700, encadenado=True)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=150, potencia=80)

    # =========================================================================
    # SECCIÓN 3: Seguir hasta matriz, escanear y dejar cementos verdes
    # =========================================================================

    def seccion_3_escanear_matriz_y_dejar_verdes(self):
        """Llegada a la matriz, escaneo de color, salida y depósito de verdes."""
        # 1. Seguir línea por la izquierda hasta ver verde
        self.robot.navegacion.seguidor_linea_color(
            self.sensor,
            velocidad_max=100,
            color_objetivo=Color.GREEN,
            lado="izquierda",
            tiempo_acomodo_ms=0,
            distancia_cm=70,
            encadenado=True
        )

        # 2. Entrar a escanear la matriz
        self.robot.navegacion.giro_absoluto(0)
        self.robot.chasis.avanzar_recto(20, velocidad=700)

        # 3. Escaneo del mosaico con la lógica de MIO SIN CAMBIO
        mosaico = self._identificar_combinacion(distancia_verificacion_cm=5)
        self.matriz_detectada = mosaico
        print("Mosaico detectado:", mosaico)

        # 4. Reacomodo y salida de la matriz
        self.robot.chasis.avanzar_recto(-37.5, velocidad=900)
        self.robot.navegacion.giro_relativo(-180, max_potencia=90, encadenado=True)
        gc.collect()

        # 5. Dejar los cementos verdes
        self.robot.chasis.avanzar_recto(-16, velocidad=900, encadenado=True)
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=350, wait_after=False)

        return mosaico

    # =========================================================================
    # SECCIÓN 4: Ir por amarillos y tomar los azules
    # =========================================================================

    def seccion_4_amarillos_y_azules(self):
        """Navegación al pasillo de amarillos y agarre de los bloques azules."""
        # 1. Seguidor 48 cm y avance
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, tiempo_acomodo_ms=100, distancia_cm=70)
        self.robot.chasis.avanzar_recto(18, velocidad=900, encadenado=False)

        # 2. Zigzag entre líneas
        self.robot.navegacion.giro_relativo(-90, max_potencia=90, encadenado=True)
        self.robot.chasis.avanzar_recto(25, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(-90, max_potencia=80, encadenado=False)
        self.robot.chasis.avanzar_recto(20, velocidad=900, encadenado=False)
        self.robot.navegacion.giro_relativo(-90, max_potencia=80, encadenado=False)
        wait(100)

        # 3. Ir por los azules cruzando 2 líneas con 6.3 cm de distancia extra
        self.robot.navegacion.avanzar_contando_lineas(
            self.sensor,
            lineas_objetivo=3,
            color_linea=Color.BLACK,
            velocidad=700,
            distancia_extra_cm=6.3,
            debug=False
        )
        self.robot.navegacion.giro_relativo(90, max_potencia=80, encadenado=True)

        # 4. Bajar jaula sobre los azules (-170° = 95.0%) y retroceder
        self.robot.garra_trasera.ir_a_porcentaje(95.0, velocidad=250, wait_after=False)
        self.robot.chasis.avanzar_recto(-22, velocidad=700, encadenado=True)

    # =========================================================================
    # SECCIÓN 5: Ir por la pala y dejar los amarillos
    # =========================================================================

    def seccion_5_tomar_pala_y_dejar_amarillos(self):
        """Captura de la pala con elevador y pinza, y depósito de los amarillos."""
        # 1. Avance y giro hacia la pala
        self.robot.chasis.avanzar_recto(12, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(-38, max_potencia=90, encadenado=True)

        # 2. Posicionar garra delantera (245° / 353° = 69.4%) y abrir pinza (200° / 420° = 47.6%)
        self.robot.garra_delantera.ir_a_porcentaje(69.4, velocidad=700, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(33.3, velocidad=1000, wait_after=True)

        # 3. Cruzar 2 líneas con 13 cm extra hasta embocar la pala
        self.robot.navegacion.avanzar_contando_lineas(
            self.sensor,
            lineas_objetivo=2,
            color_linea=Color.BLACK,
            velocidad=500,
            distancia_extra_cm=13,
            debug=False
        )

        # 4. Cerrar la pinza sobre la pala
        self.robot.garra_delantera.cerrar_al_tope(velocidad=300, limite_potencia=100)

        # 5. Retroceso y giro para ir a dejar amarillos
        self.robot.chasis.avanzar_recto(-2.5, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(36, max_potencia=80, encadenado=True)

        # 6. Avance de 48 cm y descarga de amarillos abriendo la pinza
        self.robot.chasis.avanzar_recto(48, velocidad=900)
        self.robot.garra_delantera.abrir_al_tope(velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=False)
        self.robot.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=50)

        # 7. Giro de 90° y avance híbrido buscando color amarillo
        self.robot.navegacion.giro_relativo(90, max_potencia=80, encadenado=True)
        wait(200)
        self.robot.navegacion.avanzar_distancia_luego_color(
            self.sensor,
            distancia_ciega_cm=4,
            color_objetivo=Color.YELLOW,
            distancia_maxima_cm=40,
            velocidad_alta=400,
            velocidad_escaneo=240
        )
        self.robot.chasis.avanzar_recto(8, velocidad=900)

        # 8. Salir de la sección amarilla
        wait(200)
        self.robot.chasis.avanzar_recto(-22.5, velocidad=900)
        self.robot.navegacion.giro_relativo(-92, max_potencia=90, encadenado=True)

    # =========================================================================
    # SECCIÓN 6: Retorno con la pala y acomodo final de reto 1
    # =========================================================================

    def seccion_6_retorno_pala_y_acomodo(self):
        """Retorno con la pala, subida coordinada de jaula y reacomodo en la matriz."""
        # 1. Posicionar garra delantera (69.4%) y abrir pinza
        self.robot.garra_delantera.ir_a_porcentaje_pinza(33.3, velocidad=1000, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje(69.4, velocidad=700, wait_after=False)
        self.robot.chasis.avanzar_recto(-2, velocidad=900)

        # 2. Seguir línea 85 cm levantando la jaula trasera a los 65 cm
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
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=350, wait_after=False)
        self._seguir_linea_distancia(
            distancia_cm=20,
            velocidad_max=100,
            lado="izquierda",
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            encadenado=True
        )

        # 3. Acomodo final en esquina de la matriz
        self.robot.navegacion.giro_relativo(-40, max_potencia=54, min_potencia=34)
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=False)
        self.robot.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=50)
        self.robot.navegacion.giro_relativo(40, max_potencia=54, min_potencia=34)
        gc.collect()

    # =========================================================================
    # AUXILIARES DE MATRIZ: DEJAR BLOQUES (matriz.py)
    # =========================================================================

    def dejar_bloques_matriz(self):
        """Secuencia de entrega de bloques en la matriz (primera entrega)."""
        # 1. Seguidor de 15 cm
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

        # 2. Abrir pinza (230° / 420° = 54.8%) y posicionar elevador (230° / 353° = 65.2%)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(54.8, velocidad=900, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje(65.2, velocidad=700, wait_after=True)

        # 3. Retroceso
        self.robot.chasis.avanzar_recto(-14, velocidad=400)

        # 4. Ajustar elevador a 270° (76.5%) y seguir línea 13 cm
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

        # 5. Apretar pinza y subir elevador a 100° (28.3%)
        self.robot.garra_delantera.cerrar_al_tope(velocidad=300, limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)

        # 6. Seguir línea hasta ver azul
        self.robot.navegacion.seguidor_linea_color(
            self.sensor,
            velocidad_max=100,
            color_objetivo=Color.BLUE,
            lado="derecha",
            tiempo_acomodo_ms=0
        )
        wait(400)

        # 7. Giro corto de -10° y avance 12.5 cm
        self.robot.navegacion.giro_relativo(-10, max_potencia=50, min_potencia=35)
        self.robot.chasis.avanzar_recto(12.5, velocidad=650, encadenado=True)

        # 8. Bajar elevador a 220° (62.3%) y apertura rápida de garra (130°)
        self.robot.garra_delantera.ir_a_porcentaje(62.3, velocidad=700, wait_after=True)
        self.robot.garra_delantera.abrir(130, velocidad=1000)

        # 9. Micro-retroceso, bajar elevador a 290° (82.2%) y avance 1.8 cm
        self.robot.chasis.avanzar_recto(-0.6, velocidad=650, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(1.8, velocidad=650, encadenado=True)

        # 10. Sacudida del robot para asentar los bloques
        self.robot.chasis.sacudir(iteraciones=4, potencia=50, tiempo_ms=70)

        # 11. Salida y giro de 180°
        self.robot.chasis.avanzar_recto(-1, velocidad=500)
        self.robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-17, velocidad=500)
        self.robot.navegacion.giro_relativo(180, max_potencia=80, min_potencia=70)

    def dejar_bloques_matriz2(self):
        """Secuencia auxiliar de entrega en matriz 2 (segunda entrega)."""
        # 1. Cruzar 1 línea con 8 cm de escape y 8.1 cm extra
        self.robot.chasis.avanzar_recto(8, velocidad=900, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(
            self.sensor,
            lineas_objetivo=1,
            color_linea=Color.BLACK,
            velocidad=900,
            distancia_extra_cm=8.1,
            debug=False
        )

        # 2. Giro de 95° y seguidor de 7 cm
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

        # 3. Abrir pinza (230° = 54.8%) y acomodar elevador (230° = 65.2%)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(54.8, velocidad=900, wait_after=True)
        self.robot.garra_delantera.ir_a_porcentaje(65.2, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-17, velocidad=400)

        # 4. Ajustar elevador a 270° (76.5%) y seguidor 16 cm
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

        # 5. Apretar pinza, elevador a 100° (28.3%) y seguir hasta azul
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

        # 6. Giro corto de -11.5° y avance de 3 cm
        self.robot.navegacion.giro_relativo(-11.5, max_potencia=50, min_potencia=35)
        self.robot.chasis.avanzar_recto(3, velocidad=650, encadenado=True)

        # 7. Bajar elevador a 220° (62.3%) y apertura rápida de garra (130°)
        self.robot.garra_delantera.ir_a_porcentaje(62.3, velocidad=700, wait_after=True)
        self.robot.garra_delantera.abrir(130, velocidad=1000)

        # 8. Micro-retroceso, elevador a 290° (82.2%) y avance 1.8 cm
        self.robot.chasis.avanzar_recto(-0.6, velocidad=650, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(1.8, velocidad=750, encadenado=True)

        # 9. Sacudida
        self.robot.chasis.sacudir(iteraciones=4, potencia=50, tiempo_ms=70)

        # 10. Salida completa
        self.robot.chasis.avanzar_recto(-1, velocidad=900, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=True)
        self.robot.chasis.avanzar_recto(-29, velocidad=900, encadenado=True)
        self.robot.navegacion.giro_relativo(180, max_potencia=85, min_potencia=35, encadenado=True)
        self.robot.chasis.avanzar_recto(-21, velocidad=900, encadenado=True)

    # =========================================================================
    # RECORRIDO COMPLETO DE LA MATRIZ 2 (matriz_2.py)
    # =========================================================================

    def ejecutar_matriz_2(self):
        """Ejecuta la secuencia completa de navegación y manipulación de la matriz 2."""
        print("Voltaje Hub:", self.robot.hub.battery.voltage(), "mV")
        print("Ejecutando recorrido de matriz 2...")
        gc.collect()

        # ==========================================
        # PRIMERA PARTE DE LA MATRIZ: BLOQUES AZULES
        # ==========================================
        self.robot.garra_delantera.subir_al_tope()
        self.robot.garra_delantera.abrir_al_tope()

        self.robot.chasis.avanzar_recto(-9, velocidad=900)
        self.robot.navegacion.giro_relativo(90, max_potencia=65, min_potencia=45, encadenado=True)
        wait(200)

        # Cruzar 2 líneas con escape de 8 cm y 8.2 cm extra tras la segunda línea
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

        # Avanzar hasta salir de la línea negra
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

        # ============================================
        # PRIMERA PARTE DE LA MATRIZ: BLOQUES AMARILLOS
        # ============================================
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

        # Rutina de entrega 1
        self.dejar_bloques_matriz()
        gc.collect()

        # ==========================================
        # SEGUNDA PARTE DE LA MATRIZ: RECOLECCIÓN Y DESCARGA
        # ==========================================
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

        # Bajar jaula trasera (-175° = 97.8%)
        self.robot.garra_trasera.ir_a_porcentaje(97.8, velocidad=900, wait_after=False)
        gc.collect()

        # Rutina de entrega 2
        self.dejar_bloques_matriz2()
        gc.collect()

    # =========================================================================
    # CORRIDA COMPLETA
    # =========================================================================

    def recorrido_completo(self, forzar_matriz=None):
        """Encadena todo el recorrido de prueba_reto_1 y luego ejecuta la matriz detectada.

        Argumentos:
            forzar_matriz: Si se especifica un número (ej. 2), omite el resultado
                del escáner y fuerza la ejecución de esa matriz. Si es None, ejecuta
                la detectada o la matriz 2 por defecto.
        """
        print("=== INICIANDO RECORRIDO COMPLETO (RETO 1 + MATRIZ) ===")
        self.seccion_1_salida_y_cemento()
        self.seccion_2_dejar_cemento_y_tomar_verdes()
        matriz = self.seccion_3_escanear_matriz_y_dejar_verdes()
        self.seccion_4_amarillos_y_azules()
        self.seccion_5_tomar_pala_y_dejar_amarillos()
        self.seccion_6_retorno_pala_y_acomodo()
        gc.collect()

        if forzar_matriz is not None:
            matriz = forzar_matriz

        print("Matriz seleccionada:", matriz)
        if matriz == 2 or True:
            # Por ahora solo la matriz 2 está desarrollada e implementada
            print("Iniciando recorrido de la matriz 2...")
            self.ejecutar_matriz_2()
        else:
            print("Matriz Predeterminada", matriz)
            self.ejecutar_matriz_2()
