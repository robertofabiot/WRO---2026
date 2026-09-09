"""Módulo para el escaneo, identificación y resolución de matrices de mosaicos.

Unifica las funciones de lectura estática y entrega de bloques (matriz.py)
con las secuencias completas de las matrices 1, 2, 3 y 4 (matriz_1 a 4).
Las llamadas a garras quedan explícitamente marcadas con TODO para la futura
conversión a porcentaje.
"""

from pybricks.parameters import Color
from pybricks.tools import wait
import gc
import config


class ArmadorMosaicos:
    """Controlador de lectura de matrices y ejecución de secuencias de mosaicos."""

    def __init__(self, robot, sensor_color=None):
        """
        Argumentos:
            robot: Instancia de Robot.
            sensor_color: Sensor de color del frente (por defecto robot.seguidor).
        """
        self.robot = robot
        self.sensor = sensor_color if sensor_color is not None else robot.seguidor
        self.matriz_detectada = None

    # =========================================================================
    # LECTURA Y ESCANEO DE LA MATRIZ (Lógica de escaneo del robot del usuario)
    # =========================================================================

    # Colores que la matriz puede mostrar. Todo lo demás que devuelva el escáner
    # preciso (GRAY, BLACK) es piso o línea: se descarta y se vuelve a leer en
    # lugar de dar la lectura por buena.
    COLORES_MATRIZ = (Color.GREEN, Color.YELLOW, Color.BLUE, Color.RED, Color.WHITE)

    def realizar_lectura_estatica(
        self,
        lecturas_confirmacion=None,
        espera_inicial_ms=None,
        intervalo_lecturas_ms=None,
        lecturas_maximas=None
    ):
        """Lee el color de la celda con el robot detenido y lo devuelve.

        Reemplaza la votación por mayoría del equipo original (250 ms de espera
        más 25 lecturas cada 40 ms: ~1,25 s por celda) por el criterio de los
        lazos de Navegacion: se clasifica con detectar_color_preciso() y se
        exigen unas pocas lecturas seguidas iguales. Ronda los 60 ms por celda.

        Argumentos:
            lecturas_confirmacion: lecturas seguidas del mismo color que se
                exigen antes de dar el corte por bueno.
            espera_inicial_ms: micro-pausa tras frenar, para que el sensor lea
                sin las vibraciones del motor.
            intervalo_lecturas_ms: cadencia de muestreo.
            lecturas_maximas: corte de emergencia si nunca aparece un color de
                matriz válido.

        Devuelve un Color de Pybricks, o None si la lectura no cierra.
        """
        if lecturas_confirmacion is None:
            lecturas_confirmacion = config.LECTURAS_CONFIRMACION_MATRIZ
        if espera_inicial_ms is None:
            espera_inicial_ms = config.ESPERA_ASENTAMIENTO_MS
        if intervalo_lecturas_ms is None:
            intervalo_lecturas_ms = config.INTERVALO_LECTURAS_MATRIZ_MS
        if lecturas_maximas is None:
            lecturas_maximas = config.LECTURAS_MAXIMAS_MATRIZ

        self.robot.frenar()
        wait(espera_inicial_ms)

        color_candidato = None
        confirmaciones = 0

        for _ in range(lecturas_maximas):
            color = self.robot.detectar_color_preciso(self.sensor)

            if color in self.COLORES_MATRIZ:
                if color == color_candidato:
                    confirmaciones += 1
                else:
                    color_candidato = color
                    confirmaciones = 1

                if confirmaciones >= lecturas_confirmacion:
                    return color_candidato
            else:
                color_candidato = None
                confirmaciones = 0

            wait(intervalo_lecturas_ms)

        return None

    def escanear_matriz(self):
        """Escanea la matriz de colores y devuelve el número correspondiente (1 a 5).

        La numeración y el recorrido son los del equipo original y no se tocan:
        solo cambia de dónde sale el color. El avance de 4 cm cuando sale verde
        (para desempatar matriz 1 contra matriz 4) se conserva tal cual, igual
        que el hecho de no volver atrás después de leer la segunda celda.
        """
        primer_color = self.realizar_lectura_estatica()
        if primer_color is None:
            print("No se detectó un color de matriz válido.")
            return None

        if primer_color == Color.GREEN:
            self.robot.avanzar_recto(
                distancia_cm=4,
                velocidad_max=300,
                perfil="rapido"
            )
            segundo_color = self.realizar_lectura_estatica()
            matriz_detectada = 4 if segundo_color == Color.YELLOW else 1
        elif primer_color == Color.YELLOW:
            matriz_detectada = 2
        elif primer_color == Color.BLUE:
            matriz_detectada = 3
        elif primer_color == Color.RED:
            matriz_detectada = 4
        elif primer_color == Color.WHITE:
            matriz_detectada = 5
        else:
            matriz_detectada = None

        self.matriz_detectada = matriz_detectada
        print("Matriz detectada:", matriz_detectada)
        return matriz_detectada

    # =========================================================================
    # ENTREGA DE BLOQUES EN LA MATRIZ (Preservado de matriz.py)
    # =========================================================================

    def dejar_bloques_matriz(self):
        """Secuencia de posicionamiento y entrega de bloques en la matriz (versión 1)."""
        robot = self.robot

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=65,
            distancia_cm=15,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.ir_a_porcentaje(54.8, velocidad=900, wait_after=True)
        robot.garra_delantera.ir_a_porcentaje(65.2, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=-14, velocidad_max=400, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(76.5, velocidad=700, wait_after=True)

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=100,
            distancia_cm=13,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.apretar(potencia=100, velocidad=300)
        robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)

        robot.seguir_linea_hasta_color(
            color_objetivo=Color.BLUE,
            velocidad_max=100,
            lado="derecha"
        )

        wait(400)
        robot.girar_corto(-10)
        robot.avanzar_recto(distancia_cm=12.5, velocidad_max=650, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(62.3, velocidad=700, wait_after=True)
        robot.garra_principal.mover_rapida(130)

        robot.avanzar_recto(distancia_cm=-0.6, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=1.8, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

        for _ in range(4):
            robot.girar_corto(8, potencia_max=50, potencia_min=40)
            robot.girar_corto(-8, potencia_max=50, potencia_min=40)

        robot.avanzar_recto(distancia_cm=-1, velocidad_max=500, zona_rampa_cm=0.5, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=-17, velocidad_max=500, perfil="seguro")
        robot.girar(
            angulo_deg=180,
            potencia_max=80,
            potencia_min=70,
            kp_base=4.0,
            kd_base=6.0,
            tolerancia_fin=0.6,
            perfil="seguro"
        )

    def dejar_bloques_matriz2(self):
        """Secuencia correspondiente al recorrido auxiliar de matriz 2."""
        robot = self.robot

        robot.avanzar_cruzando_lineas(
            cruces_objetivo=1,
            velocidad=900,
            escape_inicial_cm=8,
            retraso_freno_ms=90
        )
        robot.girar(
            95,
            potencia_max=85,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )
        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=50,
            distancia_cm=7,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.ir_a_porcentaje(54.8, velocidad=900, wait_after=True)
        robot.garra_delantera.ir_a_porcentaje(65.2, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=-17, velocidad_max=400, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(76.5, velocidad=700, wait_after=True)

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=100,
            distancia_cm=16,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.apretar(potencia=100, velocidad=300)
        robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)

        robot.seguir_linea_hasta_color(
            color_objetivo=Color.BLUE,
            velocidad_max=100,
            lado="derecha"
        )

        wait(200)
        robot.girar_corto(-11.5)
        robot.avanzar_recto(
            distancia_cm=3,
            velocidad_max=650,
            zona_rampa_cm=0.1,
            perfil="encadenado"
        )

        robot.garra_delantera.ir_a_porcentaje(62.3, velocidad=700, wait_after=True)
        robot.garra_principal.mover_rapida(130)

        robot.avanzar_recto(distancia_cm=-0.6, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=1.8, velocidad_max=750, zona_rampa_cm=0.1, perfil="encadenado")

        for _ in range(4):
            robot.girar_corto(8, potencia_max=50, potencia_min=40)
            robot.girar_corto(-8, potencia_max=50, potencia_min=40)

        robot.avanzar_recto(distancia_cm=-1, velocidad_max=900, zona_rampa_cm=0.1, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=-29, velocidad_max=900, zona_rampa_cm=0.1, perfil="encadenado")
        robot.girar(
            180,
            potencia_max=85,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )
        robot.avanzar_recto(distancia_cm=-21, velocidad_max=900, zona_rampa_cm=0.1, perfil="encadenado")

    def dejar_bloques_matriz3(self, distancia_entrada=0):
        """Secuencia de entrega de bloques para la matriz 3."""
        robot = self.robot

        robot.garra_delantera.ir_a_porcentaje(22.7, velocidad=700, wait_after=True)

        robot.avanzar_recto(-8)

        robot.garra_principal.ir_a_porcentaje(42.9, velocidad=100, wait_after=False)
        robot.garra_delantera.ir_a_porcentaje(77.9, velocidad=700, wait_after=True)

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=100,
            distancia_cm=15,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.apretar(potencia=100, velocidad=300)
        robot.garra_delantera.ir_a_porcentaje(28.3, velocidad=700, wait_after=True)

        robot.seguir_linea_hasta_color(
            color_objetivo=Color.BLUE,
            velocidad_max=100,
            lado="derecha"
        )

        wait(400)
        robot.girar_corto(-9.8)
        robot.avanzar_recto(
            distancia_cm=distancia_entrada,
            velocidad_max=650,
            perfil="encadenado"
        )

        robot.garra_delantera.ir_a_porcentaje(62.3, velocidad=700, wait_after=True)
        robot.garra_principal.mover_rapida(125)

        robot.avanzar_recto(distancia_cm=-0.6, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=2, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

        for _ in range(4):
            robot.girar_corto(8, potencia_max=75, potencia_min=45)
            robot.girar_corto(-8, potencia_max=75, potencia_min=45)

        robot.avanzar_recto(distancia_cm=-1, velocidad_max=500, zona_rampa_cm=0.5, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(53.8, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=-18, velocidad_max=500, perfil="seguro")
        robot.girar(
            180,
            potencia_max=90,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

    # =========================================================================
    # RECORRIDOS ESPECÍFICOS DE MATRICES (Preservados de matriz_1 a 4)
    # =========================================================================

    def ejecutar_matriz_1(self):
        """Ejecuta el recorrido de resolución para la matriz 1."""
        robot = self.robot
        print("Ejecutando recorrido de matriz 1")

        robot.garra_delantera.reset_angle(0)

        robot.avanzar_cruzando_lineas(cruces_objetivo=3, velocidad=900, retraso_freno_ms=95.5)

        robot.garra_principal.ir_a_porcentaje(23.8, velocidad=850, wait_after=True)

        robot.girar(-90, potencia_max=100)
        robot.avanzar_recto(5, 800)

        robot.garra_delantera.ir_a_porcentaje(79.3, velocidad=700, wait_after=True)

        robot.avanzar_recto(-8, 800)
        robot.girar(-90, potencia_max=100)
        robot.avanzar_recto(13)
        robot.girar(92, potencia_max=100)

        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=True)

        robot.avanzar_recto(8, 800)

        robot.garra_delantera.ir_a_porcentaje(79.3, velocidad=700, wait_after=True)

        wait(300)
        robot.avanzar_recto(-8.5, 800)
        robot.girar(88, potencia_max=100)
        robot.avanzar_recto(9.5)
        robot.girar(-90, potencia_max=80)

        robot.garra_principal.ir_a_porcentaje(0, velocidad=850, wait_after=True)

        robot.avanzar_recto(11)

        robot.garra_principal.apretar(potencia=100, velocidad=850)

        robot.avanzar_recto(-11)
        robot.girar(-90)
        robot.avanzar_recto(2.5)
        robot.girar(-90)
        robot.avanzar_recto(3)

        self.dejar_bloques_matriz()

        robot.avanzar_recto(-12)
        robot.girar(-170)
        robot.avanzar_recto(5)
        robot.seguir_linea(distancia_cm=17)
        robot.avanzar_recto(20)
        robot.girar(-90)
        robot.avanzar_recto(6.5)
        robot.girar(90)
        robot.avanzar_recto(16)

        robot.garra_delantera.ir_a_porcentaje(79.3, velocidad=700, wait_after=True)

        robot.avanzar_recto(-18)
        robot.girar(90)
        robot.avanzar_recto(16)

        robot.garra_principal.ir_a_porcentaje(8.3, velocidad=850, wait_after=True)

        robot.girar(-90)
        robot.avanzar_recto(10)

        robot.garra_principal.apretar(potencia=40, velocidad=850)

        robot.avanzar_recto(-11)
        robot.girar(-90)
        robot.avanzar_recto(9)
        robot.girar(-91)
        robot.avanzar_recto(3)

        self.dejar_bloques_matriz2()

        robot.avanzar_recto(-20)
        robot.girar(-170)

    def ejecutar_matriz_2(self):
        """Ejecuta el recorrido de resolución para la matriz 2."""
        robot = self.robot
        print("Ejecutando recorrido de matriz 2")
        gc.collect()

        robot.garra_principal.reset_angle(0)
        robot.garra_delantera.reset_angle(0)

        robot.avanzar_recto(distancia_cm=-9, velocidad_max=900)
        robot.girar(
            90,
            potencia_max=65,
            potencia_min=45,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )
        wait(200)

        robot.avanzar_cruzando_lineas(
            cruces_objetivo=2,
            velocidad=900,
            escape_inicial_cm=8,
            retraso_freno_ms=91
        )
        gc.collect()
        wait(200)

        robot.girar(
            -90,
            potencia_max=65,
            potencia_min=45,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        robot.garra_principal.ir_a_porcentaje(71.4, velocidad=900, wait_after=True)

        robot.avanzar_hasta_salir_negro(
            velocidad_max=900,
            velocidad_min=200,
            objetivo_reflexion=15,
            lecturas_salida=4
        )
        robot.avanzar_recto(distancia_cm=4.3, velocidad_max=900, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=-20.7, velocidad_max=900, perfil="seguro")
        wait(200)

        robot.girar(
            -90,
            potencia_max=75,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=60,
            distancia_cm=8,
            lado="izquierda",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )
        wait(200)

        robot.girar(
            90,
            potencia_max=85,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=True)
        robot.garra_principal.ir_a_porcentaje(71.4, velocidad=800, wait_after=False)

        wait(200)
        robot.avanzar_recto(distancia_cm=14, velocidad_max=750, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=700, wait_after=True)
        robot.garra_principal.ir_a_porcentaje(0, velocidad=300, wait_after=False)

        robot.avanzar_recto(distancia_cm=-14, velocidad_max=550, perfil="seguro")
        wait(200)

        robot.girar(
            90,
            potencia_max=85,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=100,
            distancia_cm=14,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )
        wait(200)

        robot.girar(
            90,
            potencia_max=90,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        self.dejar_bloques_matriz()

        # Segunda parte de la matriz 2
        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=80,
            distancia_cm=24,
            lado="izquierda",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.girar(
            -90,
            potencia_max=90,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        robot.avanzar_recto(distancia_cm=4.65, velocidad_max=400, perfil="seguro")

        robot.girar(
            90,
            potencia_max=90,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=80,
            distancia_cm=9,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.avanzar_recto(distancia_cm=22, velocidad_max=350, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(79.3, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=-28, velocidad_max=400, perfil="seguro")

        robot.girar(
            -90,
            potencia_max=90,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=60,
            distancia_cm=13,
            lado="izquierda",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )
        wait(200)

        robot.girar(
            90,
            potencia_max=90,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )

        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=True)
        robot.garra_principal.ir_a_porcentaje(71.4, velocidad=800, wait_after=False)

        wait(300)
        robot.avanzar_recto(distancia_cm=13, velocidad_max=750, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(79.3, velocidad=700, wait_after=True)
        robot.garra_principal.ir_a_porcentaje(0, velocidad=300, wait_after=False)

        robot.avanzar_recto(distancia_cm=14.9, velocidad_max=450, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(79.3, velocidad=700, wait_after=True)

        robot.avanzar_recto(distancia_cm=-40.9, velocidad_max=550, perfil="seguro")

        robot.girar(
            89,
            potencia_max=90,
            potencia_min=35,
            kp_base=5.0,
            tolerancia_fin=1.0,
            perfil="encadenado"
        )
        robot.avanzar_recto(distancia_cm=-15, velocidad_max=550, perfil="seguro")

        robot.torque.ir_a_porcentaje(93.1, velocidad=600, wait_after=False)

        self.dejar_bloques_matriz2()

    def ejecutar_matriz_3(self):
        """Ejecuta el recorrido de resolución para la matriz 3."""
        robot = self.robot
        print("Ejecutando recorrido de matriz 3")
        robot.establecer_norte()

        robot.avanzar_recto(-10)
        robot.girar_a_rumbo(89)

        robot.garra_delantera.reset_angle(0)
        robot.garra_principal.reset_angle(0)

        gc.collect()
        wait(400)

        robot.avanzar_cruzando_lineas(cruces_objetivo=3, velocidad=700, distancia_extra_cm=12)

        robot.garra_principal.ir_a_porcentaje(46.4, velocidad=900, wait_after=False)

        wait(50)
        robot.girar(-90)

        robot.garra_delantera.ir_a_porcentaje(73.7, velocidad=700, wait_after=True)

        robot.avanzar_recto(6, 800)

        robot.garra_principal.ir_a_porcentaje(0, velocidad=900, wait_after=True)

        robot.avanzar_recto(-20)
        robot.girar(-90)
        robot.avanzar_recto(10.3)
        robot.girar(-90)

        gc.collect()

        robot.garra_principal.ir_a_porcentaje(16.7, velocidad=900, wait_after=True)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=True)
        robot.garra_principal.ir_a_porcentaje(47.6, velocidad=900, wait_after=False)

        robot.avanzar_recto(-5.3, 800)
        robot.girar(90)

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            distancia_cm=27,
            velocidad_max=100,
            lado="izquierda",
            tiempo_acomodo_ms=50,
            tiempo_aceleracion_ms=80,
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.girar(90)
        robot.avanzar_recto(21.5, 600)

        robot.garra_delantera.ir_a_porcentaje(76.5, velocidad=700, wait_after=True)

        robot.avanzar_recto(-24, 750)
        robot.girar(90)
        wait(100)

        robot.avanzar_cruzando_lineas(cruces_objetivo=1, velocidad=300, escape_inicial_cm=5, distancia_extra_cm=5)
        robot.girar(90)

        self.dejar_bloques_matriz3(distancia_entrada=13)

        # Segunda parte de la matriz 3
        wait(30)

        robot.garra_principal.ir_a_porcentaje(19.0, velocidad=800, wait_after=False)

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            distancia_cm=27,
            velocidad_max=100,
            lado="izquierda",
            tiempo_acomodo_ms=50,
            tiempo_aceleracion_ms=80,
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )
        robot.girar(-90)
        wait(30)

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            distancia_cm=15,
            velocidad_max=100,
            lado="izquierda",
            tiempo_acomodo_ms=50,
            tiempo_aceleracion_ms=80,
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.girar(90)

        robot.garra_delantera.ir_a_porcentaje(73.7, velocidad=700, wait_after=False)

        robot.avanzar_recto(14)

        robot.garra_principal.ir_a_porcentaje(0, velocidad=800, wait_after=True)

        robot.avanzar_recto(-17)
        robot.girar(100)

        robot.garra_principal.ir_a_porcentaje(23.8, velocidad=800, wait_after=True)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=False)

        robot.girar_corto(-15)

        robot.garra_delantera.ir_a_porcentaje(68.0, velocidad=700, wait_after=False)

        robot.avanzar_recto(24.8)

        robot.garra_principal.ir_a_porcentaje(47.6, velocidad=900, wait_after=True)

        wait(50)
        robot.girar(-90)

        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=True)

        robot.avanzar_recto(23.7, 600)

        robot.garra_delantera.ir_a_porcentaje(73.7, velocidad=700, wait_after=True)

        robot.avanzar_recto(-20)
        robot.girar(90)
        robot.avanzar_recto(14.5)
        robot.girar(-90)

        robot.garra_principal.ir_a_porcentaje(31.0, velocidad=900, wait_after=True)

        robot.avanzar_recto(14)

        robot.garra_principal.ir_a_porcentaje(0, velocidad=800, wait_after=True)

        robot.avanzar_recto(-20.4)
        robot.girar(-90)
        robot.avanzar_recto(17)
        robot.girar(-90)

        robot.garra_principal.ir_a_porcentaje(28.6, velocidad=900, wait_after=True)

        self.dejar_bloques_matriz3(distancia_entrada=3)

    def ejecutar_matriz_4(self):
        """Ejecuta el recorrido de resolución para la matriz 4."""
        robot = self.robot
        print("Ejecutando recorrido de matriz 4")

        robot.avanzar_cruzando_lineas(2, 900, retraso_freno_ms=95.5)

        robot.garra_principal.ir_a_porcentaje(31.0, velocidad=850, wait_after=True)

        robot.girar(-90, potencia_max=100)
        robot.avanzar_recto(10, 800)

        robot.garra_delantera.ir_a_porcentaje(81.0, velocidad=850, wait_after=True)

        robot.avanzar_recto(-8.5, 800)
        robot.girar(88, potencia_max=100)
        robot.avanzar_recto(9.5)
        robot.girar(-90, potencia_max=80)

        robot.garra_principal.ir_a_porcentaje(0, velocidad=850, wait_after=True)

        robot.avanzar_recto(11)

        robot.garra_principal.apretar(potencia=100, velocidad=850)

        robot.avanzar_recto(distancia_cm=10, velocidad_max=900, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(82.2, velocidad=600, wait_after=True)

        robot.avanzar_recto(distancia_cm=-20.8, velocidad_max=700, perfil="seguro")
        wait(200)

        robot.girar(-90, potencia_max=85, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")
        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=60,
            distancia_cm=9,
            lado="izquierda",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )
        wait(200)

        robot.girar(90, potencia_max=85, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(0, velocidad=600, wait_after=True)
        robot.garra_principal.ir_a_porcentaje(0, velocidad=300, wait_after=False)

        wait(200)
        robot.avanzar_recto(distancia_cm=13, velocidad_max=750, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(4.2, velocidad=600, wait_after=True)
        robot.garra_principal.apretar(potencia=80, velocidad=300)

        robot.avanzar_recto(distancia_cm=-14, velocidad_max=550, perfil="seguro")
        wait(200)

        robot.girar(90, potencia_max=85, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")
        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=100,
            distancia_cm=14,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )
        wait(200)

        robot.girar(90, potencia_max=90, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")
        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=60,
            distancia_cm=12,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.ir_a_porcentaje(19.0, velocidad=300, wait_after=False)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=600, wait_after=True)

        robot.avanzar_recto(distancia_cm=-13, velocidad_max=400, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(26.3, velocidad=600, wait_after=True)

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=100,
            distancia_cm=15,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.apretar(potencia=100, velocidad=500)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=600, wait_after=True)

        robot.seguir_linea_hasta_color(color_objetivo=Color.BLUE, velocidad_max=100, lado="derecha")
        wait(20)

        robot.girar_corto(-10)
        robot.avanzar_recto(distancia_cm=13, velocidad_max=650, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(28.0, velocidad=400, wait_after=True)
        robot.garra_principal.mover_rapida(grados=50, potencia=100, abrir=True)
        robot.garra_delantera.ir_a_porcentaje(18.4, velocidad=400, wait_after=True)

        robot.avanzar_recto(distancia_cm=0.5, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

        for _ in range(5):
            robot.girar_corto(8, potencia_max=55, potencia_min=45)
            robot.girar_corto(-8, potencia_max=55, potencia_min=45)

        robot.avanzar_recto(distancia_cm=-1, velocidad_max=400, zona_rampa_cm=0.5, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(0, velocidad=400, wait_after=True)

        robot.avanzar_recto(distancia_cm=-18, velocidad_max=400, perfil="seguro")
        robot.girar(180, potencia_max=90, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")

        # Segunda parte matriz 4
        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=80,
            distancia_cm=24,
            lado="izquierda",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.girar(-90, potencia_max=90, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")
        robot.avanzar_recto(distancia_cm=4.65, velocidad_max=400, perfil="seguro")
        robot.girar(90, potencia_max=90, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=80,
            distancia_cm=9,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.avanzar_recto(distancia_cm=22, velocidad_max=350, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(42.5, velocidad=400, wait_after=True)

        robot.avanzar_recto(distancia_cm=-28, velocidad_max=400, perfil="seguro")
        robot.girar(-90, potencia_max=90, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=60,
            distancia_cm=13,
            lado="izquierda",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )
        wait(200)

        robot.girar(90, potencia_max=90, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(0, velocidad=600, wait_after=True)
        robot.garra_principal.ir_a_porcentaje(11.9, velocidad=300, wait_after=False)

        wait(300)
        robot.avanzar_recto(distancia_cm=13, velocidad_max=750, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(5.1, velocidad=600, wait_after=True)
        robot.garra_principal.apretar(potencia=80, velocidad=300)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=600, wait_after=True)

        robot.avanzar_recto(distancia_cm=14.9, velocidad_max=450, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(22.7, velocidad=600, wait_after=True)

        robot.avanzar_recto(distancia_cm=-40.9, velocidad_max=550, perfil="seguro")
        robot.girar(89, potencia_max=90, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")
        robot.avanzar_recto(distancia_cm=-15, velocidad_max=550, perfil="seguro")

        robot.torque.ir_a_porcentaje(93.1, velocidad=600, wait_after=False)

        robot.avanzar_cruzando_lineas(cruces_objetivo=1, velocidad=900, escape_inicial_cm=8, retraso_freno_ms=90)
        robot.avanzar_recto(distancia_cm=1, velocidad_max=400, perfil="seguro")
        robot.girar(96, potencia_max=85, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=50,
            distancia_cm=6,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.ir_a_porcentaje(21.4, velocidad=300, wait_after=False)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=600, wait_after=True)

        robot.avanzar_recto(distancia_cm=-16, velocidad_max=400, perfil="seguro")

        robot.garra_delantera.ir_a_porcentaje(26.6, velocidad=600, wait_after=True)

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            velocidad_max=100,
            distancia_cm=16,
            lado="derecha",
            tiempo_acomodo_ms=140,
            tiempo_aceleracion_ms=140,
            kp=1.25,
            kd=2.7,
            k_freno=0.16,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=60,
            kp_captura=2.5,
            perfil_salida="encadenado"
        )

        robot.garra_principal.apretar(potencia=100, velocidad=500)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=600, wait_after=True)

        robot.seguir_linea_hasta_color(color_objetivo=Color.BLUE, velocidad_max=100, lado="derecha")
        wait(200)

        robot.girar_corto(-10)
        robot.avanzar_recto(distancia_cm=5, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

        robot.garra_delantera.ir_a_porcentaje(28.0, velocidad=400, wait_after=True)
        robot.garra_principal.mover_rapida(grados=50, potencia=100, abrir=True)
        robot.garra_delantera.ir_a_porcentaje(18.4, velocidad=400, wait_after=True)

        robot.avanzar_recto(distancia_cm=0.4, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

        for _ in range(4):
            robot.girar_corto(8, potencia_max=50, potencia_min=45)
            robot.girar_corto(-8, potencia_max=50, potencia_min=45)

        robot.avanzar_recto(distancia_cm=-30, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")
        robot.girar(180, potencia_max=85, potencia_min=35, kp_base=5.0, tolerancia_fin=1.0, perfil="encadenado")
        robot.avanzar_recto(distancia_cm=-21, velocidad_max=650, zona_rampa_cm=0.1, perfil="encadenado")

    def armar(self, numero_mosaico=None):
        """Ejecuta el recorrido correspondiente al mosaico detectado o especificado."""
        if numero_mosaico is None:
            numero_mosaico = self.escanear_matriz()

        print("Armando matriz:", numero_mosaico)
        if numero_mosaico == 1:
            self.ejecutar_matriz_1()
        elif numero_mosaico == 2:
            self.ejecutar_matriz_2()
        elif numero_mosaico == 3:
            self.ejecutar_matriz_3()
        elif numero_mosaico == 4:
            self.ejecutar_matriz_4()
        else:
            print("No hay rutina implementada para la matriz", numero_mosaico)
