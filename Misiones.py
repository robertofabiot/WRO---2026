"""Módulo de misiones del recorrido de competencia.

Basado en la versión actualizada de prueba_reto_1, modularizado en métodos
independientes por sección, preservando al 100% las distancias, ángulos,
tiempos de espera, llamadas a gc.collect() y orden de ejecución.

Las llamadas a mecanismos quedan comentadas con la etiqueta TODO para su
posterior conversión a porcentaje en cuanto se definan los rangos máximos.
"""

from pybricks.parameters import Color
from pybricks.tools import wait
import gc
import config


class Misiones:
    """Secuencias de misiones del recorrido oficial WRO 2026 (rama actualizada)."""

    def __init__(self, robot, sensor_color=None):
        """
        Argumentos:
            robot: Instancia de Robot.
            sensor_color: Sensor de color del frente (por defecto robot.seguidor).
        """
        self.robot = robot
        self.sensor = sensor_color if sensor_color is not None else robot.seguidor

    # =========================================================================
    # SECCIÓN 1: Giro de salida y seguidor hasta el cemento
    # =========================================================================
    def seccion_1_salida_y_cemento(self):
        """Salida de base con arco, seguidor, toma de cemento y empuje de llana."""
        robot = self.robot

        # Inicialización de encoders a 0° en posición de salida
        robot.garra_principal.reset_angle(0)
        robot.garra_delantera.reset_angle(0)
        robot.torque.reset_angle(0)

        robot.giro_de_arco(
            radio_cm=22,
            angulo_deg=90,
            potencia_max=90,
            lado="derecha"
        )
        gc.collect()

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            distancia_cm=79,
            velocidad_max=100,
            lado="derecha",
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
        gc.collect()

        # Giro frente al cemento
        robot.girar(angulo_deg=-90, potencia_max=100, perfil="encadenado")

        # Bajar mecanismo de torque para tomar el cemento (-169° medido = 93.0%)
        # Original: robot.mover_torque(grados_torque=-169, velocidad_torque=600, esperar=False)
        robot.torque.ir_a_porcentaje(93.0, velocidad=600, wait_after=False)

        robot.avanzar_recto(distancia_cm=-16, velocidad_max=1000, perfil="encadenado")

        # Avance tras tomar cemento y giro en dirección a la llana
        robot.avanzar_recto(distancia_cm=8.5, velocidad_max=1000, perfil="encadenado")
        robot.girar(angulo_deg=70, potencia_max=100, perfil="encadenado")

        # Retroceso para dejar la llana y cruce de líneas para regresar a la línea
        robot.avanzar_recto(distancia_cm=-32, velocidad_max=1000, perfil="seguro")

        robot.avanzar_cruzando_lineas(
            cruces_objetivo=2,
            velocidad=900,
            escape_inicial_cm=8,
            retraso_freno_ms=0
        )
        gc.collect()

        # Giro de arco para posicionarse sobre la línea
        robot.giro_de_arco(
            radio_cm=13,
            angulo_deg=19,
            potencia_max=90,
            lado="derecha"
        )

    # =========================================================================
    # SECCIÓN 2: Dejar el cemento y tomar los cementos verdes
    # =========================================================================
    def seccion_2_dejar_cemento_y_tomar_verdes(self):
        """Seguidor hasta zona de cemento, depósito coordinado y agarre de verdes."""
        robot = self.robot

        robot.seguir_linea(
            sensor_color=robot.seguidor,
            distancia_cm=50,
            velocidad_max=100,
            lado="derecha",
            tiempo_acomodo_ms=50,
            tiempo_aceleracion_ms=80,
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=450,
            potencia_captura=35,
            kp_captura=3.5,
            margen_captura=4,
            lecturas_estables_captura=4,
            perfil_salida="encadenado"
        )
        wait(100)

        robot.girar(angulo_deg=-90, potencia_max=85, perfil="encadenado")

        # Avance coordinado con torque para soltar el cemento (sube a 0% a los 13 cm)
        # Original: robot.avanzar_con_torque(distancia_cm=39.5, distancia_activacion_torque_cm=13, torque_grados=170)
        robot.avanzar_con_torque(
            distancia_cm=39.5,
            distancia_activacion_torque_cm=13,
            torque_porcentaje=0,
            torque_velocidad=900
        )

        robot.girar(angulo_deg=90, potencia_max=85, perfil="encadenado")

        # Seguir línea para tomar los cementos verdes
        robot.seguir_linea_hasta_color(
            color_objetivo=Color.GREEN,
            velocidad_max=95,
            lado="derecha"
        )

        robot.avanzar_recto(distancia_cm=-7, velocidad_max=900, perfil="encadenado")
        robot.girar(angulo_deg=-188, potencia_max=90, perfil="encadenado")

        # Agarrar los cementos verdes (-170° medido = 93.4%)
        # Original: robot.mover_torque(grados_torque=-170, velocidad_torque=250, esperar=False)
        robot.torque.ir_a_porcentaje(93.4, velocidad=250, wait_after=False)

        robot.avanzar_recto(distancia_cm=-22, velocidad_max=900, perfil="encadenado")

    # =========================================================================
    # SECCIÓN 3: Escanear la matriz y dejar los cementos verdes
    # =========================================================================
    def seccion_3_escanear_matriz_y_dejar_verdes(self, armador):
        """Llegada a la matriz, escaneo de colores, retroceso y depósito de verdes."""
        robot = self.robot

        # Seguir línea hasta la matriz
        robot.seguir_linea_hasta_color(
            color_objetivo=Color.GREEN,
            velocidad_max=95,
            lado="izquierda"
        )

        # Entrar a escanear la matriz
        robot.girar_corto(5, potencia_max=54, potencia_min=34)
        robot.avanzar_recto(14, 700)

        matriz_detectada = armador.escanear_matriz()

        robot.girar_corto(-5, potencia_max=54, potencia_min=34)

        # Salir de la matriz
        robot.avanzar_recto(distancia_cm=-37.5, velocidad_max=900, perfil="seguro")
        robot.girar(angulo_deg=-180, potencia_max=90, perfil="encadenado")
        gc.collect()

        # Dejar los cementos verdes
        robot.avanzar_recto(distancia_cm=-19.6, velocidad_max=900, perfil="encadenado")

        # Subir torque para soltar los cementos verdes (retorna a 0%)
        # Original: robot.mover_torque(grados_torque=170, velocidad_torque=350, esperar=False)
        robot.torque.ir_a_porcentaje(0, velocidad=350, wait_after=False)

        return matriz_detectada

    # =========================================================================
    # SECCIÓN 4: Ir por los cementos amarillos y tomar los azules
    # =========================================================================
    def seccion_4_amarillos_y_azules(self):
        """Navegación al pasillo de amarillos y agarre de bloques azules."""
        robot = self.robot

        # Ir por los cementos amarillos
        robot.seguir_linea(
            sensor_color=robot.seguidor,
            distancia_cm=48,
            velocidad_max=100,
            lado="derecha",
            tiempo_acomodo_ms=100,
            tiempo_aceleracion_ms=80,
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=55,
            kp_captura=3.8,
            perfil_salida="encadenado"
        )
        robot.avanzar_recto(distancia_cm=18, velocidad_max=900, perfil="encadenado")

        robot.girar(angulo_deg=-90, potencia_max=90, perfil="encadenado")
        robot.avanzar_recto(distancia_cm=23, velocidad_max=900, perfil="encadenado")
        robot.girar(angulo_deg=-90, potencia_max=80, perfil="encadenado")
        robot.avanzar_recto(distancia_cm=15, velocidad_max=900, perfil="encadenado")
        robot.girar(angulo_deg=-93.5, potencia_max=80, perfil="encadenado")
        wait(100)

        # Ir por los azules
        robot.avanzar_cruzando_lineas(cruces_objetivo=2, velocidad=700, retraso_freno_ms=90)
        robot.girar(angulo_deg=89, potencia_max=80, perfil="encadenado")

        # Tomar los azules (-170° medido = 93.4%)
        # Original: robot.mover_torque(grados_torque=-170, velocidad_torque=250, esperar=False)
        robot.torque.ir_a_porcentaje(93.4, velocidad=250, wait_after=False)

        robot.avanzar_recto(distancia_cm=-22, velocidad_max=900, perfil="encadenado")

    # =========================================================================
    # SECCIÓN 5: Ir por la pala y dejar los amarillos
    # =========================================================================
    def seccion_5_tomar_pala_y_dejar_amarillos(self):
        """Captura de la pala con garras delantera y principal, y depósito de amarillos."""
        robot = self.robot

        # Ir por la pala
        robot.avanzar_recto(distancia_cm=20, velocidad_max=900, perfil="encadenado")
        robot.girar(angulo_deg=-38, potencia_max=90, perfil="encadenado")

        # Posicionar garra delantera a 245° (69.4%) y garra principal a 200° (47.6%)
        # Original: robot.mover_garra_delantera(posicion=245, simultaneo=True)
        # Original: robot.mover_garra_principal(velocidad=1000, grados=200, esperar=True)
        robot.garra_delantera.ir_a_porcentaje(69.4, velocidad=700, wait_after=False)
        robot.garra_principal.ir_a_porcentaje(47.6, velocidad=1000, wait_after=True)

        robot.avanzar_cruzando_lineas(2, velocidad=500, distancia_extra_cm=13)

        # Cerrar garra principal a 0%
        # Original: robot.mover_garra_principal(velocidad=300, grados=0)
        robot.garra_principal.ir_a_porcentaje(0, velocidad=300, wait_after=True)

        robot.avanzar_recto(distancia_cm=-2.5, velocidad_max=900, perfil="encadenado")
        robot.girar(angulo_deg=36, potencia_max=80, perfil="encadenado")

        # Ir a dejar los amarillos
        robot.avanzar_recto(distancia_cm=48, velocidad_max=900)

        # Abrir garra principal a 300° (71.4%), subir garra delantera a 0% y cerrar garra principal a 0%
        # Original: robot.mover_garra_principal(velocidad=1000, grados=300, esperar=True)
        # Original: robot.mover_garra_delantera(posicion=0, simultaneo=True)
        # Original: robot.mover_garra_principal(velocidad=1000, grados=0, esperar=True)
        robot.garra_principal.ir_a_porcentaje(71.4, velocidad=1000, wait_after=True)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=False)
        robot.garra_principal.ir_a_porcentaje(0, velocidad=1000, wait_after=True)

        robot.girar(angulo_deg=90, potencia_max=80, perfil="encadenado")
        wait(200)

        robot.avanzar_hibrido(distancia_inicial_cm=4, color_objetivo=Color.YELLOW)
        robot.avanzar_recto(distancia_cm=8, velocidad_max=900)

        # Salir de la sección amarilla
        wait(200)
        robot.avanzar_recto(distancia_cm=-22.5, velocidad_max=900)
        robot.girar(angulo_deg=-92, potencia_max=90, perfil="encadenado")

    # =========================================================================
    # SECCIÓN 6: Retorno con pala, seguidor con torque coordinado y resolución de matriz
    # =========================================================================
    def seccion_6_retorno_pala_y_fin(self, armador, matriz_detectada=None):
        """Retorno con pala, depósito coordinado con seguidor de línea y armado de matriz."""
        robot = self.robot

        # Posicionar garra principal a 200° (47.6%) y garra delantera a 245° (69.4%)
        # Original: robot.mover_garra_principal(velocidad=1000, grados=200, esperar=True)
        # Original: robot.mover_garra_delantera(posicion=245, simultaneo=True)
        robot.garra_principal.ir_a_porcentaje(47.6, velocidad=1000, wait_after=True)
        robot.garra_delantera.ir_a_porcentaje(69.4, velocidad=700, wait_after=False)

        robot.avanzar_recto(distancia_cm=-2, velocidad_max=900)

        # Seguir línea con disparo de torque a los 65 cm (retorna a 0%)
        # Original: torque 170° a los 65 cm
        robot.seguir_linea_y_mover_torque(
            sensor_color=robot.seguidor,
            distancia_cm=85,
            velocidad_max=100,
            lado="izquierda",
            tiempo_acomodo_ms=100,
            tiempo_aceleracion_ms=80,
            kp=1.15,
            kd=3.8,
            k_freno=0.05,
            correccion_max=100,
            objetivo_reflexion=27,
            captura_inicial=True,
            tiempo_captura_ms=280,
            potencia_captura=55,
            kp_captura=3.8,
            perfil_salida="encadenado",
            distancia_torque_cm=65,
            torque_porcentaje=0,
            torque_velocidad=350
        )

        robot.establecer_norte()
        robot.girar_corto(-40, potencia_max=54, potencia_min=34)

        # Subir garra delantera a 0% y cerrar garra principal a 0%
        # Original: robot.mover_garra_delantera(posicion=0, simultaneo=True)
        # Original: robot.mover_garra_principal(velocidad=1000, grados=0, esperar=True)
        robot.garra_delantera.ir_a_porcentaje(0, velocidad=700, wait_after=False)
        robot.garra_principal.ir_a_porcentaje(0, velocidad=1000, wait_after=True)

        robot.girar_corto(40, potencia_max=54, potencia_min=34)
        gc.collect()

        # Determinación de la matriz a ejecutar
        target_matriz = matriz_detectada
        if target_matriz is None:
            target_matriz = armador.matriz_detectada
        if target_matriz is None:
            target_matriz = 2  # Predeterminada según prueba_reto_1

        print("Matriz detectada:", target_matriz)
        if target_matriz == 2:
            print("Iniciando recorrido de la matriz 2...")
            armador.ejecutar_matriz_2()
        elif target_matriz == 3:
            print("Iniciando recorrido de la matriz 3...")
            armador.ejecutar_matriz_3()
        else:
            print("Matriz Predeterminada (ejecutando 2):", target_matriz)
            armador.ejecutar_matriz_2()
