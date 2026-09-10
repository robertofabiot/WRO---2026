from pybricks.parameters import Color
from robot import Robot
from pybricks.tools import wait
import gc

class ArmadorMosaicos:
    """Rutinas de armado de la matriz, una por cada mosaico posible.

    El numero de mosaico sale de Misiones.escanear_mosaico() y elige que
    rutina correr. Cada rutina supone que el robot arranca cuadrado en la
    esquina de la matriz.
    """

    def __init__(self, robot_instancia: Robot, sensor_color, prueba=False):
        """
        Argumentos:
            robot_instancia: instancia de Robot con el chasis y los mecanismos.
            sensor_color: ColorSensor delantero.
            prueba: True calibra los ceros de las garras y deja la jaula abajo
                antes de empezar. En la corrida real ya vienen calibradas.
        """
        self.robot = robot_instancia
        self.sensor_color = sensor_color
        self.sensor = sensor_color
        
        self.rutinas = {
            1: self._armar_verde_verde,
            2: self._armar_verde_amarillo,
            3: self._armar_azul,
            4: self._armar_amarillo,
            5: self._armar_blanco
        }

        if prueba:
            self.robot.garra_trasera.establecer_cero()
            self.robot.garra_delantera.establecer_cero()
            self.robot.garra_delantera.establecer_cero_pinza()
            self.robot.garra_trasera.ir_a_porcentaje(100)

    def armar(self, numero_mosaico: int):
        """Corre la rutina de armado del mosaico pedido.

        Argumentos:
            numero_mosaico: numero devuelto por Misiones.escanear_mosaico().
                Si no esta en la tabla, cae en la rutina verde-verde.
        """
        rutina_a_ejecutar = self.rutinas.get(numero_mosaico, self._armar_verde_verde)
        print(f"Ejecutando rutina de armado para mosaico: {numero_mosaico}")
        rutina_a_ejecutar()
    
    def _armar_verde_verde(self):
        """Rutina de armado para mosaico verde-verde (pendiente de desarrollo)."""
        pass

    def _armar_verde_amarillo(self):
        """Rutina de armado para mosaico verde-amarillo (en desarrollo)."""
        # Acomodo
        self.robot.chasis.mover_motor_derecho(-600, encadenado=True)
        self.robot.chasis.avanzar_recto(-2, velocidad=1000, encadenado=True)
        self.robot.chasis.cuadrar_contra_pared(150, potencia=100, angulo_referencia=90)

        # Recoger dos bloques azules
        self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(self.sensor_color, 2, Color.BLACK, distancia_extra_cm=6, encadenado=True, debug=False)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.chasis.avanzar_recto(12, velocidad=200)
        self.robot.garra_delantera.ir_a_porcentaje(90)

        # Dejar dos bloques azules
        self.robot.chasis.avanzar_recto(-25)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(5)
        self.robot.navegacion.giro_absoluto(270, rueda_pivote="izquierda")

        # Recoger bloque verde
        self.robot.garra_delantera.ir_a_porcentaje_pinza(70, wait_after=False)
        self.robot.navegacion.avanzar_contando_lineas(self.sensor_color, 2, Color.BLACK, tiempo_ciego_s=0.3, velocidad=-1000, velocidad_lenta=-150)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.chasis.avanzar_recto(2, velocidad=100)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=80)

        # Dejar bloque verde
        self.robot.chasis.avanzar_recto(-4)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.navegacion.giro_absoluto(270)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(0)
        self.robot.chasis.avanzar_recto(12, velocidad=100)

    def _armar_azul(self):
        """Rutina de armado para mosaico azul (primera mitad operativa). Matriz de Roberto"""
        # Acomodo
        self.robot.chasis.mover_motor_derecho(-600, encadenado=True)
        self.robot.chasis.avanzar_recto(-2, velocidad=1000, encadenado=True)
        self.robot.chasis.cuadrar_contra_pared(150, potencia=100, angulo_referencia=90)

        # Recoger dos bloques azules y dos amarillos
        self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(self.sensor_color, 1, Color.BLACK, distancia_extra_cm=13.8, encadenado=False, debug=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(67, velocidad=1000, wait_after=False)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(10, velocidad=80)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)

        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)

        # Dejar dos bloques azules y dos amarillos
        self.robot.chasis.avanzar_recto(-27, velocidad=600, encadenado=False)
        self.robot.navegacion.giro_absoluto(90, encadenado=False)
        self.robot.chasis.avanzar_recto(10, velocidad=200, encadenado=False)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(0, velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje(80, wait_after=False)
        self.robot.chasis.avanzar_recto(4)
        self.robot.chasis.avanzar_recto(-4)

        # Agarrar dos bloques azules
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)
        self.robot.chasis.avanzar_recto(-2)
        self.robot.navegacion.giro_absoluto(0, encadenado=False)
        self.robot.chasis.avanzar_recto(36, encadenado=False)
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=True)
        
        # Dejar dos bloques azules
        self.robot.navegacion.giro_absoluto(90, rueda_pivote="izquierda")
        self.robot.chasis.avanzar_recto(20, encadenado=False)
        self.robot.navegacion.giro_absoluto(182, rueda_pivote="derecha")
        self.robot.navegacion.giro_absoluto(180, rueda_pivote="derecha")

        # Juntar
        self.robot.garra_delantera.ir_a_porcentaje(50, wait_after=False)
        wait(500)
        self.robot.chasis.avanzar_recto(-25)
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(50, wait_after=False)
        self.robot.chasis.avanzar_recto(35, velocidad=300)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)

        # Dejar en matriz
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.mover_motor_izquierdo(300, encadenado=True)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor_color, 5, Color.BLACK, distancia_maxima_cm=12, distancia_extra_cm=7)
        self.robot.navegacion.giro_absoluto(180)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor_color, 0, Color.BLUE, velocidad_escaneo=700)
        self.robot.navegacion.giro_absoluto(180, encadenado=False)
        self.robot.chasis.avanzar_recto(15, encadenado=True)

        self.robot.garra_delantera.ir_a_porcentaje(85)
        
        self.robot.garra_delantera.ir_a_porcentaje_pinza(60) 
        self.robot.chasis.sacudir(iteraciones=4, potencia=60, tiempo_ms=100)
        self.robot.garra_delantera.abrir_al_tope(velocidad=1200, limite_potencia=100) 
        self.robot.chasis.avanzar_recto(-30) 
        self.robot.navegacion.giro_absoluto(0)
        self.robot.garra_trasera.ir_a_porcentaje(0, wait_after=False)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor_color, 80, 1, distancia_extra_cm=0, distancia_inicial_cm=10, lado="izquierda", tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_relativo(-90, encadenado=True)
        self.robot.garra_trasera.ir_a_porcentaje(90, wait_after=False)
        self.robot.chasis.avanzar_recto(-32, velocidad=1000)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=300, potencia=80, angulo_referencia=270)
        self.robot.chasis.avanzar_recto(53)
        self.robot.navegacion.giro_absoluto(0)

        # Recoger amarillos
        self.robot.garra_delantera.ir_a_porcentaje(90)
        self.robot.chasis.avanzar_recto(16)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        
        self.robot.navegacion.giro_absoluto(90)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor_color, 3, Color.WHITE, velocidad_escaneo=150, distancia_extra_cm=2)
        self.robot.navegacion.giro_absoluto(180)
        self.robot.garra_delantera.ir_a_porcentaje(85)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(67)
        self.robot.garra_delantera.ir_a_porcentaje(0)

        self.robot.garra_trasera.ir_a_porcentaje(0, wait_after=False)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.garra_delantera.ir_a_porcentaje(90)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.garra_delantera.abrir_al_tope(limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(13, velocidad=1000)
        
        self.robot.navegacion.giro_absoluto(90, rueda_pivote="derecha")
        self.robot.chasis.avanzar_recto(12)
        self.robot.garra_delantera.ir_a_porcentaje(90)

        self.robot.navegacion.giro_absoluto(320)
        self.robot.garra_delantera.ir_a_porcentaje(60)
        self.robot.chasis.avanzar_recto(12)

        self.robot.garra_delantera.ir_a_porcentaje(90)
        self.robot.navegacion.giro_absoluto(280, rueda_pivote="izquierda")
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(20)

    def _armar_amarillo(self):
        """Rutina de armado para mosaico amarillo (migrada de matriz 2 del otro equipo).
        Matriz de Lesther.
        """
        print("Voltaje Hub:", self.robot.hub.battery.voltage(), "mV")
        print("Ejecutando recorrido de armado para mosaico amarillo...")
        gc.collect()

        # Primera fase: Bloques azules
        self.robot.garra_delantera.subir_al_tope()
        self.robot.garra_delantera.abrir_al_tope()

        self.robot.chasis.avanzar_recto(-9, velocidad=900)
        self.robot.navegacion.giro_relativo(90, max_potencia=65, min_potencia=45, encadenado=True)
        wait(200)

        self.robot.chasis.avanzar_recto(8, velocidad=900, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(
            self.sensor_color,
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
            self.sensor_color,
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
        self._dejar_bloques_matriz()
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

        self._dejar_bloques_matriz2()
        gc.collect()

    def _armar_blanco(self):
        """Rutina de armado para mosaico blanco (pendiente de desarrollo)."""
        pass

    # =========================================================================
    # AUXILIARES DE LÍNEA Y ENTREGA EN MATRIZ (NECESARIO PARA QUE FUNCIONE
    # MATRIZ DE LESTHER)
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
            self.sensor_color,
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

    def _dejar_bloques_matriz(self):
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
            self.sensor_color,
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

    def _dejar_bloques_matriz2(self):
        """Secuencia auxiliar de entrega en matriz (segunda entrega)."""
        self.robot.chasis.avanzar_recto(8, velocidad=900, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(
            self.sensor_color,
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
            self.sensor_color,
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