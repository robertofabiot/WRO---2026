from pybricks.parameters import Color, Stop
from robot import Robot

class ArmadorMosaicos:
    def __init__(self, robot_instancia: Robot, sensor_color, prueba):
        self.robot = robot_instancia
        self.sensor_color = sensor_color
        
        # --- PATRÓN ESTRATEGIA (Diccionario de Rutinas) ---
        self.rutinas = {
            1: self._armar_verde_verde,
            2: self._armar_verde_amarillo,
            3: self._armar_azul,
            4: self._armar_amarillo,
            5: self._armar_blanco
        }

        if prueba==True:
            self.robot.garra_trasera.establecer_cero()
            self.robot.garra_delantera.establecer_cero()
            self.robot.garra_delantera.establecer_cero_pinza()
            self.robot.garra_trasera.ir_a_porcentaje(97)

    def armar(self, numero_mosaico: int):
        rutina_a_ejecutar = self.rutinas.get(numero_mosaico, self._armar_verde_verde)
        print(f"Ejecutando rutina de armado para mosaico: {numero_mosaico}")
        rutina_a_ejecutar()

    # --- RUTINAS PRIVADAS ---
    def _armar_verde_verde(self):
        pass   

    def _armar_verde_amarillo(self):
        
        self.robot.chasis.mover_motor_derecho(-600, encadenado=True)
        self.robot.chasis.avanzar_recto(-2, velocidad=1000, encadenado=True)
        self.robot.chasis.cuadrar_contra_pared(150, potencia=100, angulo_referencia=90)

        self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(self.sensor_color, 2, Color.BLACK, distancia_extra_cm=5, encadenado=True, debug=False)

        self.robot.navegacion.giro_absoluto_pd(0)
        self.robot.chasis.avanzar_recto(12, velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje(90)

        self.robot.chasis.avanzar_recto(-25)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(5)
        self.robot.navegacion.giro_absoluto_motor_derecho(270)

        self.robot.garra_delantera.ir_a_porcentaje_pinza(70, wait_after=False)
        self.robot.navegacion.avanzar_contando_lineas(self.sensor_color, 2, Color.BLACK, tiempo_ciego_s=0.2, velocidad=-1000, velocidad_lenta=-150)
        self.robot.navegacion.giro_absoluto_pd(0)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.chasis.avanzar_recto(5, velocidad=1000)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=80)

    def _armar_azul(self):
        pass

    def _armar_amarillo(self):
        pass

    def _armar_blanco(self):
        pass