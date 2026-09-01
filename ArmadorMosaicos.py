from pybricks.parameters import Color, Stop
from robot import Robot
from pybricks.tools import wait

class ArmadorMosaicos:
    def __init__(self, robot_instancia: Robot, sensor_color, prueba):
        self.robot = robot_instancia
        self.sensor_color = sensor_color
        
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
            self.robot.garra_trasera.ir_a_porcentaje(100)

    def armar(self, numero_mosaico: int):
        rutina_a_ejecutar = self.rutinas.get(numero_mosaico, self._armar_verde_verde)
        print(f"Ejecutando rutina de armado para mosaico: {numero_mosaico}")
        rutina_a_ejecutar()
    
    def _armar_verde_verde(self):
        pass   

    def _armar_verde_amarillo(self):

        # Acomodo
        self.robot.chasis.mover_motor_derecho(-600, encadenado=True)
        self.robot.chasis.avanzar_recto(-2, velocidad=1000, encadenado=True)
        self.robot.chasis.cuadrar_contra_pared(150, potencia=100, angulo_referencia=90)

        # Recoger dos bloques azules
        self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(self.sensor_color, 2, Color.BLACK, distancia_extra_cm=6, encadenado=True, debug=False)
        self.robot.navegacion.giro_absoluto_pd(0)
        self.robot.chasis.avanzar_recto(12, velocidad=200)
        self.robot.garra_delantera.ir_a_porcentaje(90)

        # Dejar dos bloques azules
        self.robot.chasis.avanzar_recto(-25)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(5)
        self.robot.navegacion.giro_absoluto_motor_derecho(270)

        # Recoger bloque verde
        self.robot.garra_delantera.ir_a_porcentaje_pinza(70, wait_after=False)
        self.robot.navegacion.avanzar_contando_lineas(self.sensor_color, 2, Color.BLACK, tiempo_ciego_s=0.3, velocidad=-1000, velocidad_lenta=-150)
        self.robot.navegacion.giro_absoluto_pd(0)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.chasis.avanzar_recto(2, velocidad=100)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=80)

        # Dejar bloque verde
        self.robot.chasis.avanzar_recto(-4)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.navegacion.giro_absoluto_pd(270)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(0)
        self.robot.chasis.avanzar_recto(12, velocidad=100)

    def _armar_azul(self):
        # Acomodo
        self.robot.chasis.mover_motor_derecho(-600, encadenado=True)
        self.robot.chasis.avanzar_recto(-2, velocidad=1000, encadenado=True)
        self.robot.chasis.cuadrar_contra_pared(150, potencia=100, angulo_referencia=90)

        # Recoger dos bloques azules y dos amarillos
        self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=True)
        self.robot.navegacion.avanzar_contando_lineas(self.sensor_color, 1, Color.BLACK, distancia_extra_cm=13.8, encadenado=False, debug=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(67, velocidad=1000, wait_after=False)
        self.robot.navegacion.giro_absoluto_pd(0)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(10, velocidad=80)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)

        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)

        # Dejar dos bloques azules y dos amarillos
        self.robot.chasis.avanzar_recto(-27, velocidad=600, encadenado=False)
        self.robot.navegacion.giro_absoluto_pd(90, encadenado=False)
        self.robot.chasis.avanzar_recto(10, velocidad=200, encadenado=False)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(0, velocidad=1000)
        self.robot.garra_delantera.ir_a_porcentaje(80, wait_after=False)
        self.robot.chasis.avanzar_recto(4)
        self.robot.chasis.avanzar_recto(-4)

        # Agarrar dos bloques azules
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)
        self.robot.chasis.avanzar_recto(-2)
        self.robot.navegacion.giro_absoluto_pd(0, encadenado=False)
        self.robot.chasis.avanzar_recto(36, encadenado=False)
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=True)
        
        # Dejar dos bloques azules
        self.robot.navegacion.giro_absoluto_motor_derecho(90)
        self.robot.chasis.avanzar_recto(20, encadenado=False)
        self.robot.navegacion.giro_absoluto_motor_izquierdo(182)
        self.robot.navegacion.giro_absoluto_motor_izquierdo(180)

        # Juntar
        self.robot.garra_delantera.ir_a_porcentaje(50, wait_after=False)
        wait(500)
        self.robot.chasis.avanzar_recto(-25)
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=False)
        self.robot.chasis.avanzar_recto(35, velocidad=300)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)

        # Dejar en matriz
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.mover_motor_izquierdo(300, encadenado=True)
        self.robot.navegacion.avanzar_tiempo_luego_color(self.sensor_color, 0.2, Color.BLACK, distancia_extra_cm=7)
        self.robot.navegacion.giro_absoluto_pd(180)
        self.robot.navegacion.avanzar_tiempo_luego_color(self.sensor_color, tiempo_ciego_s=0, color_objetivo=Color.BLUE, encadenado=True)
        self.robot.navegacion.giro_absoluto_pd(180, encadenado=False)
        self.robot.chasis.avanzar_recto(14, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje(70)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(70)
        
        # Sacudir uwu
        #self.__sacudir()

        # Acomodarse para amarillos
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(-30)

        # NUEVO: cuadrar contra pared otra vez
        self.robot.navegacion.giro_absoluto_pd(0)
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(self.sensor_color, 80, 1, 0, 10, lado="izquierda", tiempo_acomodo_ms=0)
        self.robot.navegacion.giro_preciso_pd(-90, encadenado=True)
        self.robot.chasis.avanzar_recto(-32, velocidad=1000)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=100, potencia=80, angulo_referencia=270)
        self.robot.chasis.avanzar_recto(55)
        self.robot.navegacion.giro_absoluto_pd(0)

        # Recoger amarillos
        self.robot.garra_delantera.ir_a_porcentaje(90)
        self.robot.chasis.avanzar_recto(15)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        
        self.robot.navegacion.giro_absoluto_pd(90)
        self.robot.navegacion.avanzar_tiempo_luego_color(self.sensor_color, 0.3, Color.WHITE, distancia_extra_cm=2)
        self.robot.navegacion.giro_absoluto_pd(180)
        self.robot.garra_delantera.ir_a_porcentaje(90)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(67)
        self.robot.garra_delantera.ir_a_porcentaje(20)

        self.robot.chasis.avanzar_recto(-5)
        self.robot.navegacion.giro_absoluto_pd(0, kp=3, kd=20)
        self.robot.garra_delantera.ir_a_porcentaje(90)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        self.robot.chasis.avanzar_recto(-14)
        self.robot.garra_delantera.abrir_al_tope(limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(18, velocidad=1000)
        
        self.robot.navegacion.giro_preciso_pd(38)
        self.robot.chasis.avanzar_recto(9)
        self.robot.garra_delantera.ir_a_porcentaje(90)

        self.robot.navegacion.giro_absoluto_pd(280)
        self.robot.garra_delantera.ir_a_porcentaje(0)
        self.robot.chasis.avanzar_recto(20)

    def _armar_amarillo(self):
        pass

    def _armar_blanco(self):
        pass
    
    def __sacudir(self):
        self.robot.garra_delantera.ir_a_porcentaje(85)
        self.robot.chasis.sacudir(iteraciones=6, potencia=60, tiempo_ms=100)
        # self.robot.garra_delantera.ir_a_porcentaje(70)
        # self.robot.garra_delantera.ir_a_porcentaje_pinza(90)
        # self.robot.garra_delantera.bajar_al_tope(limite_potencia=30, frenado=Stop.COAST)
        # self.robot.chasis.sacudir(iteraciones=8, potencia=80, tiempo_ms=50)