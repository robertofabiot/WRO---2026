from pybricks.parameters import Color
from pybricks.tools import wait
import config
from robot import Robot

class Misiones:
    """Las misiones del recorrido, una por metodo y en orden de ejecucion.

    Cada misión arranca donde termina la anterior, asi que no son
    independientes: para probar una sola hay que dejar el robot en la posicion
    en la que la anterior lo dejaria.
    """

    def __init__(self, robot : Robot, sensor_frente):
        """
        Argumentos:
            robot: instancia de Robot con el chasis y los mecanismos armados.
            sensor_frente: ColorSensor delantero, el que lee lineas y mosaicos.
        """
        self.robot = robot
        self.sensor = sensor_frente

    def _identificar_combinacion(self, distancia_verificacion_cm):
        """Lee el mosaico que toca armar y devuelve su numero.

        La tabla esta en config.MOSAICOS. El verde aparece en dos mosaicos
        distintos, asi que cuando sale verde hay que avanzar a leer la celda
        de al lado para desempatar, y despues volver.

        Argumentos:
            distancia_verificacion_cm: centimetros a avanzar para leer la segunda
                celda cuando la primera sale verde.

        Devuelve el numero de mosaico (1 a 5), o -1 si la lectura no cierra.
        """
        color_principal = self.sensor.color()
        if color_principal not in config.MOSAICOS: 
            return -1  
            
        decision = config.MOSAICOS[color_principal]
        
        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_verificacion_cm)
            wait(50) # Micro-pausa para que el sensor lea sin vibraciones del motor
            color_anterior = self.sensor.color()
            
            self.robot.chasis.avanzar_recto(-distancia_verificacion_cm)
            
            if color_anterior not in decision: 
                return -1
            return decision[color_anterior]
            
        return decision

    def _recoger_con_jaula_trasera(self, distancia_cm, tiempo_espera_ms=200):
        """Retrocede sobre las piezas y baja la jaula para encerrarlas.

        El chasis no espera terminar el retroceso: la jaula empieza a bajar en
        pleno movimiento.

        Argumentos:
            distancia_cm: centimetros a retroceder sobre las piezas.
            tiempo_espera_ms: milisegundos a esperar antes de bajar la jaula.
                Regula en que punto del retroceso cae la jaula.
        """
        self.robot.chasis.avanzar_recto(-distancia_cm, 1000, wait_after=False)
        wait(tiempo_espera_ms)
        self.robot.garra_trasera.ir_a_porcentaje(97, velocidad=600)

    def cemento_y_llana(self):
        """Agarra el cemento con la jaula trasera y deja la llana en su zona."""
        # Agarrar cemento
        self.robot.navegacion.giro_relativo(90, rueda_pivote="derecha", max_potencia=100, min_potencia=80, encadenado=True)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor, 100, 2, distancia_extra_cm=0, distancia_inicial_cm=35, tiempo_acomodo_ms=0, encadenado=True)
        self.robot.navegacion.giro_relativo(-90)
        self.robot.garra_trasera.ir_a_porcentaje(100, wait_after=False)
        self.robot.chasis.avanzar_recto(-10, encadenado=True)

        # Dejar llana
        self.robot.navegacion.giro_relativo(90, encadenado=True)
        self.robot.navegacion.desplazar_lateral(4, encadenado=True)
        self.robot.chasis.avanzar_recto(-50)

    def recoger_pala(self):
        """Agarra la pala con la pinza delantera y la levanta para transportarla."""
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(60, wait_after=False)
        self.robot.chasis.avanzar_recto(100, encadenado=False)
        self.robot.garra_delantera.cerrar_al_tope(limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=False)

    def dejar_cemento(self):
        """Deja el cemento en su zona y queda encarado para lo que sigue."""
        self.robot.navegacion.giro_relativo(90, rueda_pivote="izquierda", min_potencia=70)
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            distancia_total_cm=-30,
            distancia_accion_cm=0,
            accion_callback=lambda: self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000, wait_after=False),
            accion_secundaria_callback=lambda: self.robot.garra_delantera.ir_a_porcentaje_pinza(20, velocidad=1000, wait_after=False),
            retraso_secundaria_ms=600,  
            margen_cm=2
        )
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=1200)
        self.robot.navegacion.giro_relativo(175)
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=1000, wait_after=False)
        self.robot.navegacion.giro_relativo(5)

    def recoger_verdes(self):
        """Busca los bloques verdes por color, los encierra y cuadra contra la pared."""
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, distancia_ciega_cm=0, color_objetivo=Color.WHITE, distancia_maxima_cm=10, velocidad_escaneo=900, encadenado=True)
        self.robot.navegacion.avanzar_distancia_luego_color(self.sensor, distancia_ciega_cm=2, color_objetivo=Color.BLACK, distancia_maxima_cm=10, velocidad_escaneo=150, distancia_extra_cm=9)
        self.robot.navegacion.giro_relativo(-90)
        self._recoger_con_jaula_trasera(distancia_cm=65, tiempo_espera_ms=800)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=200, potencia=100)

    def detectar_mosaico(self):
        """Va hasta la matriz, la escanea y devuelve el numero de mosaico a armar.

        Devuelve el numero de mosaico (1 a 5), o -1 si la lectura no cierra.
        """
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=0, distancia_cm=70)
        self.robot.navegacion.desplazar_lateral(-2.5, encadenado=True)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.chasis.avanzar_recto(10)
        return self._identificar_combinacion(distancia_verificacion_cm=5)
    
    def dejar_verdes(self):
        """Deja los bloques verdes abriendo la jaula trasera en su zona."""
        self.robot.chasis.avanzar_recto(-35)
        self.robot.navegacion.giro_relativo(180)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.garra_trasera.ir_a_porcentaje(0, wait_after=False)
    
    def agarrar_amarillos(self):
        """Sigue la linea hasta los bloques amarillos y se mete a buscarlos."""
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=0, distancia_cm=70)
        self.robot.chasis.avanzar_recto(2)
        self.robot.navegacion.giro_absoluto(90, rueda_pivote="izquierda", encadenado=True)
        self.robot.chasis.avanzar_recto(28)
    
    def agarrar_azules(self):
        """Cruza en diagonal hasta los bloques azules y los encierra con la jaula."""
        self.robot.navegacion.giro_absoluto(291)
        self.robot.chasis.avanzar_recto(62)
        self.robot.navegacion.giro_absoluto(0)
        self._recoger_con_jaula_trasera(distancia_cm=25, tiempo_espera_ms=300)

    def dejar_amarillos(self):
        """Deja los bloques amarillos en su zona siguiendo la linea de la izquierda."""
        self.robot.chasis.avanzar_recto(15)
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=False)
        self.robot.navegacion.desplazar_lateral(-15, encadenado=True)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(60, wait_after=False)
        self.robot.navegacion.seguidor_linea_cruces(self.sensor, 100, 2, distancia_extra_cm=10, distancia_inicial_cm=10, lado="izquierda", tiempo_acomodo_ms=0)
        self.robot.garra_delantera.ir_a_porcentaje(0, wait_after=True)
        self.robot.navegacion.giro_absoluto(90)
        self.robot.chasis.avanzar_recto(20)
    
    def dejar_pala(self):
        """Vuelve a la zona azul a dejar la pala, cerrando el recorrido."""
        self.robot.chasis.avanzar_recto(-20)
        self.robot.navegacion.giro_absoluto(0)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.BLUE, lado="izquierda", tiempo_acomodo_ms=0, distancia_cm=70, distancia_maxima_cm=80)