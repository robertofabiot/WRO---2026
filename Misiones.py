from pybricks.parameters import Stop, Color
from pybricks.tools import wait
import config
from robot import Robot
import Utils

class Misiones:
    def __init__(self, robot : Robot, sensor_frente):
        self.robot = robot
        self.sensor = sensor_frente

    def _identificar_combinacion(self, sensor, distancia_si_verde):
        color_principal = sensor.color()
        if color_principal not in config.MOSAICOS: 
            return -1  
            
        decision = config.MOSAICOS[color_principal]
        
        # Si la decisión es un diccionario, significa que leyó Verde y necesita desempatar
        if type(decision) is dict:
            # 1. Avanza para buscar el color secundario
            self.robot.chasis.avanzar_recto(distancia_si_verde)
            wait(50) # Micro-pausa para que el sensor lea sin vibraciones del motor
            color_anterior = sensor.color()
            
            # 2. Restaura la posición exacta retrocediendo la misma distancia
            self.robot.chasis.avanzar_recto(-distancia_si_verde)
            
            # 3. Evalúa la lectura secundaria
            if color_anterior not in decision: 
                return -1
            return decision[color_anterior]
            
        # Si no era verde, devuelve el color directamente y se queda en su lugar original
        return decision
    
    def agarrar_bloques_blancos(self):
        self.robot.garra_trasera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=300, potencia=80, angulo_referencia=90)
        self.robot.navegacion.giro_absoluto_motor_izquierdo(180, min_speed=800,encadenado=True)
        self.robot.garra_trasera.subir(185, velocidad=1000, wait_after=False)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 157, tiempo_acomodo_ms=0, encadenado=True, margen_cm=7)
        self.robot.chasis.mover_motor_izquierdo(300, velocidad=1000, margen_grados=50, encadenado=True)
        self.robot.chasis.avanzar_recto(12, 1000, margen_cm=7, encadenado=True)
        self.robot.navegacion.giro_absoluto_pd(2, max_speed=200) # Cierra el combo fluido
        self.__recoger_bloques(25, 100, bajar=160)

    def detectar_mosaico(self):
        self.robot.chasis.avanzar_recto(7, velocidad=1000, margen_cm=7, encadenado=True)
        self.robot.chasis.mover_motor_izquierdo(300, velocidad=600, encadenado=True)
        self.robot.chasis.avanzar_recto(51, velocidad=1000, margen_cm=7, encadenado=True)        
        self.robot.chasis.mover_motor_derecho(370, velocidad=1000, encadenado=True)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 70, Color.GREEN, lado="derecha", distancia_cm=10, encadenado=True)
        self.robot.chasis.mover_motor_derecho(90, velocidad=1000)
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=400)
        self.robot.chasis.avanzar_recto(14, 1000)
        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        return mosaico

    def dejar_bloques_blancos(self):
        self.robot.chasis.avanzar_recto(-17, encadenado=False)
        self.robot.navegacion.giro_absoluto_pd(228, max_speed=400, ruta_corta=False, encadenado=False)
        self.robot.chasis.avanzar_recto(-16, velocidad=1000, margen_cm=7)
        self.robot.garra_trasera.subir(185, velocidad=1000, wait_after=False)

    def agarrar_bloques_verdes(self):
        self.robot.chasis.avanzar_recto(12, velocidad=1000)
        self.robot.chasis.mover_motor_derecho(300, velocidad=1000, encadenado=True)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 45, tiempo_acomodo_ms=150, encadenado=True, margen_cm=7)
        self.robot.chasis.mover_motor_derecho(110, velocidad=600, margen_grados=50, encadenado=True)
        self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=False)
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=400)
        self.__recoger_bloques(14, wait_ms=100 ,bajar=185)
    
    def dejar_bloques_verdes(self):
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=400, potencia=70)
        self.robot.garra_trasera.ir_a_porcentaje(95, velocidad=1000, wait_after=False)
        self.robot.navegacion.giro_absoluto_pd(0, encadenado=True)             
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 10, lado="izquierda", tiempo_acomodo_ms=100, encadenado=True)
        self.robot.navegacion.giro_absoluto_pd(180, encadenado=True)             
        self.robot.chasis.avanzar_recto(-50, velocidad=1000, encadenado=True, margen_cm=7)
        self.robot.garra_trasera.subir_al_tope(1000, limite_potencia=100)
    
    def agarrar_bloques_amarillos(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 28, encadenado=True)
        self.robot.chasis.mover_motor_derecho(500, velocidad=600, margen_grados=50, encadenado=True)
        self.robot.navegacion.avanzar_tiempo_luego_color(self.sensor, 0.3, Color.BLACK, distancia_extra_cm=8, velocidad_escaneo=150)
        self.robot.drive_base.brake()
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=200) # Cierra el combo fluido
        self.__recoger_bloques(27, 100, bajar=185)

    def dejar_bloques_amarillos(self):
        self.robot.chasis.girar_sobre_eje(-60)
        self.robot.chasis.avanzar_recto(77, 1000)
        self.robot.navegacion.giro_absoluto_motor_izquierdo(0)
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(
            self.sensor, 
            velocidad_max=100, 
            cruces_objetivo=2, 
            distancia_extra_cm=15, 
            distancia_inicial_cm=12,  # Ignora lecturas negras los primeros 12cm
            lado="izquierda", 
            tiempo_acomodo_ms=150,    # Estabilización agresiva de 150ms en lugar de 500ms
            encadenado=False
        )
        self.robot.navegacion.giro_absoluto_motor_izquierdo(270, max_speed=1000, min_speed=1000, encadenado=False)
        self.robot.garra_trasera.subir(185, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-8, velocidad=1000, encadenado=False)

    def cemento_y_llana(self):
        self.robot.chasis.avanzar_recto(13, velocidad=1000, encadenado=False)
        self.robot.navegacion.giro_absoluto_pd(90, encadenado=True)
        self.robot.garra_trasera.mover(-185, velocidad=500, wait_after=False)
        
        self.robot.chasis.avanzar_recto(-7.5, velocidad=1000, margen_cm=2, encadenado=True)
        self.robot.chasis.mover_motor_derecho(-100, velocidad=1000, encadenado=True)
        self.robot.navegacion.giro_absoluto_pd(172, min_speed=600, encadenado=True)
        self.robot.chasis.avanzar_recto(-37, velocidad=1000, encadenado=False)
        self.robot.navegacion.giro_absoluto_pd(160, min_speed=400, encadenado=True)
        self.robot.navegacion.avanzar_tiempo_luego_color(self.sensor, 0.5, Color.BLACK, encadenado=True)
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(self.sensor, 100, 2, 22, distancia_inicial_cm=7, tiempo_acomodo_ms=300, encadenado=True)
        self.robot.chasis.mover_motor_derecho(440, velocidad=1000, margen_grados=50, encadenado=True)
        self.robot.garra_trasera.subir(100, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-13, velocidad=1000, encadenado=True)
        self.robot.navegacion.giro_absoluto_motor_derecho(175, min_speed=300, encadenado=True)
        self.robot.garra_trasera.bajar(100, velocidad=1000)
        self.robot.chasis.mover_motor_derecho(400, velocidad=1000, encadenado=True)
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            20, 
            15,  
            lambda: self.robot.garra_trasera.subir(185, velocidad=1000, wait_after=False),
            encadenado=True)

    def agarrar_bloques_azules(self):
        self.robot.chasis.mover_motor_izquierdo(630, velocidad=400, encadenado=True)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 10, lado="izquierda", tiempo_acomodo_ms=100, encadenado=True)
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=200, encadenado=True)
        self.__recoger_bloques(27, 100, bajar=185)

    def dejar_bloques_azules_y_pala(self):
        self.robot.navegacion.curva_coordenada_local(40, -15)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 130, lado="izquierda", encadenado=True)

    def __recoger_bloques(self, distancia, wait_ms=200, bajar=165):
        self.robot.chasis.avanzar_recto(-distancia, 1000, wait_after=False)
        wait(wait_ms)
        self.robot.garra_trasera.ir_a_porcentaje(95, velocidad=1000)