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
            self.robot.chasis.avanzar_recto(distancia_si_verde)
            wait(50) # Micro-pausa para que el sensor lea sin vibraciones del motor
            color_anterior = sensor.color()
            
            self.robot.chasis.avanzar_recto(-distancia_si_verde)
            
            if color_anterior not in decision: 
                return -1
            return decision[color_anterior]
            
        return decision
    
    def agarrar_bloques_blancos(self):
        self.robot.garra_trasera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=300, potencia=80, angulo_referencia=90)
        self.robot.navegacion.giro_absoluto_motor_izquierdo(180, max_speed=1000, min_speed=800, kp=2.5, kd=28, encadenado=True, desaceleracion=500)

        self.robot.garra_trasera.subir(185, velocidad=1000, wait_after=False)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 157, tiempo_acomodo_ms=0, encadenado=True, margen_cm=7)
        self.robot.chasis.mover_motor_izquierdo(300, velocidad=1000, margen_grados=50, encadenado=True)
        self.robot.chasis.avanzar_recto(12, 1000, margen_cm=7, encadenado=True)
        self.robot.navegacion.giro_absoluto_pd(2, max_speed=200, min_speed=100, kp=3.5, kd=22)
        self.__recoger_bloques(25, 100, bajar=160)

    def detectar_mosaico(self):
        self.robot.chasis.avanzar_recto(7, velocidad=1000, margen_cm=7, encadenado=True)
        
        self.robot.navegacion.giro_absoluto_motor_izquierdo(55, max_speed=600, min_speed=500, kp=7, kd=24)
        
        # Termina sobre la linea negra, no sobre una distancia integrada. Los 52 cm
        # ciegos pasan de largo las lineas intermedias sin siquiera mirar el sensor;
        # la ventana de busqueda es de 5 cm para que no agarre la linea equivocada.
        self.robot.navegacion.avanzar_distancia_luego_color(
            self.sensor,
            distancia_ciega_cm=52,
            color_objetivo=Color.BLACK,
            distancia_maxima_cm=57,
            velocidad_escaneo=250,
            encadenado=True
        )

        self.robot.chasis.mover_motor_derecho(370, velocidad=1000, encadenado=True)
        
        self.robot.navegacion.seguidor_linea_color(
            self.sensor, 
            velocidad_max=100, 
            color_objetivo=Color.GREEN, 
            lado="derecha", 
            distancia_cm=10, 
            tiempo_acomodo_ms=0, 
            encadenado=True
        )
        
        self.robot.chasis.mover_motor_derecho(90, velocidad=1000)
        
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=150, kp=3.0, kd=26.0)
        
        self.robot.chasis.avanzar_recto(14, 1000)
        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        return mosaico

    def dejar_bloques_blancos(self):
        self.robot.chasis.avanzar_recto(-17, encadenado=False)
        self.robot.navegacion.giro_absoluto_pd(228, max_speed=400, ruta_corta=False, encadenado=False)
        self.robot.chasis.avanzar_recto(-20, velocidad=1000, margen_cm=7)
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=1000, wait_after=False)

    def agarrar_bloques_verdes(self):
        self.robot.chasis.avanzar_recto(9, velocidad=300, margen_cm=2, encadenado=True)
        self.robot.chasis.mover_motor_derecho(300, velocidad=300, encadenado=True)
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 40, tiempo_acomodo_ms=0, encadenado=False, margen_cm=7)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 40, 20, tiempo_acomodo_ms=0, encadenado=False, margen_cm=7)
        
        # self.robot.chasis.mover_motor_derecho(110, velocidad=1000, margen_grados=50, encadenado=True)
        
        # self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=True)
        
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=120, kp=3.0, kd=26.0)
        
        self.__recoger_bloques(13, wait_ms=300)
    
    def dejar_bloques_verdes(self):
        self.robot.garra_trasera.soltar()
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=400, potencia=70)
        
        # La garra baja en segundo plano mientras el chasis empieza la secuencia de escape
        self.robot.garra_trasera.ir_a_porcentaje(95, velocidad=1000, wait_after=False)
        
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=150, kp=3.0, kd=26.0, encadenado=True)             
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 10, lado="izquierda", tiempo_acomodo_ms=0, encadenado=True)
        
        self.robot.navegacion.giro_absoluto_pd(180, max_speed=800, min_speed=150, kp=3.0, kd=28.0, encadenado=True)             
        
        self.robot.chasis.avanzar_recto(-52, velocidad=1000, encadenado=True, margen_cm=7)
        
        # Tope final por detección de corriente (asegura que guarda la garra por completo)
        self.robot.garra_trasera.subir_al_tope(1000, limite_potencia=100)
    
    def agarrar_bloques_amarillos(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 28, tiempo_acomodo_ms=150, encadenado=True)
        
        self.robot.chasis.mover_motor_derecho(500, velocidad=600, margen_grados=50, encadenado=True)
        
        self.robot.navegacion.avanzar_tiempo_luego_color(
            self.sensor, 
            tiempo_ciego_s=0.3, 
            color_objetivo=Color.BLACK, 
            distancia_extra_cm=4, 
            velocidad_alta=1000, 
            velocidad_escaneo=200, 
            encadenado=True
        )
        
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=120, kp=3.0, kd=26.0) 
        
        self.__recoger_bloques(18, 150)

    def dejar_bloques_amarillos(self):
        self.robot.chasis.girar_sobre_eje(-60, encadenado=True)
        
        self.robot.chasis.avanzar_recto(85, 1000, margen_cm=5, encadenado=True)

        self.robot.navegacion.giro_absoluto_motor_izquierdo(
            0, 
            max_speed=1000, 
            min_speed=400, 
            kp=2.5, 
            kd=12.0, 
            encadenado=True,
            desaceleracion=500
        )
        
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(
            self.sensor,
            velocidad_max=100, 
            cruces_objetivo=1, 
            distancia_extra_cm=16, 
            distancia_inicial_cm=20,
            lado="izquierda", 
            tiempo_acomodo_ms=0,
            encadenado=True
        )
        
        self.robot.navegacion.giro_absoluto_motor_izquierdo(
            270, 
            max_speed=1000, 
            min_speed=500, 
            kp=2.5, 
            kd=25.0, 
            encadenado=False
        )
        
        # Escape final de la zona de descarga mientras la jaula trasera sube de forma asíncrona
        self.robot.garra_trasera.ir_a_porcentaje(30, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-8, velocidad=1000, encadenado=False)

    def cemento_y_llana(self):
        self.robot.garra_delantera.establecer_cero(velocidad=1000, limite_potencia=30)
        self.robot.garra_delantera.establecer_cero_pinza(velocidad=1000, limite_potencia=30)
        self.robot.garra_delantera.ir_a_porcentaje_pinza(70, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.chasis.avanzar_recto(14, velocidad=50, margen_cm=2, encadenado=True)
        
        # AGARRAR CEMENTO
        self.robot.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.garra_delantera.ir_a_porcentaje(60, velocidad=200, wait_after=False)

        # DEJAR LLANA
        self.robot.navegacion.giro_absoluto_pd(160, max_speed=800, min_speed=600, kp=2.0, kd=35.0, encadenado=False)
        self.robot.garra_trasera.ir_a_porcentaje(96, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-50, velocidad=1000, margen_cm=7)
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(5, velocidad=600, margen_cm=3, encadenado=True)

        # AGARRAR PALA
        self.robot.navegacion.giro_absoluto_pd(155, max_speed=800, min_speed=600, kp=2.0, kd=35.0, encadenado=False)
        self.robot.navegacion.avanzar_tiempo_luego_color(
            self.sensor, 
            tiempo_ciego_s=0.8,
            color_objetivo=Color.BLACK, 
            distancia_extra_cm=0, 
            velocidad_alta=1000, 
            velocidad_escaneo=400, 
            encadenado=True
        )
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(
            sensor_color = self.sensor,
            velocidad_max=100,
            cruces_objetivo=1,
            distancia_inicial_cm=10,
            distancia_extra_cm=17,
            tiempo_acomodo_ms=300,
            margen_cm=3,
            encadenado=True
        )

        self.robot.chasis.giro_preciso(-70, encadenado=True)

        self.robot.chasis.avanzar_recto(-12, wait_after=False, margen_cm=5, encadenado=False)
        wait(800)
        self.robot.garra_delantera.ir_a_porcentaje(90, wait_after=False)
        self.robot.garra_trasera.ir_a_porcentaje(90)

        # DEJAR PALA Y CEMENTO
        self.robot.garra_delantera.ir_a_porcentaje_pinza(20, velocidad=1000, wait_after=False)
        self.robot.garra_delantera.ir_a_porcentaje(0, velocidad=1000, wait_after=False)
        wait(200)
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            distancia_total_cm=25,
            distancia_accion_cm=10,
            accion_callback= lambda: self.robot.garra_trasera.subir(50, velocidad=1000),
            margen_cm=2
        )
        self.robot.navegacion.giro_absoluto_pd(356, max_speed=800, min_speed=120, kp=3.0, kd=26.0) 
        self.robot.chasis.avanzar_recto(-10, velocidad=200, wait_after=True, encadenado=True)
        
    def agarrar_bloques_azules(self):
        self.__recoger_bloques(35, 300)

    def dejar_bloques_azules_y_pala(self):
        self.robot.navegacion.curva_coordenada_local(40, -15)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 130, lado="izquierda", encadenado=True)

    def __recoger_bloques(self, distancia, wait_ms=200, bajar=165):
        self.robot.chasis.avanzar_recto(-distancia, 1000, wait_after=False)
        wait(wait_ms)
        self.robot.garra_trasera.ir_a_porcentaje(97, velocidad=600)