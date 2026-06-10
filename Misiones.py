from pybricks.parameters import Stop, Color
from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot # Importado para el Type Hinting
import config # Asegúrate de importar config al inicio de tu archivo

class Misiones:
    def __init__(self, robot: Robot, sensor_frente: ColorSensor):
        self.robot = robot
        self.sensor = sensor_frente
        self.sensor_trasero = sensor_frente


    

    def _identificar_combinacion(self, sensor: ColorSensor, distancia_si_verde):
        color_principal = sensor.color()
        if color_principal not in config.MOSAICOS:
            return -1
        decision = config.MOSAICOS[color_principal]
        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_si_verde)
            color_anterior = sensor.color()
            if color_anterior not in decision: return -1
            return decision[color_anterior]
        return decision

    def pruebas_matrices(self):


        #Avanza como 67cm, gira a la izquierda, avanza 13cm, baja la garra y cuerra la garra. 
        # Retrocede 13cm, gira hacia la izquierda, avanza 60cm en el seguidor, abre la garra 
        # y deja los bloques en un lugar especifico, retrocede y gira hacia la derecha y avanza 13cm,
        # baja y cierra la garra,  

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=63, tiempo_acomodo_ms=0)

        self.robot.navegacion.giro_preciso_pd(-91)

        self.robot.mecanismos.elevador_delantero.mover(-270)

        self.robot.chasis.avanzar_recto(14)

        self.robot.mecanismos.garra_delantera.abrir_al_tope(velocidad=1000)
    
        self.robot.mecanismos.elevador_delantero.mover(270)

        self.robot.chasis.avanzar_recto(-10)

        self.robot.navegacion.giro_preciso_pd(-90)

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=58, tiempo_acomodo_ms=200)

        self.robot.mecanismos.elevador_delantero.mover(-270)

        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000)

        self.robot.chasis.avanzar_recto(-10)

        self.robot.navegacion.giro_preciso_pd(90)



    
    def cemento_y_llana(self):
# Preparación inicial
        self.robot.mecanismos.garra_delantera.llevar_al_tope("positivo", velocidad=1000, limite_potencia=60)

        # 1. ARRANQUE FLUIDO: Aprovechamos la inercia sin frenar (Stop.NONE)
        self.robot.chasis.mover_en_arco(radio_cm=14, distancia_cm=17, stop=Stop.NONE, margen_cm=3)

        # 2. SEGUIDOR BLINDADO: Ya tiene el Control de Tracción y frenado exacto en grados por defecto.
        self.robot.navegacion.seguidor_linea_distancia(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=91, 
            lado="derecha", 
            tiempo_acomodo_ms=0
        )

        self.robot.navegacion.giro_preciso_pd(-90)

        self.robot.mecanismos.garra_trasera.mover(171, velocidad=250, wait_after=False)
        
        self.robot.chasis.avanzar_recto(-8, velocidad=1000, frenado=Stop.HOLD, margen_cm=0)


        self.robot.navegacion.giro_preciso_pd(90)

        self.robot.chasis.avanzar_recto(-27, velocidad=1300, frenado=Stop.NONE, margen_cm=4)
        self.robot.chasis.mover_en_arco(-140, distancia_cm=28, stop=Stop.NONE, margen_cm=1)

        self.robot.chasis.mover_motor_izquierdo(160, wait_after=False)

        self.robot.navegacion.seguidor_linea_distancia(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=50, 
            lado="derecha", 
            tiempo_acomodo_ms=0
        )
        self.robot.navegacion.giro_preciso_pd(90)

    def agarrar_bloques_blancos(self):
        # 1. ARRANQUE ASÍNCRONO: Aceleramos la apertura de garra a 1000 para no perder tiempo
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-5, frenado=Stop.HOLD)
        
        # Encadenamiento fluido (Stop.NONE)
        self.robot.chasis.mover_en_arco(-14.5, distancia_cm=17.5, stop=Stop.NONE, margen_cm=2)
        
        # 2. SEGUIDOR BLINDADO: Aterriza exacto a los 15 cm, el Congelador Dinámico elimina el temblor
        self.robot.navegacion.seguidor_linea_distancia(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=16, 
            lado="derecha", 
            tiempo_acomodo_ms=0, 
            kp=1.2, 
            kd=3.5
        )
        
        # 3. ASENTAMIENTO FÍSICO Y BLINDAJE DEL IMU (Cero crasheos, Cero grados de error)
        self.robot.chasis.drive_base.stop() # <-- EL SALVAVIDAS: Libera el giroscopio
        self.robot.hub.imu.reset_heading(0)
        
        # 4. EL GIRO VIOLENTO 180°
        # Borramos el bloque inmenso de settings. ¡El Caballo de Troya lo hace automático!
        self.robot.navegacion.giro_preciso_pd(180)
        
        # 5. RECOLECCIÓN LETAL
        # Aceleramos la garra a 1000 y atacamos en reversa
        self.robot.mecanismos.garra_trasera.mover(172, velocidad=1500, wait_after=False)
        
        # ARREGLO CRÍTICO: Cambiamos BRAKE por HOLD. 
        # Al embestir los bloques, el chasis se convierte en una pared de concreto. Cero rebotes.
        self.robot.chasis.avanzar_recto(-21, velocidad=150, frenado=Stop.HOLD)
       

    def dejar_bloques_blancos(self):
        
        # 1. PRIMER PIVOTE CONTROLADO
        self.robot.chasis.motor_derecha.hold() 
        self.robot.chasis.mover_motor_izquierdo(400, velocidad=1000, frenado=Stop.HOLD)
        
        # 2. EL SPRINT HACKEADO
        self.robot.chasis.drive_base.settings(
            straight_speed=930,          # Emparejamos a 1000 para máximo poder
            straight_acceleration=1500,   
            turn_rate=config.TURN_RATE,   
            turn_acceleration=config.STRAIGHT_ACCEL 
        )
        
        # ARREGLO CRÍTICO: Frenamos con Stop.HOLD. 
        # Cero derrapes antes del siguiente pivote.
        self.robot.chasis.avanzar_recto(57, velocidad=930, frenado=Stop.HOLD)
        
        # Restauramos aceleración
        self.robot.chasis.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED, 
            straight_acceleration=config.STRAIGHT_ACCEL,
            turn_rate=config.TURN_RATE,
            turn_acceleration=config.STRAIGHT_ACCEL
        )
        
        # 3. SEGUNDO PIVOTE CONTROLADO
        self.robot.navegacion.giro_preciso_pd(-45)
        
        # 4. SEGUIDOR POR COLOR 
        self.robot.navegacion.seguidor_linea_color(
            self.sensor, 
            100, 
            Color.GREEN, 
            lado="derecha", 
            distancia_cm=35
        )
        
        # 5. ACERCAMIENTO FINAL Y MICRO-AJUSTES
        # Frenamos el pivote en seco
        self.robot.chasis.mover_motor_derecho(80, velocidad=800, frenado=Stop.HOLD)        
        
        # EL TRUCO DEL MICRO-MOVIMIENTO: 
        # Para movernos 5 milímetros (-0.5) sin patinar, bajamos la velocidad a 300.
        self.robot.chasis.avanzar_recto(-1, velocidad=300, frenado=Stop.HOLD)

        # 6. ASENTAMIENTO Y GIRO MASIVO (230°)
        # Ritual sagrado para no crashear el giroscopio y lograr exactitud pura
   
        self.robot.chasis.drive_base.stop() 
        self.robot.hub.imu.reset_heading(0)
        
        self.robot.navegacion.giro_preciso_pd(235)       
        
        # 7. ENTREGA LETAL
        # Subimos la velocidad de la garra a 1000 (250 era muy lento) y clavamos el retroceso
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-18, velocidad=1000, frenado=Stop.HOLD)
       

    def agarrar_bloques_verdes(self):
        
        # 1. Arranque explosivo aprovechando inercia
        self.robot.chasis.avanzar_recto(16, velocidad=1000, frenado=Stop.BRAKE)

        # 2. EL ANCLA PERFECTA: Clavamos el motor izquierdo para que el derecho pivotee a vel 1000 sin derrapar
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(370, velocidad=1000, margen_grados=30)
        
        # 3. SEGUIDOR CON CUADRATURA: Reemplazamos el seguidor normal por el de "prueba" 
        # para que se alinee perfectamente al final y podamos BORRAR el wait(300)
        self.robot.navegacion.seguidor_linea_distancia(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=42, 
            lado="izquierda", 
            tiempo_acomodo_ms=0, 
            kp=1.2, 
            kd=3.5
        )
        self.robot.navegacion.giro_preciso_pd(180)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=800, frenado=Stop.HOLD, wait_after=False)
        self.robot.chasis.avanzar_recto(-19, velocidad=1000)
  
  

    def dejar_bloques_verdes_y_detectar_mosaico(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=200, distancia_cm=70)

        self.robot.chasis.mover_motor_derecho(80)
        self.robot.chasis.avanzar_recto(15)

        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        if (mosaico == 1 or mosaico == 2):
            self.robot.chasis.avanzar_recto(-5, velocidad=1000)
            


        self.robot.chasis.avanzar_recto(-20, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(180)
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=100,wait_after=False)
        self.robot.chasis.avanzar_recto(-6, velocidad=1000)
        return mosaico 


    def agarrar_bloques_amarillos(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm=17, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(33, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(45)


        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=13 , tiempo_acomodo_ms=100)
        self.robot.navegacion.giro_preciso_pd(-180)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=300, wait_after=False)
        self.robot.chasis.avanzar_recto(-20)


    def dejar_bloques_amarillos(self):
        self.robot.navegacion.giro_preciso_pd(-55)
        self.robot.chasis.avanzar_recto(distancia_cm=80, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(55)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=55, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=100, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-15.5, velocidad=1000)



    def agarrar_bloques_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(distancia_cm=17, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=47)
        self.robot.navegacion.giro_preciso_pd(-50)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=250, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-10, velocidad=1000)
        self.robot.mecanismos.garra_trasera.avanzar_y_gatillar(chasis=self.robot.chasis, distancia_total_cm=34, vel_chasis=1000, distancia_trigger_cm=15, grados_garra=-170)
        self.robot.navegacion.giro_preciso_pd(55)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor,  velocidad_max=100, distancia_cm=16 ,tiempo_acomodo_ms=100)
        self.robot.navegacion.giro_preciso_pd(-180)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=200, wait_after=False)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.navegacion.giro_preciso_pd(-30)
        self.robot.chasis.avanzar_recto(distancia_cm=36, velocidad=950)
        self.robot.navegacion.giro_preciso_pd(30, margen_grados=10)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=160, lado="izquierda")   


""" --------PARTE DE IDEA OLVIDADA---------
    def agarrar_bloques_amarillos_y_azules(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm=64, lado="izquierda")
        self.robot.chasis.mover_motor_izquierdo(700, frenado=Stop.BRAKE)
        self.robot.mecanismos.garra_trasera.mover(170,velocidad=130, wait_after=False)
        self.robot.chasis.avanzar_recto(-35)
        self.robot.chasis.avanzar_recto(65)
        self.robot.chasis.mover_motor_izquierdo(700)

    def dejar_bloques_amarillos_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(15)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=24 ,margen_cm=30)
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(10)
        self.robot.mecanismos.elevador_delantero.mover(90, wait_after=False)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope()
        self.robot.chasis.avanzar_recto(-10)
        self.robot.navegacion.giro_preciso_pd(45)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=26) 
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.chasis.avanzar_recto(-7)
        self.robot.mecanismos.garra_trasera(-170)
        """


                
    # # # def agarrar_bloques_amarillos(self):
    # # #     self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 25)
    # # #     self.robot.navegacion.giro_preciso_pd(-45)
    # # #     self.robot.chasis.avanzar_recto(25)
    # # #     self.robot.navegacion.giro_preciso_pd(45)
    # # #     self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.YELLOW, distancia_cm=26)
    # # #     self.robot.chasis.avanzar_recto(-10, velocidad=1000)
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
    # # #     self.robot.chasis.giro_preciso(-175)
    # # #     self.robot.chasis.mover_motor_derecho(30)
    # # #     self.robot.chasis.avanzar_recto(-20)
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)

    # # # def dejar_bloques_amarillos(self):
    # # #     self.robot.chasis.mover_motor_derecho(225, velocidad=1000, margen_grados=30)
    # # #     self.robot.chasis.avanzar_recto(60, velocidad=1000, frenado=Stop.NONE)
    # # #     self.robot.chasis.mover_motor_izquierdo(225, velocidad=1000, margen_grados=30)
    # # #     self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 58, lado="izquierda", tiempo_acomodo_ms=800)
    # # #     wait(500)
    # # #     self.robot.navegacion.giro_preciso_pd(-90)
    # # #     self.robot.chasis.avanzar_recto(-17, velocidad=1000)
    # # #     self.robot.mecanismos.garra_trasera.mover(-55)

    # # # def recoger_bloques_azules(self):
    # # #     self.robot.chasis.avanzar_recto(15, velocidad=1000)
    # # #     self.robot.navegacion.giro_preciso_pd(-90)
    # # #     self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 50)
    # # #     self.robot.navegacion.giro_preciso_pd(-45)
    # # #     self.robot.chasis.avanzar_recto(13, velocidad=1000)
    # # #     self.robot.chasis.mover_motor_izquierdo(300)
    # # #     self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.BLUE, distancia_cm=25)
    # # #     self.robot.chasis.avanzar_recto(-10)
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
    # # #     self.robot.chasis.giro_preciso(-175)
    # # #     self.robot.chasis.mover_motor_derecho(30)
    # # #     self.robot.chasis.avanzar_recto(-20)
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)

    # # # def dejar_bloques_azules_y_pala(self):
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)
    # # #     self.robot.navegacion.giro_preciso_pd(-35)
    # # #     self.robot.chasis.avanzar_recto(52, velocidad=1100)
    # # #     self.robot.chasis.mover_motor_izquierdo(210)
    # # #     self.robot.chasis.avanzar_recto(-10)
    # # #     self.robot.chasis.giro_preciso(-180)
    # # #     self.robot.chasis.avanzar_recto(-10)
    # # #     self.robot.chasis.mover_motor_derecho(-150)
    # # #     self.robot.chasis.avanzar_recto(-46)
    # # #     self.robot.chasis.mover_motor_izquierdo(-120)
    # # #     self.robot.chasis.avanzar_recto(-80)
    # # #     self.robot.chasis.avanzar_recto(20, velocidad=1000)
    # # #     self.robot.navegacion.giro_preciso_pd(-90)
    # # #     self.robot.mecanismos.garra_trasera.mover(-55)from pybricks.parameters import Stop, Color
from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot # Importado para el Type Hinting
import config # Asegúrate de importar config al inicio de tu archivo

class Misiones:
    def __init__(self, robot: Robot, sensor_frente: ColorSensor):
        self.robot = robot
        self.sensor = sensor_frente
        self.sensor_trasero = sensor_frente


    

    def _identificar_combinacion(self, sensor: ColorSensor, distancia_si_verde):
        color_principal = sensor.color()
        if color_principal not in config.MOSAICOS:
            return -1
        decision = config.MOSAICOS[color_principal]
        if type(decision) is dict:
            self.robot.chasis.avanzar_recto(distancia_si_verde)
            color_anterior = sensor.color()
            if color_anterior not in decision: return -1
            return decision[color_anterior]
        return decision

    def pruebas_matrices(self):


        #Avanza como 67cm, gira a la izquierda, avanza 13cm, baja la garra y cuerra la garra. 
        # Retrocede 13cm, gira hacia la izquierda, avanza 60cm en el seguidor, abre la garra 
        # y deja los bloques en un lugar especifico, retrocede y gira hacia la derecha y avanza 13cm,
        # baja y cierra la garra,  

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=63, tiempo_acomodo_ms=0)

        self.robot.navegacion.giro_preciso_pd(-91)

        self.robot.mecanismos.elevador_delantero.mover(-270)

        self.robot.chasis.avanzar_recto(14)

        self.robot.mecanismos.garra_delantera.abrir_al_tope(velocidad=1000)
    
        self.robot.mecanismos.elevador_delantero.mover(270)

        self.robot.chasis.avanzar_recto(-10)

        self.robot.navegacion.giro_preciso_pd(-90)

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=58, tiempo_acomodo_ms=200)

        self.robot.mecanismos.elevador_delantero.mover(-270)

        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000)

        self.robot.chasis.avanzar_recto(-10)

        self.robot.navegacion.giro_preciso_pd(90)



    
    def cemento_y_llana(self):
# Preparación inicial
        self.robot.mecanismos.garra_delantera.llevar_al_tope("positivo", velocidad=1000, limite_potencia=60)

        # 1. ARRANQUE FLUIDO: Aprovechamos la inercia sin frenar (Stop.NONE)
        self.robot.chasis.mover_en_arco(radio_cm=14, distancia_cm=17, stop=Stop.NONE, margen_cm=3)

        # 2. SEGUIDOR BLINDADO: Ya tiene el Control de Tracción y frenado exacto en grados por defecto.
        self.robot.navegacion.seguidor_linea_distancia(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=91, 
            lado="derecha", 
            tiempo_acomodo_ms=0
        )

        self.robot.navegacion.giro_preciso_pd(-90)

        self.robot.mecanismos.garra_trasera.mover(171, velocidad=250, wait_after=False)
        
        self.robot.chasis.avanzar_recto(-8, velocidad=1000, frenado=Stop.HOLD, margen_cm=0)


        self.robot.navegacion.giro_preciso_pd(90)

        self.robot.chasis.avanzar_recto(-27, velocidad=1300, frenado=Stop.NONE, margen_cm=4)
        self.robot.chasis.mover_en_arco(-140, distancia_cm=28, stop=Stop.NONE, margen_cm=1)

        self.robot.chasis.mover_motor_izquierdo(160, wait_after=False)

        self.robot.navegacion.seguidor_linea_distancia(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=50, 
            lado="derecha", 
            tiempo_acomodo_ms=0
        )
        self.robot.navegacion.giro_preciso_pd(90)

    def agarrar_bloques_blancos(self):
        # 1. ARRANQUE ASÍNCRONO: Aceleramos la apertura de garra a 1000 para no perder tiempo
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-5, frenado=Stop.HOLD)
        
        # Encadenamiento fluido (Stop.NONE)
        self.robot.chasis.mover_en_arco(-14.5, distancia_cm=17.5, stop=Stop.NONE, margen_cm=2)
        
        # 2. SEGUIDOR BLINDADO: Aterriza exacto a los 15 cm, el Congelador Dinámico elimina el temblor
        self.robot.navegacion.seguidor_linea_distancia(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=16, 
            lado="derecha", 
            tiempo_acomodo_ms=0, 
            kp=1.2, 
            kd=3.5
        )
        
        # 3. ASENTAMIENTO FÍSICO Y BLINDAJE DEL IMU (Cero crasheos, Cero grados de error)
        self.robot.chasis.drive_base.stop() # <-- EL SALVAVIDAS: Libera el giroscopio
        self.robot.hub.imu.reset_heading(0)
        
        # 4. EL GIRO VIOLENTO 180°
        # Borramos el bloque inmenso de settings. ¡El Caballo de Troya lo hace automático!
        self.robot.navegacion.giro_preciso_pd(180)
        
        # 5. RECOLECCIÓN LETAL
        # Aceleramos la garra a 1000 y atacamos en reversa
        self.robot.mecanismos.garra_trasera.mover(172, velocidad=1500, wait_after=False)
        
        # ARREGLO CRÍTICO: Cambiamos BRAKE por HOLD. 
        # Al embestir los bloques, el chasis se convierte en una pared de concreto. Cero rebotes.
        self.robot.chasis.avanzar_recto(-21, velocidad=150, frenado=Stop.HOLD)
       

    def dejar_bloques_blancos(self):
        
        # 1. PRIMER PIVOTE CONTROLADO
        self.robot.chasis.motor_derecha.hold() 
        self.robot.chasis.mover_motor_izquierdo(400, velocidad=1000, frenado=Stop.HOLD)
        
        # 2. EL SPRINT HACKEADO
        self.robot.chasis.drive_base.settings(
            straight_speed=930,          # Emparejamos a 1000 para máximo poder
            straight_acceleration=1500,   
            turn_rate=config.TURN_RATE,   
            turn_acceleration=config.STRAIGHT_ACCEL 
        )
        
        # ARREGLO CRÍTICO: Frenamos con Stop.HOLD. 
        # Cero derrapes antes del siguiente pivote.
        self.robot.chasis.avanzar_recto(57, velocidad=930, frenado=Stop.HOLD)
        
        # Restauramos aceleración
        self.robot.chasis.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED, 
            straight_acceleration=config.STRAIGHT_ACCEL,
            turn_rate=config.TURN_RATE,
            turn_acceleration=config.STRAIGHT_ACCEL
        )
        
        # 3. SEGUNDO PIVOTE CONTROLADO
        self.robot.navegacion.giro_preciso_pd(-45)
        
        # 4. SEGUIDOR POR COLOR 
        self.robot.navegacion.seguidor_linea_color(
            self.sensor, 
            100, 
            Color.GREEN, 
            lado="derecha", 
            distancia_cm=35
        )
        
        # 5. ACERCAMIENTO FINAL Y MICRO-AJUSTES
        # Frenamos el pivote en seco
        self.robot.chasis.mover_motor_derecho(80, velocidad=800, frenado=Stop.HOLD)        
        
        # EL TRUCO DEL MICRO-MOVIMIENTO: 
        # Para movernos 5 milímetros (-0.5) sin patinar, bajamos la velocidad a 300.
        self.robot.chasis.avanzar_recto(-1, velocidad=300, frenado=Stop.HOLD)

        # 6. ASENTAMIENTO Y GIRO MASIVO (230°)
        # Ritual sagrado para no crashear el giroscopio y lograr exactitud pura
   
        self.robot.chasis.drive_base.stop() 
        self.robot.hub.imu.reset_heading(0)
        
        self.robot.navegacion.giro_preciso_pd(235)       
        
        # 7. ENTREGA LETAL
        # Subimos la velocidad de la garra a 1000 (250 era muy lento) y clavamos el retroceso
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-18, velocidad=1000, frenado=Stop.HOLD)
       

    def agarrar_bloques_verdes(self):
        
        # 1. Arranque explosivo aprovechando inercia
        self.robot.chasis.avanzar_recto(16, velocidad=1000, frenado=Stop.BRAKE)

        # 2. EL ANCLA PERFECTA: Clavamos el motor izquierdo para que el derecho pivotee a vel 1000 sin derrapar
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(370, velocidad=1000, margen_grados=30)
        
        # 3. SEGUIDOR CON CUADRATURA: Reemplazamos el seguidor normal por el de "prueba" 
        # para que se alinee perfectamente al final y podamos BORRAR el wait(300)
        self.robot.navegacion.seguidor_linea_distancia(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=42, 
            lado="izquierda", 
            tiempo_acomodo_ms=0, 
            kp=1.2, 
            kd=3.5
        )
        self.robot.navegacion.giro_preciso_pd(180)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=800, frenado=Stop.HOLD, wait_after=False)
        self.robot.chasis.avanzar_recto(-19, velocidad=1000)
  
  

    def dejar_bloques_verdes_y_detectar_mosaico(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="derecha", tiempo_acomodo_ms=200, distancia_cm=70)

        self.robot.chasis.mover_motor_derecho(80)
        self.robot.chasis.avanzar_recto(15)

        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        if (mosaico == 1 or mosaico == 2):
            self.robot.chasis.avanzar_recto(-5, velocidad=1000)
            


        self.robot.chasis.avanzar_recto(-20, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(180)
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=100,wait_after=False)
        self.robot.chasis.avanzar_recto(-6, velocidad=1000)
        return mosaico 


    def agarrar_bloques_amarillos(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm=17, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(33, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(45)


        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=13 , tiempo_acomodo_ms=100)
        self.robot.navegacion.giro_preciso_pd(-180)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=300, wait_after=False)
        self.robot.chasis.avanzar_recto(-20)


    def dejar_bloques_amarillos(self):
        self.robot.navegacion.giro_preciso_pd(-55)
        self.robot.chasis.avanzar_recto(distancia_cm=80, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(55)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=55, lado="izquierda")
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=100, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-15.5, velocidad=1000)



    def agarrar_bloques_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(distancia_cm=17, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=47)
        self.robot.navegacion.giro_preciso_pd(-50)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=250, wait_after=False)
        self.robot.chasis.avanzar_recto(distancia_cm=-10, velocidad=1000)
        self.robot.mecanismos.garra_trasera.avanzar_y_gatillar(chasis=self.robot.chasis, distancia_total_cm=34, vel_chasis=1000, distancia_trigger_cm=15, grados_garra=-170)
        self.robot.navegacion.giro_preciso_pd(55)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor,  velocidad_max=100, distancia_cm=16 ,tiempo_acomodo_ms=100)
        self.robot.navegacion.giro_preciso_pd(-180)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=200, wait_after=False)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.navegacion.giro_preciso_pd(-30)
        self.robot.chasis.avanzar_recto(distancia_cm=36, velocidad=950)
        self.robot.navegacion.giro_preciso_pd(30, margen_grados=10)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=160, lado="izquierda")   


""" --------PARTE DE IDEA OLVIDADA---------
    def agarrar_bloques_amarillos_y_azules(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm=64, lado="izquierda")
        self.robot.chasis.mover_motor_izquierdo(700, frenado=Stop.BRAKE)
        self.robot.mecanismos.garra_trasera.mover(170,velocidad=130, wait_after=False)
        self.robot.chasis.avanzar_recto(-35)
        self.robot.chasis.avanzar_recto(65)
        self.robot.chasis.mover_motor_izquierdo(700)

    def dejar_bloques_amarillos_azules_y_pala(self):
        self.robot.chasis.avanzar_recto(15)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=110, distancia_cm=24 ,margen_cm=30)
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(10)
        self.robot.mecanismos.elevador_delantero.mover(90, wait_after=False)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope()
        self.robot.chasis.avanzar_recto(-10)
        self.robot.navegacion.giro_preciso_pd(45)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, velocidad_max=100, distancia_cm=26) 
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.chasis.avanzar_recto(-7)
        self.robot.mecanismos.garra_trasera(-170)
        """


                
    # # # def agarrar_bloques_amarillos(self):
    # # #     self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 25)
    # # #     self.robot.navegacion.giro_preciso_pd(-45)
    # # #     self.robot.chasis.avanzar_recto(25)
    # # #     self.robot.navegacion.giro_preciso_pd(45)
    # # #     self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.YELLOW, distancia_cm=26)
    # # #     self.robot.chasis.avanzar_recto(-10, velocidad=1000)
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
    # # #     self.robot.chasis.giro_preciso(-175)
    # # #     self.robot.chasis.mover_motor_derecho(30)
    # # #     self.robot.chasis.avanzar_recto(-20)
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)

    # # # def dejar_bloques_amarillos(self):
    # # #     self.robot.chasis.mover_motor_derecho(225, velocidad=1000, margen_grados=30)
    # # #     self.robot.chasis.avanzar_recto(60, velocidad=1000, frenado=Stop.NONE)
    # # #     self.robot.chasis.mover_motor_izquierdo(225, velocidad=1000, margen_grados=30)
    # # #     self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 58, lado="izquierda", tiempo_acomodo_ms=800)
    # # #     wait(500)
    # # #     self.robot.navegacion.giro_preciso_pd(-90)
    # # #     self.robot.chasis.avanzar_recto(-17, velocidad=1000)
    # # #     self.robot.mecanismos.garra_trasera.mover(-55)

    # # # def recoger_bloques_azules(self):
    # # #     self.robot.chasis.avanzar_recto(15, velocidad=1000)
    # # #     self.robot.navegacion.giro_preciso_pd(-90)
    # # #     self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 50)
    # # #     self.robot.navegacion.giro_preciso_pd(-45)
    # # #     self.robot.chasis.avanzar_recto(13, velocidad=1000)
    # # #     self.robot.chasis.mover_motor_izquierdo(300)
    # # #     self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.BLUE, distancia_cm=25)
    # # #     self.robot.chasis.avanzar_recto(-10)
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
    # # #     self.robot.chasis.giro_preciso(-175)
    # # #     self.robot.chasis.mover_motor_derecho(30)
    # # #     self.robot.chasis.avanzar_recto(-20)
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)

    # # # def dejar_bloques_azules_y_pala(self):
    # # #     self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)
    # # #     self.robot.navegacion.giro_preciso_pd(-35)
    # # #     self.robot.chasis.avanzar_recto(52, velocidad=1100)
    # # #     self.robot.chasis.mover_motor_izquierdo(210)
    # # #     self.robot.chasis.avanzar_recto(-10)
    # # #     self.robot.chasis.giro_preciso(-180)
    # # #     self.robot.chasis.avanzar_recto(-10)
    # # #     self.robot.chasis.mover_motor_derecho(-150)
    # # #     self.robot.chasis.avanzar_recto(-46)
    # # #     self.robot.chasis.mover_motor_izquierdo(-120)
    # # #     self.robot.chasis.avanzar_recto(-80)
    # # #     self.robot.chasis.avanzar_recto(20, velocidad=1000)
    # # #     self.robot.navegacion.giro_preciso_pd(-90)
    # # #     self.robot.mecanismos.garra_trasera.mover(-55)