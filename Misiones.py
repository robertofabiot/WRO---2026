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

    def pruebas(self):
        self.robot.navegacion.seguidor_linea_atras(self.sensor, velocidad_max=100, distancia_cm=30)


    def cemento_y_llana(self):

        self.robot.mecanismos.garra_delantera.llevar_al_tope("positivo", velocidad=1000, limite_potencia=60)

       

        # 2. ENCADENAMIENTO: Aumentamos el margen para no frenar a 0 antes del seguidor de línea.

        self.robot.chasis.mover_en_arco(radio_cm=14, distancia_cm=17, stop=Stop.NONE, margen_cm=3)

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 110, 84, tiempo_acomodo_ms=0, margen_cm=10)

        self.robot.navegacion.giro_preciso_pd(-94, margen_grados=5)

       

        # 3. DEPURACIÓN DE LATENCIAS: Eliminamos todos los wait(1) entre llamadas

        self.robot.mecanismos.garra_trasera.mover(167, velocidad=180, wait_after=False)

        self.robot.chasis.avanzar_recto(-11, velocidad=1000, frenado=Stop.COAST, margen_cm=2)

       

        # Transición inmediata al giro

        self.robot.navegacion.giro_preciso_pd(90, max_speed=1000, min_speed=40, kp=8.5, kd=115.0, margen_grados=2)

       

        # Transición suave del avance al arco

        self.robot.chasis.avanzar_recto(-23, velocidad=1300, frenado=Stop.NONE, margen_cm=4)

        self.robot.chasis.mover_en_arco(-142, distancia_cm=30, stop=Stop.COAST, margen_cm=3)

       

        # # CONCURRENCIA 2: Bajamos el motor mientras el robot se estabiliza para el seguidor

        self.robot.chasis.mover_motor_izquierdo(130, wait_after=False)

       

        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 110, 38, tiempo_acomodo_ms=100)

        self.robot.navegacion.giro_preciso_pd(90, max_speed=1000, min_speed=40, kp=8.5, kd=115.0)

    def agarrar_bloques_blancos(self):
        # 1. Movimiento asíncrono y encadenado
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-5, frenado=Stop.NONE, margen_cm=1)
        self.robot.chasis.mover_en_arco(-12, distancia_cm=14, stop=Stop.NONE, margen_cm=1)
        
        # 2. CUADRATURA PERFECTA: Usamos la versión "_prueba" que tiene la Fase 2 de acomodo final.
        # Esto reemplaza al wait(200) y asegura que el robot quede 100% paralelo a la línea.
        self.robot.navegacion.seguidor_linea_distancia_prueba(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=10, 
            lado="derecha", 
            tiempo_acomodo_ms=0, 
            kp=1.2, 
            kd=3.5
        )
        
        # 3. Limpiamos el giroscopio ahora que el robot está perfectamente recto
        self.robot.hub.imu.reset_heading(0)
        
        # 4. EL GIRO VIOLENTO: Al límite del hardware.
        # Calculado: (1000 max_motor * 56) / 160 = 350 max turn_rate.
        self.robot.chasis.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED, 
            straight_acceleration=config.STRAIGHT_ACCEL, 
            turn_rate=350,          # Límite físico absoluto para que no de ValueError
            turn_acceleration=1500  # Latigazo instantáneo para romper la inercia
        )
        
        # Usamos el .turn() nativo que no tiene lag y frena por hardware
        self.robot.navegacion.giro_preciso_pd(-175)
        
        # Restauramos la configuración
        self.robot.chasis.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED, 
            straight_acceleration=config.STRAIGHT_ACCEL, 
            turn_rate=config.TURN_RATE, 
            turn_acceleration=config.STRAIGHT_ACCEL
        )
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=350, wait_after=False)

        self.robot.chasis.avanzar_recto(-21.5, velocidad=1000, frenado=Stop.BRAKE)
       

    def dejar_bloques_blancos(self):
         # 1. PIVOTES CONTROLADOS
        self.robot.chasis.motor_derecha.hold() 
        self.robot.chasis.mover_motor_izquierdo(400, velocidad=1000, frenado=Stop.HOLD)
        
        # 2. EL SPRINT HACKEADO (BLINDADO CONTRA ERRORES)
        # Pasamos los 4 argumentos obligatorios para que Pybricks no tire ValueError
        self.robot.chasis.drive_base.settings(
            straight_speed=950,           # Velocidad tope segura
            straight_acceleration=1500,   # Aceleración agresiva (bajamos de 2000 a 1500 por seguridad)
            turn_rate=config.TURN_RATE,   # Mantenemos el giro normal
            turn_acceleration=config.STRAIGHT_ACCEL 
        )
        
        # El robot saltará hacia adelante al instante
        self.robot.chasis.avanzar_recto(42, velocidad=950, frenado=Stop.NONE)
        
        # 3. RESTAURAMOS LA ACELERACIÓN A LA NORMALIDAD
        self.robot.chasis.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED, 
            straight_acceleration=config.STRAIGHT_ACCEL,
            turn_rate=config.TURN_RATE,
            turn_acceleration=config.STRAIGHT_ACCEL
        )
        
        # 4. SEGUNDO PIVOTE CONTROLADO
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(400, velocidad=1000, frenado=Stop.HOLD)
        
        # 5. EL SEGUIDOR BLINDADO PARA EL ESCANEO
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, distancia_cm=40)
        
        # # Acercamiento final y escaneo
        self.robot.chasis.mover_motor_derecho(70)        

        self.robot.navegacion.giro_preciso_pd(227)       


        # self.robot.chasis.avanzar_recto(-9)
        # self.robot.navegacion.giro_preciso_pd(55)
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad=1500, wait_after=False, margen_grados=3)
        self.robot.chasis.avanzar_recto(-21, velocidad=1000, margen_cm=2)
       


    def agarrar_bloques_verdes(self):
        
        # 1. Arranque explosivo aprovechando inercia
        self.robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.BRAKE)

        # 2. EL ANCLA PERFECTA: Clavamos el motor izquierdo para que el derecho pivotee a vel 1000 sin derrapar
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(300, velocidad=1000, margen_grados=30)
        
        # 3. SEGUIDOR CON CUADRATURA: Reemplazamos el seguidor normal por el de "prueba" 
        # para que se alinee perfectamente al final y podamos BORRAR el wait(300)
        self.robot.navegacion.seguidor_linea_distancia_prueba(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=45, 
            lado="derecha", 
            tiempo_acomodo_ms=0, 
            kp=1.2, 
            kd=3.5
        )
        
        self.robot.navegacion.giro_preciso_pd(-180)
        
        
        self.robot.mecanismos.garra_trasera.mover(167, velocidad=180, frenado=Stop.HOLD, wait_after=False)
        self.robot.chasis.avanzar_recto(-23, velocidad=1000)
  
  

    def dejar_bloques_verdes_y_detectar_mosaico(self):
        self.robot.chasis.avanzar_recto(5, velocidad=1000)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, tiempo_acomodo_ms=200, distancia_cm=50)

        self.robot.chasis.mover_motor_derecho(80)
        self.robot.chasis.avanzar_recto(15)

        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        if (mosaico == 1 or mosaico == 2):
            self.robot.chasis.avanzar_recto(-5, velocidad=1000)
            


        self.robot.chasis.avanzar_recto(-20, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(180)
        self.robot.chasis.avanzar_recto(-5, velocidad=1000)
        return mosaico 




    def agarrar_bloques_amarillos_y_azules(self):
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.GREEN, lado="izquierda", distancia_cm=70)
        self.robot.chasis.mover_motor_izquierdo(700, frenado=Stop.BRAKE)
        self.robot.mecanismos.garra_trasera.mover(170,velocidad=130, wait_after=False)
        self.robot.chasis.avanzar_recto(-35)
        self.robot.chasis.avanzar_recto(50)
        self.robot.chasis.mover_motor_izquierdo(700)
        
    def agarrar_bloques_amarillos(self):
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 25)
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(25)
        self.robot.navegacion.giro_preciso_pd(45)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.YELLOW, distancia_cm=26)
        self.robot.chasis.avanzar_recto(-10, velocidad=1000)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
        self.robot.chasis.giro_preciso(-175)
        self.robot.chasis.mover_motor_derecho(30)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)

    def dejar_bloques_amarillos(self):
        self.robot.chasis.mover_motor_derecho(225, velocidad=1000, margen_grados=30)
        self.robot.chasis.avanzar_recto(60, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_motor_izquierdo(225, velocidad=1000, margen_grados=30)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 58, lado="izquierda", tiempo_acomodo_ms=800)
        wait(500)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.chasis.avanzar_recto(-17, velocidad=1000)
        self.robot.mecanismos.garra_trasera.mover(-55)

    def recoger_bloques_azules(self):
        self.robot.chasis.avanzar_recto(15, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 50)
        self.robot.navegacion.giro_preciso_pd(-45)
        self.robot.chasis.avanzar_recto(13, velocidad=1000)
        self.robot.chasis.mover_motor_izquierdo(300)
        self.robot.navegacion.seguidor_linea_color(self.sensor, 100, Color.BLUE, distancia_cm=25)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
        self.robot.chasis.giro_preciso(-175)
        self.robot.chasis.mover_motor_derecho(30)
        self.robot.chasis.avanzar_recto(-20)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)

    def dejar_bloques_azules_y_pala(self):
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)
        self.robot.navegacion.giro_preciso_pd(-35)
        self.robot.chasis.avanzar_recto(52, velocidad=1100)
        self.robot.chasis.mover_motor_izquierdo(210)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.chasis.giro_preciso(-180)
        self.robot.chasis.avanzar_recto(-10)
        self.robot.chasis.mover_motor_derecho(-150)
        self.robot.chasis.avanzar_recto(-46)
        self.robot.chasis.mover_motor_izquierdo(-120)
        self.robot.chasis.avanzar_recto(-80)
        self.robot.chasis.avanzar_recto(20, velocidad=1000)
        self.robot.navegacion.giro_preciso_pd(-90)
        self.robot.mecanismos.garra_trasera.mover(-55)