from pybricks.parameters import Stop, Color
from pybricks.pupdevices import ColorSensor
from pybricks.tools import wait
import config
from robot import Robot # Importado para el Type Hinting

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
        self.robot.chasis.avanzar_recto(-12, velocidad=1000, frenado=Stop.COAST, margen_cm=2)
        
        # Transición inmediata al giro
        self.robot.navegacion.giro_preciso_pd(90, max_speed=1000, min_speed=40, kp=8.5, kd=115.0, margen_grados=2)
        
        # Transición suave del avance al arco
        self.robot.chasis.avanzar_recto(-23, velocidad=1300, frenado=Stop.NONE, margen_cm=4)
        self.robot.chasis.mover_en_arco(-140, distancia_cm=30, stop=Stop.COAST, margen_cm=3)
        
        # CONCURRENCIA 2: Bajamos el motor mientras el robot se estabiliza para el seguidor
        self.robot.chasis.mover_motor_izquierdo(100, wait_after=False)
        
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 110, 36, tiempo_acomodo_ms=100)
        self.robot.navegacion.giro_preciso_pd(90, max_speed=1000, min_speed=40, kp=8.5, kd=115.0)

    def agarrar_bloques_blancos(self):
        self.robot.mecanismos.garra_trasera.mover(-170, velocidad= 1300, wait_after= False)
        self.robot.chasis.avanzar_recto(-5, frenado=Stop.NONE, margen_cm=1)
        self.robot.chasis.mover_en_arco(-12, distancia_cm=14, stop=Stop.NONE, margen_cm=1)
        
        # 1. El seguidor ahora frenará en seco (con el cambio a .brake() que harás en Navegacion.py)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, distancia_cm=10, lado="derecha", tiempo_acomodo_ms=0, kp=1.2, kd=3.5)

        wait(200)
        
        # 2. LIMPIEZA DE GIROSCOPIO: Borramos la "basura" de vibración del seguidor.
        # Ahora el robot sabe que su posición física actual, montado en la línea, es exactamente el grado 0.
        self.robot.hub.imu.reset_heading(0)
        
        # 3. GIRO PURO: Usamos la función que no pelea con el DriveBase para un control mecánico absoluto.
# GIRO PURO CON VOLTAJE DIRECTO
        self.robot.navegacion.giro_preciso_pd(-180, max_speed=1000, min_speed=40, kp=8.5, kd=115.0)       # 4. Transición limpia a la recolección
        self.robot.chasis.avanzar_recto(-19, velocidad=1000, frenado=Stop.COAST, margen_cm=3)
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1300, wait_after=True)



    def detectar_mosaico(self):
        # 1. Arranque agresivo
        self.robot.chasis.avanzar_recto(5, velocidad=1000, frenado=Stop.NONE)
        
        # 2. EL TRUCO DEL PIVOTE A 1000: Anclamos la derecha en seco y disparamos la izquierda
        self.robot.chasis.motor_derecha.hold() 
        self.robot.chasis.mover_motor_izquierdo(500, velocidad=1000, frenado=Stop.NONE)
        
        # 3. Avance larguísimo a tope de velocidad
        self.robot.chasis.avanzar_recto(36, velocidad=1000, frenado=Stop.NONE)
        
        # 4. EL TRUCO DEL SEGUNDO PIVOTE: Anclamos la izquierda y disparamos la derecha
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(460, velocidad=1000, frenado=Stop.NONE)
        
        # 5. Entramos al seguidor de línea a toda velocidad (aprovechando la inercia de Stop.NONE)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 20, tiempo_acomodo_ms=0, kp=0.45, kd=1.8, k_freno=0.8)
        
        self.robot.chasis.avanzar_recto(34, velocidad=1000)
        mosaico = self._identificar_combinacion(self.sensor, -5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        if (mosaico == 1 or mosaico == 2):
            self.robot.chasis.avanzar_recto(5, velocidad=1000)
            
        return mosaico
        

    def dejar_bloques_blancos(self):

        self.robot.chasis.avanzar_recto(-22)
        self.robot.navegacion.giro_preciso_pd(220)
        self.robot.chasis.avanzar_recto(-21, velocidad=1000, margen_cm=2)
        self.robot.mecanismos.garra_trasera.mover(-170, margen_grados=0)


    def agarrar_bloques_verdes(self):
        'En este codigo de aca se va a comentar las funciones que estan pendientes de pruebas para el dia lunes 18 de mayo'
        """def agarrar_bloques_verdes(self):
        # 1. Arranque explosivo aprovechando inercia
        self.robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.NONE)
        
        # 2. EL ANCLA PERFECTA: Clavamos el motor izquierdo para que el derecho pivotee a vel 1000 sin derrapar
        self.robot.chasis.motor_izquierda.hold()
        self.robot.chasis.mover_motor_derecho(250, velocidad=1000, margen_grados=30)
        
        # 3. SEGUIDOR CON CUADRATURA: Reemplazamos el seguidor normal por el de "prueba" 
        # para que se alinee perfectamente al final y podamos BORRAR el wait(300)
        self.robot.navegacion.seguidor_linea_distancia_prueba(
            self.sensor, 
            velocidad_max=100, 
            distancia_cm=60, 
            lado="derecha", 
            tiempo_acomodo_ms=0, 
            kp=1.2, 
            kd=3.5
        )
        
        # ELIMINADO: wait(300) -> Ya no es necesario porque el seguidor "prueba" lo deja clavado.
        
        # 4. GIRO VIOLENTO 180°: Usamos el hack de hardware en C (DriveBase) para romper la inercia instantáneamente
        import config # (Asegúrate de que config esté importado al inicio de Misiones.py)
        turn_rate_original = config.TURN_RATE
        self.robot.chasis.drive_base.settings(turn_rate=900, turn_acceleration=2000)
        
        self.robot.chasis.drive_base.turn(-180)
        
        # Restauramos la configuración
        self.robot.chasis.drive_base.settings(turn_rate=turn_rate_original, turn_acceleration=config.STRAIGHT_ACCEL)
        
        # 5. ATAQUE FINAL ASÍNCRONO: 
        # Aumentamos la velocidad de la garra y del avance en reversa.
        self.robot.mecanismos.garra_trasera.mover(170, velocidad=1300, frenado=Stop.HOLD, wait_after=False)
        self.robot.chasis.avanzar_recto(-20, velocidad=1000)
        """
        
        self.robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.NONE)
        self.robot.chasis.mover_motor_derecho(250, margen_grados=30)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 60, tiempo_acomodo_ms=0, kp=0.45, kd=1.8, k_freno=0.8)
        wait(300)
        self.robot.navegacion.giro_preciso_pd(-180)
        self.robot.mecanismos.garra_trasera.mover(170, frenado=Stop.HOLD, wait_after=False)
        self.robot.chasis.avanzar_recto(-20)

    def dejar_bloques_verdes(self):
        wait(300)
        self.robot.navegacion.giro_preciso_pd(-180)
        
    
    def agarrar_bloques_amarillos(self):
        self.robot.mecanismos.garra_trasera.mover(-50)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 19)
        self.robot.navegacion.giro_preciso_pd(-55)
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