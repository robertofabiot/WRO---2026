from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

class Robot:
    def __init__(self, port_izq, port_der, port_garra):
        """
        Configuración inicial del robot.
        Define la geometría, los puertos y activa la estabilización por giroscopio.
        """
        # 1. Inicializar el Hub (Orientación: Arriba=Z, Frente=X)
        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        
        # 2. Configurar Motores
        # Ajustamos las direcciones según tu construcción física
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE)
        self.motor_derecha = Motor(port_der, Direction.CLOCKWISE)
        self.motor_garra = Motor(port_garra, Direction.CLOCKWISE)
        
        # 3. Configurar Base Motriz (DriveBase)
        # Rueda: 56mm, Eje: 160mm
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, 56, 160)
        
        # 4. Activar Estabilización Gyro (Crucial para ir recto)
        self.drive_base.use_gyro(True)
        
        # 5. Configurar velocidades por defecto (basado en 'base_settings')
        self.VELOCIDAD_BASE = 950
        self.drive_base.settings(straight_speed=500, straight_acceleration=600, turn_rate=200)

# region Movimiento Básico (Chasis)

    def avanzar_recto(self, distancia_cm, velocidad=None, frenado=Stop.BRAKE):
        """
        Avanza en línea recta usando el giroscopio para corregir el rumbo.
        :param distancia_cm: Distancia en centímetros (negativo para retroceder).
        :param velocidad: (Opcional) Velocidad en deg/s. Si no se da, usa la por defecto.
        """
        if velocidad is None:
            velocidad = self.VELOCIDAD_BASE
            
        # Limitador de seguridad para no quemar motores
        velocidad = max(min(velocidad, 977), -977)
        
        # Conversión cm -> mm requerida por Pybricks
        distancia_mm = distancia_cm * 10
        
        self.drive_base.straight(distancia_mm, then=frenado)
    
    def avanzar_hasta_color(self, sensor_color, color_objetivo, velocidad=300):
        """
        Avanza o retrocede en línea recta perfecta (asistida por giroscopio) 
        hasta que el sensor detecte el color especificado.
        
        :param sensor_color: El objeto ColorSensor instanciado en app.py.
        :param color_objetivo: El color a buscar (ej. Color.RED, Color.BLUE).
        :param velocidad: Velocidad en mm/s. Usa un valor negativo para retroceder.
        """
        # Iniciamos el movimiento recto asistido por giroscopio
        self.drive_base.drive(velocidad, 0)
        
        while True:
            # Leemos el color actual bajo el sensor
            color_actual = sensor_color.color()
            
            # Condición de salida: Si el color leído coincide con el objetivo
            if color_actual == color_objetivo:
                break
                
            # Micro-pausa para no saturar las lecturas del I2C del sensor
            self.esperar(1)
            
        # Frenamos en seco al encontrar el color
        self.drive_base.stop()

    def girar_sobre_eje(self, grados):
        """
        Gira el robot sobre su propio eje (giro tipo tanque).
        :param grados: Grados a girar (positivo = derecha, negativo = izquierda).
        """
        self.drive_base.turn(grados)
    
    def giro_preciso(self, angulo_objetivo):
        """
        Gira el robot a un ángulo exacto usando el giroscopio y un Control Proporcional.
        """
        # 1. Reiniciamos el giroscopio a 0 grados antes de empezar a girar
        self.hub.imu.reset_heading(0) # Nota: Usa self.prime_hub si no cambiaste el nombre en el __init__
        
        # --- VARIABLES DE AJUSTE ---
        kp = 2.5           # Constante proporcional (agresividad del giro)
        min_speed = 50     # Velocidad mínima para que no se atasque al final
        tolerancia = 1     # Margen de error aceptable (en grados)
        
        while True:
            # 2. Leemos hacia dónde está apuntando el robot actualmente
            angulo_actual = self.hub.imu.heading()
            
            # 3. Calculamos cuánto nos falta para llegar (el error)
            error = angulo_objetivo - angulo_actual
            
            # 4. Condición de salida: Si estamos lo suficientemente cerca, rompemos el bucle
            if abs(error) <= tolerancia:
                break
                
            # 5. Calculamos la velocidad de giro (mientras menor el error, más lento gira)
            turn_rate = error * kp
            
            # 6. Aseguramos que el robot no vaya tan lento que la fricción lo detenga antes de llegar
            if turn_rate > 0:
                turn_rate = max(turn_rate, min_speed)
            else:
                turn_rate = min(turn_rate, -min_speed)
                
            # 7. Movemos la base: 0 velocidad de avance, 'turn_rate' de giro
            self.drive_base.drive(0, turn_rate)
            
            # Pequeña pausa para estabilizar las lecturas del procesador
            wait(10)
            
        # 8. Frenamos los motores en seco al llegar al objetivo
        self.drive_base.stop()

    def mover_en_arco(self, radio_cm, angulo=None, distancia_cm=None):
        """
        Realiza una curva suave.
        :param radio_cm: Radio de la curva en cm.
        :param angulo: Cuánto girar en grados.
        :param distancia_cm: Cuánto recorrer sobre la curva en cm.
        """
        radio_mm = radio_cm * 10
        distancia_mm = distancia_cm * 10 if distancia_cm is not None else None
        
        self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm)

# endregion

# region Control de Motores Individuales

    def mover_motor_izquierdo(self, grados, velocidad=500):
        """Mueve solo la rueda izquierda."""
        self.motor_izquierda.run_angle(velocidad, grados)

    def mover_motor_derecho(self, grados, velocidad=500):
        """Mueve solo la rueda derecha."""
        self.motor_derecha.run_angle(velocidad, grados)

    def mover_garra(self, grados, velocidad=500):
        """
        Mueve el motor de la garra/accesorio.
        :param grados: Grados relativos a mover.
        :param velocidad: Velocidad del movimiento.
        """
        self.motor_garra.run_angle(velocidad, grados)

# endregion

    def seguir_linea(self, sensor_color, distancia_cm=None, velocidad=150, kp=3.6, kd=1.0):
        """
        Sigue la línea usando control PD inyectando voltaje directo (Duty Cycle) 
        a los motores para una respuesta instantánea y sin latencia.
        :param sensor_color: El objeto ColorSensor instanciado.
        :param distancia_cm: Distancia límite a recorrer en cm (None para infinito).
        :param velocidad: Velocidad base (Duty Cycle, típicamente de 0 a 10000, 
                          pero ajustado a la escala de Pybricks).
        :param kp: Agresividad de la corrección.
        :param kd: Suavizado de la oscilación.
        """
        last_error = 0 
        
        # Si se definió una distancia, reseteamos la odometría de la base
        if distancia_cm is not None:
            self.drive_base.reset()
            distancia_mm = distancia_cm * 10
            
        while True: 
            # Condición de salida por odometría
            if distancia_cm is not None and abs(self.drive_base.distance()) >= distancia_mm:
                break
                
            current_reflection = sensor_color.reflection()
            error = current_reflection - 35
            derivative = error - last_error
            correction = (error * kp) + (derivative * kd)

            # Lógica pura del coach usando dc()
            self.motor_izquierda.dc(velocidad + correction)
            self.motor_derecha.dc(velocidad - correction)
            
            last_error = error
            
            # Micro-pausa para estabilizar la lectura del sensor
            self.esperar(10)
            
        # Frenar en seco al cumplir la distancia
        self.motor_izquierda.brake()
        self.motor_derecha.brake()

# region Utilidades y Sensores

    def esperar(self, milisegundos):
        """Pausa el programa."""
        wait(milisegundos)

    def resetear_giroscopio(self):
        """Reinicia el ángulo del giroscopio a 0."""
        self.hub.imu.reset_heading(0)
        
    def obtener_angulo(self):
        """Devuelve el ángulo actual del robot."""
        return self.hub.imu.heading()

# endregion