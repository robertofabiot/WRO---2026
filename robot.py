from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port, Stop
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

class Robot:
    def __init__(self, port_izq, port_der, port_garra):
        """
        Inicializa el robot, sus motores y la base motriz.
        Se pasan los puertos en lugar de los motores ya instanciados para mantener
        la creación de objetos encapsulada dentro de la clase.
        """
        self.prime_hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        
        # Motores de tracción
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE) # Ajusta la dirección según tu ensamble
        self.motor_derecha = Motor(port_der)
        
        # Motor de manipulación (Garra)
        self.motor_garra = Motor(port_garra)
        
        # Base motriz: Ajusta el diámetro de la rueda (ej. 56) y la separación (ej. 160)
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, 56, 160)
        self.drive_base.use_gyro(True) # Activa el uso interno del giroscopio si usas Pybricks 3.x

# region Auxiliares
    @staticmethod
    def cm_a_mm(cm):
        return cm * 10

    def espera(self, milisegundos):
        """Pausa la ejecución del programa."""
        wait(milisegundos)
# endregion

# region Movimiento de Tracción (DriveBase)
    def avanzar_cm(self, cm):
        """Avance básico usando la odometría de la DriveBase."""
        mm = self.cm_a_mm(cm)
        self.drive_base.straight(mm)

    def retroceder_cm(self, cm):
        """Retroceso básico."""
        self.avanzar_cm(-cm)

    def avance_giroscopico(self, cm, velocidad=300):
        """
        Avance hiperpreciso en línea recta utilizando un control Proporcional
        con el giroscopio para evitar desviaciones por fricción o peso asimétrico.
        Reemplaza la función 'GyroDrive' de tu amigo.
        """
        mm_objetivo = self.cm_a_mm(cm)
        self.drive_base.reset()
        angulo_inicial = self.prime_hub.imu.heading()
        kp = 3.0 # Constante de corrección (ajustable)

        # Determinar dirección
        direccion = 1 if cm > 0 else -1

        while abs(self.drive_base.distance()) < abs(mm_objetivo):
            # Calculamos el error de desviación
            error = angulo_inicial - self.prime_hub.imu.heading()
            
            # Calculamos la corrección (turn_rate)
            correccion = error * kp
            
            # Aplicamos velocidad y corrección
            self.drive_base.drive(velocidad * direccion, correccion)
            wait(10)

        # Frenado en seco
        self.drive_base.stop()

    def curva(self, cm, angulo):
        """
        Reemplaza los 'DRIVE_BASE.arc' del código original.
        Realiza una curva precisa usando la cinemática de la base.
        :param radio_mm: Radio de la curva en milímetros. Un radio de 0 hace que gire sobre su propio eje.
        :param angulo: Grados que abarcará la curva (positivo = derecha, negativo = izquierda).
        """
        mm_objetivo = self.cm_a_mm(cm)
        self.drive_base.curve(mm_objetivo, angulo)

# endregion

# region Giros
    def giro_preciso(self, angulo_objetivo):
        """
        Gira el robot a un ángulo exacto usando el giroscopio y un Control Proporcional.
        Excelente para giros de 90° o reorientaciones en pista.
        """
        self.prime_hub.imu.reset_heading(0)
        kp = 2.5
        min_speed = 50
        tolerancia = 1

        while True:
            angulo_actual = self.prime_hub.imu.heading()
            error = angulo_objetivo - angulo_actual

            if abs(error) <= tolerancia:
                break

            turn_rate = error * kp

            if turn_rate > 0:
                turn_rate = max(turn_rate, min_speed)
            else:
                turn_rate = min(turn_rate, -min_speed)

            self.drive_base.drive(0, turn_rate)
            wait(10)

        self.drive_base.stop()
# endregion

# region Control de Motores Individuales
    def mover_motor_izquierdo(self, velocidad, grados):
        """Mueve solo la llanta izquierda (ideal para acomodarse contra paredes)."""
        self.motor_izquierda.run_angle(velocidad, grados, then=Stop.HOLD, wait=True)

    def mover_motor_derecho(self, velocidad, grados):
        """Mueve solo la llanta derecha."""
        self.motor_derecha.run_angle(velocidad, grados, then=Stop.HOLD, wait=True)
# endregion

# region Manipuladores (Garra)
    def mover_garra(self, velocidad, grados):
        """
        Controla el motor de la garra trasera. 
        Reemplaza 'bajar_garraT' y 'subir_garraT'. El signo de los grados define si sube o baja.
        """
        self.motor_garra.run_angle(velocidad, grados, then=Stop.HOLD, wait=True)
# endregion

# region Seguidores
    def seguidor_linea(self, sensor_color, velocidad=150):
        """
        Seguidor de línea usando control PD (Proporcional-Derivativo) 
        para una navegación fluida sin oscilaciones bruscas.
        """
        kp = 3.6
        kd = 1.0
        last_error = 0 
        
        while True: 
            # Valor objetivo (setpoint) suele ser el promedio entre blanco y negro (ej. 35)
            current_reflection = sensor_color.reflection()
            error = current_reflection - 35 
            derivative = error - last_error
            
            correction = (error * kp) + (derivative * kd)
            
            # Se aplica la corrección a los motores
            self.motor_izquierda.dc(velocidad - correction)
            self.motor_derecha.dc(velocidad + correction)
            
            last_error = error
            wait(10) # Pequeña pausa para no saturar el procesador
# endregion