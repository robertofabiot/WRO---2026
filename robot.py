from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
import config
from Chasis import Chasis
from Navegacion import Navegacion
from Mecanismos import Mecanismos

class Robot:
    def __init__(self, port_izq: Port, port_der: Port, port_elevador_delantero: Port, port_garra_delantera: Port, port_garra_trasera: Port):
        # 1. Hardware Base
        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE)
        self.motor_derecha = Motor(port_der, Direction.CLOCKWISE)
        
        # 2. Inicialización de motores de mecanismos independientes
        self.motor_elevador_del = Motor(port_elevador_delantero, Direction.COUNTERCLOCKWISE)
        self.motor_garra_delantera = Motor(port_garra_delantera)
        self.motor_garra_trasera = Motor(port_garra_trasera)
        
        # 3. Configuración de DriveBase
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, config.DIAMETRO_RUEDA, config.SEPARACION_RUEDAS)
        self.drive_base.use_gyro(True)
        self.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED,
            straight_acceleration=config.STRAIGHT_ACCEL,
            turn_rate=config.TURN_RATE
        )
        
        # 4. Subsistemas (Patrón Facade / Inyección de dependencias)
        self.chasis = Chasis(self.drive_base, self.motor_izquierda, self.motor_derecha, self.hub, config.VELOCIDAD_BASE)
        self.navegacion = Navegacion(self.chasis)
        
        # Nuevos mecanismos separados centralizados
        self.mecanismos = Mecanismos(self.motor_garra_delantera, self.motor_elevador_del, self.motor_garra_trasera)