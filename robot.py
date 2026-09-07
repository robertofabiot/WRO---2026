from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
import config
from Chasis import Chasis
from Navegacion import Navegacion
from Mecanismos import GarraDelantera, GarraTrasera

class Robot:
    def __init__(self, port_izq, port_der, port_garra_trasera, port_garra_delantera, port_pinza): 
        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE)
        self.motor_derecha = Motor(port_der, Direction.CLOCKWISE)
        
        # Motores de mecanismos
        self.motor_garra_trasera = Motor(port_garra_trasera, Direction.COUNTERCLOCKWISE)
        self.motor_garra_delantera = Motor(port_garra_delantera) 
        self.motor_pinza = Motor(port_pinza)
        
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, config.DIAMETRO_RUEDA, config.SEPARACION_RUEDAS)
        self.drive_base.use_gyro(True)
        self.drive_base.settings(
            straight_speed=config.VELOCIDAD_RECTA, 
            straight_acceleration=config.ACELERACION_RECTA, 
            turn_rate=config.TASA_GIRO
        )
        
        self.chasis = Chasis(self.drive_base, self.motor_izquierda, self.motor_derecha, self.hub, config.VELOCIDAD_BASE)
        self.navegacion = Navegacion(self.chasis)
        self.chasis.navegacion = self.navegacion
        
        self.garra_trasera = GarraTrasera(self.motor_garra_trasera, rango_maximo_grados=config.RANGO_MAXIMO_GARRA_TRASERA)
        self.garra_delantera = GarraDelantera(
            self.motor_garra_delantera, 
            self.motor_pinza, 
            rango_maximo_grados=config.RANGO_MAXIMO_GARRA_DELANTERA,
            rango_maximo_pinza=config.RANGO_MAXIMO_PINZA
        )