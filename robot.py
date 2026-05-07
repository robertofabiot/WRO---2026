from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Stop, Color
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase

import config
from Chasis import Chasis
from Navegacion import Navegacion
from Mecanismos import GarraDelantera, EjeCentral # <-- Agregamos esto (Fase 5)

class Robot:
    def __init__(self, port_izq, port_der, port_eje_central, port_garra_delantera): 
        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        
        # Motores
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE)
        self.motor_derecha = Motor(port_der, Direction.CLOCKWISE)
        self.motor_eje_central = Motor(port_eje_central, Direction.COUNTERCLOCKWISE)
        self.motor_garra_delantera = Motor(port_garra_delantera)
        
        # DriveBase
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, config.DIAMETRO_RUEDA, config.SEPARACION_RUEDAS)
        self.drive_base.use_gyro(True)
        self.drive_base.settings(
            straight_speed=config.STRAIGHT_SPEED, 
            straight_acceleration=config.STRAIGHT_ACCEL, 
            turn_rate=config.TURN_RATE
        )
        
        # SUBSISTEMAS (Patrón Facade)
        # 1. Chasis y Navegación
        self.chasis = Chasis(self.drive_base, self.motor_izquierda, self.motor_derecha, self.hub, config.VELOCIDAD_BASE)
        self.navegacion = Navegacion(self.chasis)
        
        # 2. Mecanismos (Fase 5)
        self.garra_delantera = GarraDelantera(self.motor_garra_delantera)
        self.eje_central = EjeCentral(self.motor_eje_central)