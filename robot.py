from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Stop, Color
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

import config
from Chasis import Chasis
from Navegacion import Navegacion

class Robot:
    def __init__(self, port_izq, port_der, port_eje_central, port_garra_delantera): 
        self.hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        
        # Motores de tracción
        self.motor_izquierda = Motor(port_izq, Direction.COUNTERCLOCKWISE)
        self.motor_derecha = Motor(port_der, Direction.CLOCKWISE)
        
        # Motores de mecanismos
        self.motor_eje_central = Motor(port_eje_central, Direction.COUNTERCLOCKWISE)
        self.motor_garra_delantera = Motor(port_garra_delantera)
        
        # Se reemplazan los números quemados (56, 160, etc.) por config
        self.drive_base = DriveBase(self.motor_izquierda, self.motor_derecha, config.DIAMETRO_RUEDA, config.SEPARACION_RUEDAS)
        self.drive_base.use_gyro(True)
        
        self.VELOCIDAD_BASE = config.VELOCIDAD_BASE
        self.drive_base.settings(straight_speed=config.STRAIGHT_SPEED, straight_acceleration=config.STRAIGHT_ACCEL, turn_rate=config.TURN_RATE)
        # Inicializamos el subsistema del Chasis
        self.chasis = Chasis(self.drive_base, self.motor_izquierda, self.motor_derecha, self.hub, self.VELOCIDAD_BASE)
        self.navegacion = Navegacion(self.chasis) 

    def identificar_combinacion(self, sensor, distancia_si_verde):
        color_principal = sensor.color()
        if color_principal not in config.MOSAICOS: # <-- Cambio aquí
            print("Error: Color principal no reconocido")
            print(f"Color escaneado: {color_principal}") 
            return -1  
        decision = config.MOSAICOS[color_principal] # <-- Cambio aquí
        if type(decision) is dict:
            self.avanzar_recto(distancia_si_verde)
            color_anterior = sensor.color()
            if color_anterior not in decision:
                print(f"Error: El segundo color ({color_anterior}) no completa un patrón verde válido.")
                return -1
            return decision[color_anterior]
        return decision
    # endregion