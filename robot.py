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

    # region MECANISMOS - Garra Delantera
    def abrir_garra_delantera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_garra_delantera.angle() + abs(grados)
            self.motor_garra_delantera.run_angle(velocidad, abs(grados), then=frenado, wait=False)
            while abs(angulo_meta - self.motor_garra_delantera.angle()) > margen_grados:
                if self.motor_garra_delantera.stalled(): break
                wait(2)
        else:
            self.motor_garra_delantera.run_angle(velocidad, abs(grados), then=frenado, wait=wait_after)
    
    def abrir_garra_delantera_al_tope(self, velocidad=800, limite_potencia=50):
        self.motor_garra_delantera.run_until_stalled(abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)

    def cerrar_garra_delantera_al_tope(self, velocidad=800, limite_potencia=50):
        self.motor_garra_delantera.run_until_stalled(-abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)
    # endregion

    # region MECANISMOS - Eje Central y Garra Trasera
    def mover_eje_central(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_eje_central.angle() + grados
            self.motor_eje_central.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor_eje_central.angle()) > margen_grados:
                if self.motor_eje_central.stalled(): break
                wait(2)
        else:
            self.motor_eje_central.run_angle(velocidad, grados, then=frenado, wait=wait_after)
    
    def llevar_eje_central_al_tope(self, direccion, velocidad=1000, limite_potencia=60):
        if direccion == "positivo" or direccion == 1:
            vel_real = abs(velocidad)
        elif direccion == "negativo" or direccion == -1:
            vel_real = -abs(velocidad)
        else:
            print("Error: La dirección debe ser 'positivo' o 'negativo'.")
            return None
        angulo_tope = self.motor_eje_central.run_until_stalled(vel_real, then=Stop.HOLD, duty_limit=limite_potencia)
        return angulo_tope

    def mover_garra_trasera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_eje_central(grados, velocidad, wait_after, frenado, margen_grados)
    # endregion

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