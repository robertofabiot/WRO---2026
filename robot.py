"""Instanciación y cableado del hardware del robot.

Único lugar donde se crean las instancias de motores, sensores, PrimeHub
y DriveBase, inyectándolos limpiamente a Chasis, Navegacion y Mecanismos.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port
from pybricks.robotics import DriveBase

import config
from Chasis import Chasis
from Navegacion import Navegacion
from Mecanismos import MecanismoTorque, GarraDelantera, GarraPrincipal


class Robot:
    """Representa el robot completo con todos sus subsistemas integrados."""

    def __init__(
        self,
        port_izq=config.PORT_MOTOR_IZQ,
        port_der=config.PORT_MOTOR_DER,
        port_torque=config.PORT_TORQUE,
        port_garra_delantera=config.PORT_GARRA_DELANTERA,
        port_garra_principal=config.PORT_GARRA_PRINCIPAL,
        port_sensor_color=config.PORT_SENSOR_COLOR
    ):
        # 1. Hub central
        self.hub = PrimeHub()
        self.Hub = self.hub  # Alias para compatibilidad

        # 2. Motores de tracción (Direcciones compensadas)
        self.motor_derecho = Motor(port_der, config.DIRECCION_MOTOR_DER)
        self.motor_izquierdo = Motor(port_izq, config.DIRECCION_MOTOR_IZQ)
        self.motor_derecha = self.motor_derecho       # Alias compatibilidad MIO
        self.motor_izquierda = self.motor_izquierdo   # Alias compatibilidad MIO

        # 3. Motores de mecanismos
        self.motor_torque = Motor(port_torque, config.DIRECCION_TORQUE)
        self.motor_garra_delantera = Motor(port_garra_delantera)
        self.motor_garra_principal = Motor(port_garra_principal)
        self.motor_garra = self.motor_garra_principal      # Alias compatibilidad
        self.motor_garra_trasera = self.motor_torque       # Alias compatibilidad MIO
        self.motor_pinza = self.motor_garra_principal      # Alias compatibilidad MIO

        # 4. Sensor de color
        self.seguidor = ColorSensor(port_sensor_color)
        self.sensor_color = self.seguidor
        self.sensor_frente = self.seguidor                 # Alias compatibilidad MIO

        # 5. DriveBase de Pybricks
        self.drive_base = DriveBase(
            self.motor_izquierdo,
            self.motor_derecho,
            wheel_diameter=config.DIAMETRO_RUEDA,
            axle_track=config.SEPARACION_RUEDAS
        )

        # 6. Subsistemas de mecanismos
        self.torque = MecanismoTorque(self.motor_torque)
        self.garra_delantera = GarraDelantera(self.motor_garra_delantera)
        self.garra_principal = GarraPrincipal(self.motor_garra_principal)
        self.garra_trasera = self.torque                   # Alias compatibilidad MIO
        self.pinza = self.garra_principal                  # Alias compatibilidad MIO
        self.garra_delantera.pinza = self.garra_principal  # Enlace cruzado compatibilidad MIO

        # 7. Subsistemas de chasis y navegación
        self.chasis = Chasis(
            drive_base=self.drive_base,
            motor_izquierdo=self.motor_izquierdo,
            motor_derecho=self.motor_derecho,
            hub=self.hub,
            torque=self.torque
        )

        self.navegacion = Navegacion(
            chasis=self.chasis,
            sensor_color=self.seguidor,
            torque=self.torque
        )

    # =========================================================================
    # ATAJOS DE CONVENIENCIA (Delegan a Chasis, Navegacion o Mecanismos)
    # =========================================================================

    def avanzar_recto(self, *args, **kwargs):
        """Atajo directo a chasis.avanzar_recto."""
        return self.chasis.avanzar_recto(*args, **kwargs)

    def girar(self, *args, **kwargs):
        """Atajo directo a navegacion.girar."""
        return self.navegacion.girar(*args, **kwargs)

    def girar_corto(self, *args, **kwargs):
        """Atajo directo a navegacion.girar_corto."""
        return self.navegacion.girar_corto(*args, **kwargs)

    def girar_a_rumbo(self, *args, **kwargs):
        """Atajo directo a navegacion.girar_a_rumbo."""
        return self.navegacion.girar_a_rumbo(*args, **kwargs)

    def giro_de_arco(self, *args, **kwargs):
        """Atajo directo a navegacion.giro_de_arco."""
        return self.navegacion.giro_de_arco(*args, **kwargs)

    def giro_derecha(self, *args, **kwargs):
        """Atajo directo a navegacion.giro_derecha."""
        return self.navegacion.giro_derecha(*args, **kwargs)

    def giro_izquierda(self, *args, **kwargs):
        """Atajo directo a navegacion.giro_izquierda."""
        return self.navegacion.giro_izquierda(*args, **kwargs)

    def seguir_linea(self, *args, **kwargs):
        """Atajo directo a navegacion.seguir_linea."""
        return self.navegacion.seguir_linea(*args, **kwargs)

    def seguir_linea_hasta_color(self, *args, **kwargs):
        """Atajo directo a navegacion.seguir_linea_hasta_color."""
        return self.navegacion.seguir_linea_hasta_color(*args, **kwargs)

    def seguir_linea_y_mover_torque(self, *args, **kwargs):
        """Atajo directo a navegacion.seguir_linea_y_mover_torque."""
        return self.navegacion.seguir_linea_y_mover_torque(*args, **kwargs)

    def detectar_color_preciso(self, *args, **kwargs):
        """Atajo directo a navegacion.detectar_color_preciso."""
        return self.navegacion.detectar_color_preciso(*args, **kwargs)

    def avanzar_hasta_color(self, *args, **kwargs):
        """Atajo directo a navegacion.avanzar_hasta_color."""
        return self.navegacion.avanzar_hasta_color(*args, **kwargs)

    def avanzar_cruzando_lineas(self, *args, **kwargs):
        """Atajo directo a navegacion.avanzar_cruzando_lineas."""
        return self.navegacion.avanzar_cruzando_lineas(*args, **kwargs)

    def avanzar_hibrido(self, *args, **kwargs):
        """Atajo directo a navegacion.avanzar_hibrido."""
        return self.navegacion.avanzar_hibrido(*args, **kwargs)

    def avanzar_hasta_salir_negro(self, *args, **kwargs):
        """Atajo directo a navegacion.avanzar_hasta_salir_negro."""
        return self.navegacion.avanzar_hasta_salir_negro(*args, **kwargs)

    def avanzar_con_torque(self, *args, **kwargs):
        """Atajo directo a chasis.avanzar_con_torque."""
        return self.chasis.avanzar_con_torque(*args, **kwargs)

    def mover_torque(self, *args, **kwargs):
        """Atajo directo a torque.mover_grados."""
        return self.torque.mover_grados(*args, **kwargs)

    def mover_garra_delantera(self, *args, **kwargs):
        """Atajo directo a garra_delantera.mover_grados."""
        return self.garra_delantera.mover_grados(*args, **kwargs)

    def mover_garra_principal(self, *args, **kwargs):
        """Atajo directo a garra_principal.mover_grados."""
        return self.garra_principal.mover_grados(*args, **kwargs)

    def mover_garra_rapida(self, *args, **kwargs):
        """Atajo directo a garra_principal.mover_rapida."""
        return self.garra_principal.mover_rapida(*args, **kwargs)

    def frenar(self):
        """Atajo directo a chasis.frenar."""
        return self.chasis.frenar()

    def reset_motores(self):
        """Atajo directo a chasis.reset_motores."""
        return self.chasis.reset_motores()
