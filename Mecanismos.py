from pybricks.parameters import Stop
from pybricks.tools import wait
from pybricks.pupdevices import Motor

class MecanismoBase:
    """Clase base para reutilizar lógica de movimiento de motores."""
    # Anotación de tipo: le decimos que "motor" es de la clase Motor de Pybricks
    def __init__(self, motor: Motor):
        self.motor = motor

    def mover_angulo(self, grados: int, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor.angle() + grados
            self.motor.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor.angle()) > margen_grados:
                if self.motor.stalled(): 
                    break
                wait(2)
        else:
            self.motor.run_angle(velocidad, grados, then=frenado, wait=wait_after)

    def llevar_al_tope(self, direccion: str, velocidad=1000, limite_potencia=60):
        if direccion in ["positivo", 1]:
            vel_real = abs(velocidad)
        elif direccion in ["negativo", -1]:
            vel_real = -abs(velocidad)
        else:
            print("Error: La dirección debe ser 'positivo' o 'negativo'.")
            return None
        
        angulo_tope = self.motor.run_until_stalled(vel_real, then=Stop.HOLD, duty_limit=limite_potencia)
        return angulo_tope

class GarraDelantera(MecanismoBase):
    def abrir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_angulo(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def cerrar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_angulo(-abs(grados), velocidad, wait_after, frenado, margen_grados)

    def abrir_al_tope(self, velocidad=800, limite_potencia=50):
        self.motor.run_until_stalled(abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)

    def cerrar_al_tope(self, velocidad=800, limite_potencia=50):
        self.motor.run_until_stalled(-abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)

class ElevadorDelantero(MecanismoBase):
    def mover(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_angulo(grados, velocidad, wait_after, frenado, margen_grados)

class GarraTrasera(MecanismoBase):
    def mover(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_angulo(grados, velocidad, wait_after, frenado, margen_grados)

class Mecanismos:
    def __init__(self, motor_garra_delantera: Motor, motor_elevador_del: Motor, motor_garra_trasera: Motor):
        self.garra_delantera = GarraDelantera(motor_garra_delantera)
        self.elevador_delantero = ElevadorDelantero(motor_elevador_del)
        self.garra_trasera = GarraTrasera(motor_garra_trasera)