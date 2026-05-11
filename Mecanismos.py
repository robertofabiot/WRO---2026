from pybricks.parameters import Stop
from pybricks.tools import wait

class Garra:
    """Clase base para todos los mecanismos de agarre."""
    def __init__(self, motor):
        self.motor = motor

    def mover(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor.angle() + grados
            self.motor.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor.angle()) > margen_grados:
                if self.motor.stalled(): break
                wait(2)
        else:
            self.motor.run_angle(velocidad, grados, then=frenado, wait=wait_after)

    def llevar_al_tope(self, direccion, velocidad=1000, limite_potencia=60):
        if direccion in ("positivo", 1):
            vel_real = abs(velocidad)
        elif direccion in ("negativo", -1):
            vel_real = -abs(velocidad)
        else:
            print("Error: Dirección inválida.")
            return None
        return self.motor.run_until_stalled(vel_real, then=Stop.HOLD, duty_limit=limite_potencia)

    def abrir_al_tope(self, velocidad=800, limite_potencia=50):
        return self.llevar_al_tope("positivo", velocidad, limite_potencia)

    def cerrar_al_tope(self, velocidad=800, limite_potencia=50):
        return self.llevar_al_tope("negativo", velocidad, limite_potencia)


class GarraDelantera(Garra):
    """Añade métodos semánticos específicos para abrir y cerrar por grados."""
    def abrir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def cerrar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover(-abs(grados), velocidad, wait_after, frenado, margen_grados)


class GarraTrasera(Garra):
    """Hereda directamente de Garra. Usa mover(), abrir_al_tope() y cerrar_al_tope()"""
    pass