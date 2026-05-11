from pybricks.parameters import Stop
from pybricks.tools import wait

class Garra:
    """Clase base para mecanismos de tipo pinza o garra."""
    def __init__(self, motor):
        self.motor = motor

    def mover(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Mueve el motor una cantidad específica de grados."""
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor.angle() + grados
            self.motor.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor.angle()) > margen_grados:
                if self.motor.stalled(): break
                wait(2)
        else:
            self.motor.run_angle(velocidad, grados, then=frenado, wait=wait_after)

    def llevar_al_tope(self, direccion, velocidad=1000, limite_potencia=60):
        """Mueve el motor hasta que se bloquee mecánicamente en la dirección indicada."""
        if direccion == "positivo" or direccion == 1:
            vel_real = abs(velocidad)
        elif direccion == "negativo" or direccion == -1:
            vel_real = -abs(velocidad)
        else:
            print("Error: La dirección debe ser 'positivo' o 'negativo'.")
            return None
        return self.motor.run_until_stalled(vel_real, then=Stop.HOLD, duty_limit=limite_potencia)

    def abrir_al_tope(self, velocidad=800, limite_potencia=50):
        """Abre la garra hasta el bloqueo mecánico."""
        return self.llevar_al_tope("positivo", velocidad, limite_potencia)

    def cerrar_al_tope(self, velocidad=800, limite_potencia=50):
        """Cierra la garra hasta el bloqueo mecánico."""
        return self.llevar_al_tope("negativo", velocidad, limite_potencia)


class GarraDelantera(Garra):
    """Garra frontal con métodos específicos de apertura y cierre."""
    def abrir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Abre la garra por grados positivos."""
        self.mover(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def cerrar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Cierra la garra por grados negativos."""
        self.mover(-abs(grados), velocidad, wait_after, frenado, margen_grados)


class GarraTrasera(Garra):
    """Garra trasera que hereda todas las funciones de movimiento y tope."""
    def mover_garra_trasera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Mantiene compatibilidad con llamadas existentes de 'mover_garra_trasera'."""
        self.mover(grados, velocidad, wait_after, frenado, margen_grados)