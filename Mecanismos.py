from pybricks.parameters import Stop
from pybricks.tools import wait

class GarraDelantera:
    def __init__(self, motor):
        self.motor = motor

    # Mantenemos las funciones por grados intactas
    def abrir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor.angle() + abs(grados)
            self.motor.run_angle(velocidad, abs(grados), then=frenado, wait=False)
            while abs(angulo_meta - self.motor.angle()) > margen_grados:
                if self.motor.stalled(): break
                wait(2)
        else:
            self.motor.run_angle(velocidad, abs(grados), then=frenado, wait=wait_after)

    def cerrar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor.angle() - abs(grados)
            self.motor.run_angle(velocidad, -abs(grados), then=frenado, wait=False)
            while abs(angulo_meta - self.motor.angle()) > margen_grados:
                if self.motor.stalled(): break
                wait(2)
        else:
            self.motor.run_angle(velocidad, -abs(grados), then=frenado, wait=wait_after)
    
    def abrir_al_tope(self, velocidad=800, limite_potencia=50):
        self.motor.run_until_stalled(abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)

    def cerrar_al_tope(self, velocidad=800, limite_potencia=50):
        self.motor.run_until_stalled(-abs(velocidad), then=Stop.HOLD, duty_limit=limite_potencia)


class EjeCentral:
    def __init__(self, motor):
        self.motor = motor

    # Mantenemos las funciones por grados intactas
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
        if direccion == "positivo" or direccion == 1:
            vel_real = abs(velocidad)
        elif direccion == "negativo" or direccion == -1:
            vel_real = -abs(velocidad)
        else:
            print("Error: La dirección debe ser 'positivo' o 'negativo'.")
            return None
        angulo_tope = self.motor.run_until_stalled(vel_real, then=Stop.HOLD, duty_limit=limite_potencia)
        return angulo_tope

    def mover_garra_trasera(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover(grados, velocidad, wait_after, frenado, margen_grados)