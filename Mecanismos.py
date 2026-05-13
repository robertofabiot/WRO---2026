from pybricks.parameters import Stop
from pybricks.tools import wait

class Garra:
    """Clase base. Mueve un motor genérico. Orientación semántica: Vertical."""
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

    # --- MÉTODOS VERTICALES (Heredables) ---
    def subir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover(-abs(grados), velocidad, wait_after, frenado, margen_grados)

    def bajar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def subir_al_tope(self, velocidad=800, limite_potencia=50):
        return self.llevar_al_tope("negativo", velocidad, limite_potencia)

    def bajar_al_tope(self, velocidad=800, limite_potencia=50):
        return self.llevar_al_tope("positivo", velocidad, limite_potencia)


class GarraDelantera(Garra):
    """
    Mecanismo compuesto: 
    - Hereda mover(), subir() y bajar() para el Motor 1 (Elevador).
    - Agrega mover_pinza(), abrir() y cerrar() para el Motor 2 (Pinza).
    """
    def __init__(self, motor_elevador, motor_pinza):
        # 1. Le mandamos el motor elevador a la clase padre (Motor 1)
        super().__init__(motor_elevador) 
        
        # 2. Guardamos el motor de la pinza (Motor 2) creando una "sub-garra" interna
        self.pinza = Garra(motor_pinza)

    def mover_pinza(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.pinza.mover(grados, velocidad, wait_after, frenado, margen_grados)

    def abrir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_pinza(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def cerrar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_pinza(-abs(grados), velocidad, wait_after, frenado, margen_grados)

    def abrir_al_tope(self, velocidad=800, limite_potencia=50):
        return self.pinza.llevar_al_tope("positivo", velocidad, limite_potencia)

    def cerrar_al_tope(self, velocidad=800, limite_potencia=50):
        return self.pinza.llevar_al_tope("negativo", velocidad, limite_potencia)


class GarraTrasera(Garra):
    """
    Es una jaula vertical. El motor está invertido físicamente, 
    por lo que se sobrescriben los métodos para invertir su comportamiento 
    llamando a los métodos opuestos de la clase padre (Garra).
    """

    def subir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        # Subir en la garra trasera equivale a bajar en la lógica base
        super().bajar(grados, velocidad, wait_after, frenado, margen_grados)

    def bajar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        # Bajar en la garra trasera equivale a subir en la lógica base
        super().subir(grados, velocidad, wait_after, frenado, margen_grados)

    def subir_al_tope(self, velocidad=800, limite_potencia=50):
        # Subir al tope llama a bajar_al_tope del padre
        return super().bajar_al_tope(velocidad, limite_potencia)

    def bajar_al_tope(self, velocidad=800, limite_potencia=50):
        # Bajar al tope llama a subir_al_tope del padre
        return super().subir_al_tope(velocidad, limite_potencia)