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

    # Quitamos wait_after porque run_until_stalled no lo soporta
    def llevar_al_tope(self, direccion, velocidad=1000, limite_potencia=60, frenado=Stop.HOLD):
        if direccion in ("positivo", 1):
            vel_real = abs(velocidad)
        elif direccion in ("negativo", -1):
            vel_real = -abs(velocidad)
        else:
            print("Error: Dirección inválida.")
            return None
        return self.motor.run_until_stalled(vel_real, then=frenado, duty_limit=limite_potencia)

    # --- MÉTODOS VERTICALES (Heredables) ---
    def subir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover(-abs(grados), velocidad, wait_after, frenado, margen_grados)

    def bajar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def subir_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        return self.llevar_al_tope("negativo", velocidad, limite_potencia, frenado)

    def bajar_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        return self.llevar_al_tope("positivo", velocidad, limite_potencia, frenado)
    
    def soltar(self):
        """
        Detiene el motor y lo libera (Coast). 
        El motor dejará de hacer fuerza y se moverá libremente si se le aplica peso o fuerza externa.
        """
        self.motor.stop()

class GarraDelantera(Garra):
    """
    Mecanismo compuesto: 
    - Hereda mover(), subir() y bajar() para el Motor 1 (Elevador).
    - Agrega mover_pinza(), abrir() y cerrar() para el Motor 2 (Pinza).
    """
    def __init__(self, motor_elevador, motor_pinza, rango_maximo_grados=None, rango_maximo_pinza=None):
        super().__init__(motor_elevador)
        self.rango_maximo = rango_maximo_grados
        self.pinza = Garra(motor_pinza)
        self.pinza.rango_maximo = rango_maximo_pinza

    def mover_pinza(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.pinza.mover(grados, velocidad, wait_after, frenado, margen_grados)

    def abrir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_pinza(-abs(grados), velocidad, wait_after, frenado, margen_grados)

    def cerrar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_pinza(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def abrir_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        return self.pinza.llevar_al_tope("negativo", velocidad, limite_potencia, frenado)

    def cerrar_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        return self.pinza.llevar_al_tope("positivo", velocidad, limite_potencia, frenado)

    def establecer_cero(self, velocidad=800, limite_potencia=50):
        """Lleva el elevador al tope físico superior seguro y establece el origen absoluto."""
        self.subir_al_tope(velocidad=velocidad, limite_potencia=limite_potencia)
        self.motor.reset_angle(0)

    def ir_a_porcentaje(self, porcentaje, velocidad=800, wait_after=True, frenado=Stop.HOLD):
        """
        Mueve el elevador a una posición absoluta.
        """
        if self.rango_maximo is None:
            raise ValueError("Debes configurar el rango_maximo_grados al instanciar la garra delantera")
            
        porcentaje_seguro = max(0.0, min(100.0, float(porcentaje)))
        angulo_objetivo = (porcentaje_seguro / 100.0) * self.rango_maximo
        
        self.motor.run_target(
            speed=velocidad, 
            target_angle=angulo_objetivo, 
            then=frenado, 
            wait=wait_after
        )

    def establecer_cero_pinza(self, velocidad=800, limite_potencia=50):
        """Lleva la pinza al tope físico (abierta) y establece el origen absoluto."""
        self.abrir_al_tope(velocidad=velocidad, limite_potencia=limite_potencia)
        self.pinza.motor.reset_angle(0)

    def ir_a_porcentaje_pinza(self, porcentaje, velocidad=800, wait_after=True, frenado=Stop.HOLD):
        """
        Mueve la pinza a una posición absoluta.
        0% = Posición del cero establecido (típicamente cerrada)
        100% = Apertura máxima
        """
        if self.pinza.rango_maximo is None:
            raise ValueError("Debes configurar el rango_maximo_pinza al instanciar la garra delantera")
            
        porcentaje_seguro = max(0.0, min(100.0, float(porcentaje)))
        angulo_objetivo = (porcentaje_seguro / 100.0) * self.pinza.rango_maximo
        
        self.pinza.motor.run_target(
            speed=velocidad, 
            target_angle=angulo_objetivo, 
            then=frenado, 
            wait=wait_after
        )

class GarraTrasera(Garra):
    """
    Es una jaula vertical. El motor está invertido físicamente, 
    por lo que se sobrescriben los métodos para invertir su comportamiento 
    llamando a los métodos opuestos de la clase padre (Garra).
    """

    def __init__(self, motor, rango_maximo_grados=None):
        super().__init__(motor)
        self.rango_maximo = rango_maximo_grados

    def establecer_cero(self, velocidad=800, limite_potencia=50):
        """Lleva la garra al tope físico superior seguro y establece el origen absoluto."""
        self.subir_al_tope(velocidad=velocidad, limite_potencia=limite_potencia)
        self.motor.reset_angle(0)

    def ir_a_porcentaje(self, porcentaje, velocidad=800, wait_after=True, frenado=Stop.HOLD):
        """
        Mueve la garra a una posición absoluta.
        0% = Arriba (Guardada totalmente)
        100% = Abajo (Rozando el piso, equivalente a -166 grados)
        """
        if self.rango_maximo is None:
            raise ValueError("Debes configurar el rango_maximo_grados al instanciar la garra")
            
        # Limitamos por seguridad física (nunca pasará de 100 ni bajará de 0)
        porcentaje_seguro = max(0.0, min(100.0, float(porcentaje)))
        
        # Mapeo lineal
        angulo_objetivo = (porcentaje_seguro / 100.0) * self.rango_maximo
        
        # run_target va al ángulo exacto sin importar la posición actual
        self.motor.run_target(
            speed=velocidad, 
            target_angle=angulo_objetivo, 
            then=frenado, 
            wait=wait_after
        )

    # --- MÉTODOS RELATIVOS (Mantenidos por compatibilidad) ---
    def subir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        super().bajar(grados, velocidad, wait_after, frenado, margen_grados)

    def bajar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        super().subir(grados, velocidad, wait_after, frenado, margen_grados)

    def subir_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        return super().bajar_al_tope(velocidad, limite_potencia, frenado)

    def bajar_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        return super().subir_al_tope(velocidad, limite_potencia, frenado)