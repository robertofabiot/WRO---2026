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

    def avanzar_y_gatillar(self, chasis, distancia_total_cm, vel_chasis, distancia_trigger_cm, grados_garra, vel_garra=1000):
        """
        Hace un ÚNICO recorrido de 'distancia_total_cm'.
        En el milímetro exacto de 'distancia_trigger_cm', dispara la garra sin detener el chasis.
        """
        from pybricks.parameters import Stop
        from pybricks.tools import wait
        import config

        # 1. Matemáticas de precisión
        distancia_total_mm = distancia_total_cm * 10
        trigger_mm = abs(distancia_trigger_cm * 10)
        
        # Blindaje Anti-Errores: Si el trigger es más largo que el recorrido, lo ajustamos al máximo
        if trigger_mm > abs(distancia_total_mm):
            trigger_mm = abs(distancia_total_mm)
            
        # 2. Configuración de hardware segura (Topado a 930 para que no llore Pybricks)
        vel_segura = min(abs(vel_chasis), 930)
        _, accel_lin, vel_giro, accel_giro = chasis.drive_base.settings()
        chasis.drive_base.settings(vel_segura, accel_lin, vel_giro, accel_giro)
        
        dist_inicial = chasis.drive_base.distance()
        
        # 3. EL ÚNICO RECORRIDO (Modo asíncrono)
        # Aquí le decimos: "Maje, andate 42cm (o -42cm) de un solo y no me bloquées el código"
        chasis.drive_base.straight(distancia_total_mm, then=Stop.HOLD, wait=False)

        garra_disparada = False

        # 4. EL FRANCOTIRADOR (El vigía)
        while not chasis.drive_base.done():
            # Medimos cuánto ha avanzado de su recorrido total (ej. va por 10cm... 12cm... 14cm...)
            recorrido_actual = abs(chasis.drive_base.distance() - dist_inicial)

            # ¡ZAS! Llegó a los 15cm (trigger_mm). Disparamos la garra.
            if recorrido_actual >= trigger_mm and not garra_disparada:
                self.mover(
                    grados_garra, 
                    velocidad=vel_garra, 
                    wait_after=False, 
                    frenado=Stop.HOLD
                )
                garra_disparada = True

            wait(1) # Respiro de 1ms para la placa

        # 5. Seguro de vida: Si el recorrido terminó y no se disparó, lo detonamos al final
        if not garra_disparada:
            self.mover(grados_garra, velocidad=vel_garra, wait_after=False, frenado=Stop.HOLD)

        # 6. Restauramos la velocidad del chasis a su normalidad
        chasis.drive_base.settings(config.STRAIGHT_SPEED, accel_lin, vel_giro, accel_giro)
class Mecanismos:
    def __init__(self, motor_garra_delantera: Motor, motor_elevador_del: Motor, motor_garra_trasera: Motor):
        self.garra_delantera = GarraDelantera(motor_garra_delantera)
        self.elevador_delantero = ElevadorDelantero(motor_elevador_del)
        self.garra_trasera = GarraTrasera(motor_garra_trasera)