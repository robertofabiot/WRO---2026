from pybricks.parameters import Stop
from pybricks.tools import wait
from pybricks.pupdevices import Motor
import config # Movido arriba para eliminar lag de I/O en ejecución

class MecanismoBase:
    """Clase base para reutilizar lógica de movimiento de motores."""
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

    def llevar_al_tope(self, direccion: str, velocidad=1000, limite_potencia=10, wait_after=True):
        """
        Lleva el motor al tope físico. 
        Soporta modo síncrono (preciso) y asíncrono (hack de voltaje).
        """
        if direccion in ["positivo", 1]:
            vel_real = abs(velocidad)
            pot_real = abs(limite_potencia)
        elif direccion in ["negativo", -1]:
            vel_real = -abs(velocidad)
            pot_real = -abs(limite_potencia)
        else:
            print("Error: La dirección debe ser 'positivo' o 'negativo'.")
            return None
        
        # --- LÓGICA DE BIFURCACIÓN ASÍNCRONA ---
        if wait_after:
            # Opción 1 (Predeterminada): El código se congela hasta que la garra muerde el objetivo.
            return self.motor.run_until_stalled(vel_real, then=Stop.HOLD, duty_limit=limite_potencia)
        else:
            # Opción 2 (Modo Asíncrono): Engañamos al firmware inyectando voltaje puro continuo.
            # El motor empuja con el límite de potencia indicado mientras el código avanza instantáneamente.
            self.motor.dc(pot_real)
            return None

class GarraDelantera(MecanismoBase):
    
    # --- AQUÍ ESTÁ LA MAGIA ---
    def __init__(self, motor: Motor):
        # 1. Llamamos a la clase padre para que guarde el motor
        super().__init__(motor)
        
        # 2. Le inyectamos los límites destrabados ÚNICAMENTE a esta garra
        self.motor.control.limits(speed=1500, acceleration=5000)
    # --------------------------

    def abrir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_angulo(-abs(grados), velocidad, wait_after, frenado, margen_grados)

    def cerrar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        self.mover_angulo(abs(grados), velocidad, wait_after, frenado, margen_grados)

    # REFACTORIZACIÓN DRY: Ahora las garras reciclan la función llevar_al_tope de la base
    def abrir_al_tope(self, velocidad=800, limite_potencia=50, wait_after=True):
        self.llevar_al_tope("negativo", velocidad, limite_potencia, wait_after)

    def cerrar_al_tope(self, velocidad=800, limite_potencia=50, wait_after=True):
        self.llevar_al_tope("positivo", velocidad, limite_potencia, wait_after)

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
        # (Los imports fueron movidos a la línea 1 para evitar lag en el procesador)
        
        # 1. Matemáticas de precisión
        distancia_total_mm = distancia_total_cm * 10
        trigger_mm = abs(distancia_trigger_cm * 10)
        
        # Blindaje Anti-Errores
        if trigger_mm > abs(distancia_total_mm):
            trigger_mm = abs(distancia_total_mm)
            
        # 2. Configuración de hardware segura
        vel_segura = min(abs(vel_chasis), 930)
        _, accel_lin, vel_giro, accel_giro = chasis.drive_base.settings()
        chasis.drive_base.settings(vel_segura, accel_lin, vel_giro, accel_giro)
        
        dist_inicial = chasis.drive_base.distance()
        
        # 3. EL ÚNICO RECORRIDO (Modo asíncrono)
        chasis.drive_base.straight(distancia_total_mm, then=Stop.HOLD, wait=False)

        garra_disparada = False

        # 4. EL FRANCOTIRADOR (El vigía)
        while not chasis.drive_base.done():
            recorrido_actual = abs(chasis.drive_base.distance() - dist_inicial)

            if recorrido_actual >= trigger_mm and not garra_disparada:
                self.mover(
                    grados_garra, 
                    velocidad=vel_garra, 
                    wait_after=False, 
                    frenado=Stop.HOLD
                )
                garra_disparada = True

            wait(1)

        # 5. Seguro de vida
        if not garra_disparada:
            self.mover(grados_garra, velocidad=vel_garra, wait_after=False, frenado=Stop.HOLD)

        # 6. Restauramos la velocidad del chasis a su normalidad
        chasis.drive_base.settings(config.STRAIGHT_SPEED, accel_lin, vel_giro, accel_giro)

class Mecanismos:
    def __init__(self, motor_garra_delantera: Motor, motor_elevador_del: Motor, motor_garra_trasera: Motor):
        self.garra_delantera = GarraDelantera(motor_garra_delantera)
        self.elevador_delantero = ElevadorDelantero(motor_elevador_del)
        self.garra_trasera = GarraTrasera(motor_garra_trasera)