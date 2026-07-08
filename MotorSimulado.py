from pybricks.pupdevices import Motor
from pybricks.parameters import Port

# 1. Creamos una clase que imite los comandos de un motor real
class MotorSimulado:
    def __init__(self, port):
        self.port = port
        print(f"⚠️ [AVISO] Usando Motor SIMULADO en {port}")

    def run(self, speed):
        pass # No hace nada, pero evita que el programa de error

    def run_target(self, speed, target_angle):
        pass

    def stop(self):
        pass

    def angle(self):
        return 0 # Finge que el ángulo actual siempre es 0

# 2. Intentamos conectar el motor real
try:
    motor_izquierdo = Motor(Port.A)
except OSError:
    # 3. Si falla (porque no está conectado), usamos el simulado
    motor_izquierdo = MotorSimulado(Port.A)

# --- A partir de aquí tu código fluye normal ---
# Pybricks creerá que tiene un motor, ya sea real o el simulado.

motor_izquierdo.run(500) 
print("El código sigue funcionando perfectamente.")