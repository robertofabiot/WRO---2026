from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

class Robot:
    def __init__(self, motor_izquierda, motor_derecha):
        self.prime_hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        self.motor_izquierda = motor_izquierda
        self.motor_derecha = motor_derecha
        self.drive_base = DriveBase(motor_izquierda, motor_derecha, 56, 160)

# region Giros    
    def giro_preciso(self, angulo_objetivo):
        """
        Gira el robot a un ángulo exacto usando el giroscopio y un Control Proporcional.
        """
        # 1. Reiniciamos el giroscopio a 0 grados antes de empezar a girar
        self.prime_hub.imu.reset_heading(0)

        # --- VARIABLES DE AJUSTE ---
        kp = 2.5           # Constante proporcional (agresividad del giro)
        min_speed = 50     # Velocidad mínima para que no se atasque al final
        tolerancia = 1     # Margen de error aceptable (en grados)

        while True:
            # 2. Leemos hacia dónde está apuntando el robot actualmente
            angulo_actual = self.prime_hub.imu.heading()

            # 3. Calculamos cuánto nos falta para llegar (el error)
            error = angulo_objetivo - angulo_actual

            # 4. Condición de salida: Si estamos lo suficientemente cerca, rompemos el bucle
            if abs(error) <= tolerancia:
                break

            # 5. Calculamos la velocidad de giro (mientras menor el error, más lento gira)
            turn_rate = error * kp

            # 6. Aseguramos que el robot no vaya tan lento que la fricción lo detenga antes de llegar
            if turn_rate > 0:
                turn_rate = max(turn_rate, min_speed)
            else:
                turn_rate = min(turn_rate, -min_speed)

            # 7. Movemos la base: 0 velocidad de avance, 'turn_rate' de giro
            self.drive_base.drive(0, turn_rate)

            # Pequeña pausa para estabilizar las lecturas del procesador
            wait(10)

        # 8. Frenamos los motores en seco al llegar al objetivo
        self.drive_base.stop()
# endregion

# region seguidor_linea
    def seguidor_linea(self, sensor_color):
        speed = 150
        kp = 3.6
        last_error = 0 
        kd = 1
        while True: 

            current_reflection = sensor_color.reflection()
            error = current_reflection - 35
            derivative = error - last_error
            correction = (error * kp) + (derivative * kd)

            self.motor_izquierda.dc(speed - correction)
            self.motor_derecha.dc(speed + correction)
# endregion

# region avance_y_retroceso
    def avanzar_cm(self, cm):
        mm = cm * 10
        self.drive_base.straight(mm)