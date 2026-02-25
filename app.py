from pybricks.parameters import Port, Color
from pybricks.pupdevices import ColorSensor
from robot import Robot

# Configuración de Hardware
mi_robot = Robot(port_izq=Port.A, port_der=Port.B, port_garra=Port.C)
sensor = ColorSensor(Port.D)

mi_robot.avanzar_recto(5)
mi_robot.mover_motor_izquierdo(500)
mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=100)
mi_robot.mover_motor_izquierdo(-400)
mi_robot.mover_motor_derecho(-450)
mi_robot.mover_garra(-180)
mi_robot.avanzar_recto(-38)

mi_robot.mover_en_arco(-30, distancia_cm=23)
mi_robot.mover_en_arco(7, 23)
mi_robot.seguidor_linea_distancia(sensor, 100, 65)
mi_robot.mover_motor_derecho(-480)
mi_robot.mover_garra(180)
mi_robot.avanzar_recto(10)
mi_robot.mover_motor_derecho(530)
mi_robot.avanzar_recto(10)
mi_robot.seguidor_linea_distancia(sensor, 60, 23)
