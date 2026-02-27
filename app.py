from pybricks.parameters import Port, Color, Stop
from pybricks.pupdevices import ColorSensor
from robot import Robot

# Configuración de Hardware
mi_robot = Robot(port_izq=Port.A, port_der=Port.B, port_garra=Port.C)
sensor = ColorSensor(Port.D)

def cemento_y_llana():
    """
    Empieza: en el start point, viendo hacia los bloques de colores
    Termina: dejando el bloque de cemento, con la garra hacia arriba, viendo hacia la pared de la mesa
    """
    #Acomodarse para el seguidor de línea
    mi_robot.avanzar_recto(5) 
    mi_robot.mover_motor_izquierdo(500)

    #Empieza seguidor de línea
    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=100, tiempo_acomodo_ms=150)

    #Se acomoda para recoger el bloque de cemento
    mi_robot.mover_motor_izquierdo(-400)
    mi_robot.mover_motor_derecho(-450)

    #Recoge el bloque y deja la llana en su lugar
    mi_robot.mover_garra(-180)
    mi_robot.avanzar_recto(-38)

    #Se acomoda para el siguiente seguidor de línea
    mi_robot.mover_en_arco(-30, distancia_cm=15, stop = Stop.NONE)
    mi_robot.mover_en_arco(20, distancia_cm=18)

    #Empieza seguidor de línea y deja bloque de cemento
    mi_robot.seguidor_linea_distancia(sensor, 100, 56, tiempo_acomodo_ms=700)
    mi_robot.mover_motor_derecho(-400)
    mi_robot.mover_garra(90)

def bloques_blancos():
    """
    Empieza: dejando el bloque de cemento, con la garra hacia arriba, viendo hacia la pared de la mesa
    Termina: en el área de los bloques blancos, en posición de 45 grados 
    """
    #Sale del cemento
    mi_robot.avanzar_recto(10)

    #Se acomoda a la línea
    mi_robot.mover_motor_derecho(520)
    mi_robot.avanzar_recto(10)

    #Se acomoda para recoger los bloques
    mi_robot.seguidor_linea_distancia(sensor, 80, 10)
    mi_robot.giro_preciso(-180, kp_nuevo=2.8)
    mi_robot.avanzar_recto(-8)

    #Recoge los bloques
    mi_robot.mover_garra(-100)

    # Dejar los bloques blancos
    mi_robot.giro_preciso(55)
    mi_robot.avanzar_recto(62)
    mi_robot.seguidor_linea_distancia(sensor, 80, 22)
    mi_robot.giro_preciso(225, kp_nuevo=2.8)
    mi_robot.avanzar_recto(-18)

if __name__ == "__main__":
    cemento_y_llana()
    bloques_blancos()