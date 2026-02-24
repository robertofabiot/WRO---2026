from pybricks.parameters import Port
from robot import Robot

# Configuración de Hardware
mi_robot = Robot(port_izq=Port.A, port_der=Port.B, port_garra=Port.C)

def rutina_cubo_cemento():
    """
    Rutina principal: Recogida y entrega del bloque.
    """
    # 1. Salida de la base
    mi_robot.mover_en_arco(radio_cm=11, angulo=-35)  # radio 110mm -> 11cm
    mi_robot.mover_en_arco(radio_cm=-11, angulo=-35) # radio -110mm -> -11cm
    mi_robot.avanzar_recto(-64)                     # -66mm -> -6.6cm

    # 2. Acomodarse para el bloque
    mi_robot.mover_en_arco(radio_cm=-9, distancia_cm=-9)
    mi_robot.avanzar_recto(-7)
    
    # Ajuste fino con motor individual (Izquierda)
    mi_robot.mover_motor_izquierdo(grados=-370, velocidad=400)

    # 3. Agarrar el bloque (Bajar garra)
    mi_robot.mover_garra(grados=-200, velocidad=500)

    # 4. Llevar el bloque
    mi_robot.avanzar_recto(44) # 40mm -> 4cm

    # 5. Salida hacia la zona de entrega
    mi_robot.mover_en_arco(radio_cm=40, angulo=-30)
    mi_robot.avanzar_recto(-14)
    mi_robot.mover_en_arco(radio_cm=-20, angulo=-30)
    mi_robot.avanzar_recto(-52)
    
    # 6. Soltar bloque
    mi_robot.girar_sobre_eje(-92)
    mi_robot.mover_garra(grados=200, velocidad=500) # Subir garra

    # 7. Retirada final
    mi_robot.avanzar_recto(-5)
    mi_robot.avanzar_recto(10)
    mi_robot.giro_preciso(90)

if __name__ == "__main__":
    rutina_cubo_cemento()