import usys
import uselect
from pybricks.tools import wait
from robot import Robot
import config

def main():
    print("========================================")
    print("Inicializando robot para Control Remoto...")
    
    mi_robot = Robot(
        port_izq=config.PORT_MOTOR_IZQ, 
        port_der=config.PORT_MOTOR_DER, 
        port_garra_trasera=config.PORT_GARRA_TRASERA, 
        port_garra_delantera=config.PORT_GARRA_DELANTERA,
        port_pinza=config.PORT_PINZA
    )

    # Configurar el polling del teclado (standard input)
    keyboard = uselect.poll()
    keyboard.register(usys.stdin)
    
    print("========================================")
    print("       CONTROL REMOTO ACTIVADO")
    print("========================================")
    print("Controles del Chasis (Incrementales):")
    print("  W : Aumentar velocidad hacia adelante")
    print("  S : Aumentar velocidad hacia atrás")
    print("  A : Aumentar giro a la izquierda")
    print("  D : Aumentar giro a la derecha")
    print("  Espacio : Detener chasis por completo")
    print("\nControles de la Garra Delantera:")
    print("  I / K : Subir / Bajar elevador")
    print("  J / L : Abrir / Cerrar pinza")
    print("\nControles de la Garra Trasera:")
    print("  U / O : Subir / Bajar garra trasera")
    print("\nOpciones:")
    print("  Q : Salir del programa")
    print("========================================")

    speed = 0
    turn_rate = 0
    max_speed = config.STRAIGHT_SPEED
    max_turn = config.TURN_RATE
    step_speed = 100
    step_turn = 100
    grados_mecanismo = 45 # Cuantos grados se mueve el mecanismo por pulsación

    while True:
        # Procesar entrada del teclado sin bloquear
        if keyboard.poll(0):
            cmd = usys.stdin.read(1)
            if cmd:
                cmd = cmd.lower()
                
                # ------ Chasis ------
                if cmd == 'w':
                    speed += step_speed
                    if speed > max_speed: speed = max_speed
                elif cmd == 's':
                    speed -= step_speed
                    if speed < -max_speed: speed = -max_speed
                elif cmd == 'a':
                    turn_rate -= step_turn
                    if turn_rate < -max_turn: turn_rate = -max_turn
                elif cmd == 'd':
                    turn_rate += step_turn
                    if turn_rate > max_turn: turn_rate = max_turn
                elif cmd == ' ':
                    speed = 0
                    turn_rate = 0
                    
                # ------ Mecanismos (Garra Delantera) ------
                elif cmd == 'i':
                    mi_robot.garra_delantera.subir(grados_mecanismo, wait_after=False)
                elif cmd == 'k':
                    mi_robot.garra_delantera.bajar(grados_mecanismo, wait_after=False)
                elif cmd == 'j':
                    mi_robot.garra_delantera.abrir(grados_mecanismo, wait_after=False)
                elif cmd == 'l':
                    mi_robot.garra_delantera.cerrar(grados_mecanismo, wait_after=False)
                    
                # ------ Mecanismos (Garra Trasera) ------
                elif cmd == 'u':
                    mi_robot.garra_trasera.subir(grados_mecanismo, wait_after=False)
                elif cmd == 'o':
                    mi_robot.garra_trasera.bajar(grados_mecanismo, wait_after=False)
                
                # ------ Salir ------
                elif cmd == 'q':
                    print("Saliendo del Control Remoto...")
                    mi_robot.drive_base.stop()
                    # Detener motores de mecanismos también
                    mi_robot.motor_garra_delantera.stop()
                    mi_robot.motor_garra_trasera.stop()
                    if hasattr(mi_robot, 'motor_pinza'):
                        mi_robot.motor_pinza.stop()
                    break

        # Aplicar el movimiento al chasis continuamente
        if speed == 0 and turn_rate == 0:
            mi_robot.drive_base.stop()
        else:
            mi_robot.drive_base.drive(speed, turn_rate)
            
        # Pequeña pausa para no saturar el procesador del hub
        wait(20)

if __name__ == "__main__":
    main()
