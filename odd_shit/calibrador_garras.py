from pybricks.tools import wait
from robot import Robot
import config

def calibrar(nombre, funcion_cero, funcion_maximo, motor):
    print("\n" + "-"*40)
    print(f"--- Iniciando calibración: {nombre} ---")
    print("1. Buscando el INICIO (cero) a baja velocidad...")
    
    # Reducimos la velocidad y el límite de potencia para no forzar los engranes al topar
    funcion_cero(velocidad=250, limite_potencia=35)
    
    print("   -> ¡Inicio encontrado! Estableciendo a 0 grados.")
    motor.reset_angle(0)
    wait(500)
    
    print("2. Buscando el FIN (rango máximo) a baja velocidad...")
    funcion_maximo(velocidad=250, limite_potencia=35)
    
    max_grados = motor.angle()
    print("   -> ¡Fin encontrado!")
    
    print("\n" + "="*45)
    print(f"✅ CALIBRACIÓN COMPLETADA PARA {nombre}")
    print("="*45)
    print(f"🌟 VALOR A INYECTAR EN robot.py:  {max_grados}  🌟")
    print("="*45)
    
    # Volver al inicio por cortesía
    print("Regresando al origen...")
    funcion_cero(velocidad=500, limite_potencia=50)

def main():
    print("========================================")
    print("        CALIBRADOR DE MECANISMOS        ")
    print("========================================")
    print("Inicializando robot...")
    
    mi_robot = Robot(
        port_izq=config.PORT_MOTOR_IZQ, 
        port_der=config.PORT_MOTOR_DER, 
        port_garra_trasera=config.PORT_GARRA_TRASERA, 
        port_garra_delantera=config.PORT_GARRA_DELANTERA,
        port_pinza=config.PORT_PINZA
    )
    
    while True:
        print("\n¿Qué mecanismo deseas calibrar?")
        print("1 - Garra Trasera")
        print("2 - Garra Delantera (Elevador)")
        print("3 - Garra Delantera (Pinza)")
        print("Q - Salir")
        
        print("Escribe el número y presiona Enter: ", end="")
        opcion = input().strip().lower()
        
        if opcion == '1':
            calibrar(
                "Garra Trasera", 
                mi_robot.garra_trasera.subir_al_tope,
                mi_robot.garra_trasera.bajar_al_tope,
                mi_robot.motor_garra_trasera
            )
        elif opcion == '2':
            calibrar(
                "Garra Delantera (Elevador)", 
                mi_robot.garra_delantera.subir_al_tope,
                mi_robot.garra_delantera.bajar_al_tope,
                mi_robot.motor_garra_delantera
            )
        elif opcion == '3':
            calibrar(
                "Garra Delantera (Pinza)", 
                mi_robot.garra_delantera.abrir_al_tope,
                mi_robot.garra_delantera.cerrar_al_tope,
                mi_robot.motor_pinza
            )
        elif opcion == 'q':
            print("Saliendo del calibrador...")
            break
        else:
            print("Opción inválida. Intenta de nuevo.")

if __name__ == "__main__":
    main()
