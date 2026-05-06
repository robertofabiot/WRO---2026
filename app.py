from pybricks.pupdevices import ColorSensor
from pybricks.tools import StopWatch, wait
from pybricks.parameters import Stop, Color
from robot import Robot
from ArmadorMosaicos import ArmadorMosaicos
from RevisadorBateria import RevisadorBateria
import config # <-- Importamos el nuevo archivo

# Configuración de Hardware usando config.py
mi_robot = Robot(
    port_izq=config.PORT_MOTOR_IZQ, 
    port_der=config.PORT_MOTOR_DER, 
    port_eje_central=config.PORT_EJE_CENTRAL, 
    port_garra_delantera=config.PORT_GARRA_DELANTERA
)
sensor = ColorSensor(config.PORT_SENSOR_FRENTE)
sensor_trasero = ColorSensor(config.PORT_SENSOR_TRASERO)

armador = ArmadorMosaicos(mi_robot, sensor)
revisador_bateria = RevisadorBateria(mi_robot)

def cemento_y_llana():
    """
    Empieza: en el start point, viendo hacia los bloques de colores
    Termina: dejando el bloque de cemento, con la garra hacia arriba, viendo hacia la pared de la mesa
    """
    # Acomodarse para seguidor
    mi_robot.chasis.mover_en_arco(radio_cm=13, distancia_cm=15, stop=Stop.NONE)

    # Seguidor de línea hasta cemento
    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=89, tiempo_acomodo_ms=0, margen_cm=10)

    # Acomodarse para cemento
    mi_robot.giro_preciso_pd(-94, margen_grados=5)
    mi_robot.mover_garra_trasera(53, velocidad=1200, wait_after=False) 
    mi_robot.chasis.avanzar_recto(-11, velocidad=1000, frenado=Stop.COAST, margen_cm=6)
    mi_robot.mover_eje_central(50)


    # Empujar la llana
    mi_robot.chasis.avanzar_recto(11, velocidad=1000, frenado=Stop.BRAKE, margen_cm=7)
    wait(1)
    mi_robot.chasis.mover_motor_derecho(-490, velocidad=1000, margen_grados=0) 
    wait(1)
    mi_robot.chasis.avanzar_recto(-18, velocidad=1000, frenado=Stop.NONE, margen_cm=3) # Llana dejada

    # Se acomoda para el siguiente seguidor de línea (Ambos arcos fluyen juntos)
    mi_robot.chasis.mover_en_arco(-50, distancia_cm=20)
    wait(1)
    mi_robot.chasis.mover_en_arco(19, angulo=25)

    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=69, tiempo_acomodo_ms=0, margen_cm=8)

    # Dejar cemento
    mi_robot.chasis.mover_motor_derecho(-420, velocidad=1000, margen_grados=20)
    mi_robot.mover_eje_central(-70, velocidad=1000, margen_grados=20) # Cemento dejado

def agarrar_bloques_blancos():
    """
    Empieza: dejando el bloque de cemento, con la garra hacia arriba, viendo hacia la pared de la mesa
    Termina: en el área de los bloques blancos, en posición de 45 grados viendo hacia afuera
    """
    # Sale del área de cemento para ir hacia los bloques blancos
    mi_robot.chasis.mover_en_arco(-14, distancia_cm=20, stop=Stop.BRAKE)

    # Seguidor hasta el cuadro blanco
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)
    mi_robot.seguir_hasta_interseccion(sensor, sensor_trasero, 50, kp=0.6)

    # Acomodo para agarrar bloques blancos 
    mi_robot.chasis.avanzar_recto(-10, velocidad=1000)
    mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
    mi_robot.chasis.giro_preciso(-175, margen_grados=5) #Antes no tenia margen
    mi_robot.chasis.mover_motor_derecho(30)
    mi_robot.chasis.avanzar_recto(-20, margen_cm=3) 

    # Agarrar bloques
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)

def dejar_bloques_blancos():
    mi_robot.chasis.avanzar_recto(5, velocidad=1000, frenado=Stop.NONE)
    mi_robot.chasis.mover_motor_izquierdo(370, velocidad=1000)

    mi_robot.chasis.mover_en_arco(radio_cm=-114, angulo=24, stop=Stop.NONE)

    # Seguidor hasta detectar el color verde
    mi_robot.seguidor_linea_color(sensor, 100, Color.GREEN, lado="derecha", distancia_cm=17)

    # Retrocede para no pegar en el mosaico
    mi_robot.chasis.avanzar_recto(-0.5)
    # Dejar los bloques
    mi_robot.giro_eje_puro(218, kp=5 ,kd=10, min_speed=200)
    mi_robot.chasis.avanzar_recto(-24, velocidad=1000, margen_cm=2)
    mi_robot.mover_garra_trasera(-80, margen_grados=20)

def detectar_mosaico():
    """
    Empieza: dejando los bloques blancos, en posición de 45 grados hacia afuera
    Termina: en el mosaico, con el sensor de color sobre el bloque de en medio de la primera fila
            (el que se detecta de primero)
    """
    # Acomodarse para seguidor
    mi_robot.chasis.avanzar_recto(14, velocidad=1000, frenado=Stop.NONE)
    mi_robot.chasis.mover_motor_derecho(250, margen_grados=30)

    # Seguir línea por estabilidad
    mi_robot.seguidor_linea_distancia(sensor, 60, 15, tiempo_acomodo_ms=0, kp=0.45, kd=1.8, k_freno=0.8) 

    # Retroceder hasta el mosaico
    mi_robot.chasis.avanzar_recto(-28 , velocidad=700) 

    # Escaneo y acomodo de ser necesario
    mi_robot.mover_garra_trasera(45, frenado=Stop.HOLD)
    wait(100)
    mosaico = mi_robot.identificar_combinacion(sensor_trasero, -5)
    print(f"{mosaico}" if mosaico != -1 else "la cagaste") #Para ver en consola la combinación detectada
    if (mosaico == 1 or mosaico == 2): # Condición para acomodos
        mi_robot.chasis.avanzar_recto(5)
    return mosaico

def agarrar_bloques_amarillos():
    """
    Empieza: en el mosaico, con el sensor de color sobre el bloque de en medio de la primera fila
            (el que se detecta de primero)
    Termina: con los bloques amarillos agarrados, listo para ir a dejarlos
    """
    #Camino para ir por los bloques amarillos 
    mi_robot.mover_garra_trasera(-50)
    mi_robot.seguidor_linea_distancia(sensor, 100, 19)
    mi_robot.giro_preciso_pd(-55)
    mi_robot.chasis.avanzar_recto(25)
    mi_robot.giro_preciso_pd(45)

    # Seguir hasta el cuadro amarillo
    mi_robot.seguidor_linea_color(sensor, 100, Color.YELLOW, distancia_cm=26)

    # Acomodo para agarrar bloques amarillos
    mi_robot.chasis.avanzar_recto(-10, velocidad=1000)
    mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
    mi_robot.chasis.giro_preciso(-175)
    mi_robot.chasis.mover_motor_derecho(30)
    mi_robot.chasis.avanzar_recto(-20) 

    # Agarrar bloques
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)

def dejar_bloques_amarillos():
    """
    Empieza: con los bloques amarillos agarrados, listo para ir a dejarlos
    Termina: dejando los bloques amarillos, viendo hacia la pared de la mesa
    """
    # Acomodarse para buscar la línea
    mi_robot.chasis.mover_motor_derecho(225, velocidad=1000, margen_grados=30)
    mi_robot.chasis.avanzar_recto(60, velocidad=1000, frenado=Stop.NONE)
    mi_robot.chasis.mover_motor_izquierdo(225, velocidad=1000, margen_grados=30)

    # Seguir la línea
    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=58 , lado="izquierda", tiempo_acomodo_ms=800)
    wait(500)

    # Dejar los bloques
    mi_robot.giro_preciso_pd(-90)
    mi_robot.chasis.avanzar_recto(-17, velocidad=1000)
    mi_robot.mover_garra_trasera(-55)

def recoger_bloques_azules():
    """
    Empieza: dejando los bloques amarillos, viendo hacia la pared de la mesa
    Termina: con los bloques azules agarrados, listo para salir
    """
    # Acomodarse para seguidor de línea
    mi_robot.chasis.avanzar_recto(15, velocidad=1000)
    mi_robot.giro_preciso_pd(-90)

    # Seguir línea
    mi_robot.seguidor_linea_distancia(sensor, 100, 50)

    # Buscar el siguiente seguidor
    mi_robot.giro_preciso_pd(-45)
    mi_robot.chasis.avanzar_recto(13, velocidad=1000)
    mi_robot.chasis.mover_motor_izquierdo(300)

    # Seguidor hasta el cuadro azul
    mi_robot.seguidor_linea_color(sensor, 100, Color.BLUE, distancia_cm=25)

    # Acomodo para agarrar bloques azules
    mi_robot.chasis.avanzar_recto(-10)
    mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
    mi_robot.chasis.giro_preciso(-175)
    mi_robot.chasis.mover_motor_derecho(30)
    mi_robot.chasis.avanzar_recto(-20) 

    # Agarrar bloques
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)

def dejar_bloques_azules_y_pala():
    """
    Empieza: con los bloques azules agarrados, listo para salir
    Termina: con los bloques azules cerca del inicio, viendo hacia los bloques del mosaico.
            La pala queda en el inicio.
    """
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)
    mi_robot.giro_preciso_pd(-35)
    mi_robot.chasis.avanzar_recto(52, velocidad=1100)

    # Giro para salir con la pala
    mi_robot.chasis.mover_motor_izquierdo(210)
    mi_robot.chasis.avanzar_recto(-10)
    mi_robot.chasis.giro_preciso(-180)
    mi_robot.chasis.avanzar_recto(-10)
    mi_robot.chasis.mover_motor_derecho(-150)
    mi_robot.chasis.avanzar_recto(-46)
    mi_robot.chasis.mover_motor_izquierdo(-120)
    mi_robot.chasis.avanzar_recto(-80)

    # Acá deja la pala en el inicio
    mi_robot.chasis.avanzar_recto(20, velocidad=1000)
    mi_robot.giro_preciso_pd(-90)

    # Dejamos los bloques azules en un espacio accesible 
    mi_robot.mover_garra_trasera(-55) 

def ejecutar_y_medir_tiempo():
    """
    Ejecuta las rutinas y calcula el tiempo exacto que tarda el robot en la vida real.
    """
    cronometro = StopWatch()
    
    # Reiniciamos y arrancamos el reloj justo antes de moverse
    cronometro.reset()
    cronometro.resume()
    
    print("Iniciando recorrido...")
    
    # Llamamos a las rutinas
    cemento_y_llana()
    agarrar_bloques_blancos()
    dejar_bloques_blancos()
    mosaico = detectar_mosaico()
    agarrar_bloques_amarillos()
    dejar_bloques_amarillos()
    recoger_bloques_azules()
    dejar_bloques_azules_y_pala()

    """Cuando se vayan a hacer las pruebas completas, quitar el '= 1' y usar la variable 
    mosaico almacenada después de la función 'detectar_mosaico'"""
    armador.armar(mosaico = 1)
    
    # Pausamos el reloj al terminar el último movimiento
    cronometro.pause()
    
    # Pybricks mide en milisegundos, lo convertimos a segundos para que sea más legible
    tiempo_ms = cronometro.time()
    tiempo_segundos = tiempo_ms / 1000
    
    # Imprimimos el resultado en la consola
    print(f"¡Recorrido completado!")
    print(f"Tiempo total: {tiempo_segundos} segundos.")
    
    return tiempo_segundos

if __name__ == "__main__":
    if not revisador_bateria.revisar_bateria():
        print("Ejecución cancelada.")
    else:
        # cemento_y_llana()
        # agarrar_bloques_blancos()
        # dejar_bloques_blancos()
        # numero_mosaico = detectar_mosaico()
        # agarrar_bloques_amarillos()
        # dejar_bloques_amarillos()
        # recoger_bloques_azules()
        # dejar_bloques_azules_y_pala()
    
        """Cuando se vayan a hacer las pruebas completas, quitar el '= 1' y usar la variable 
        mosaico almacenada después de la función 'detectar_mosaico'"""
        armador.armar(numero_mosaico = 1)