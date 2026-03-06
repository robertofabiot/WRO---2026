from pybricks.parameters import Port, Color, Stop
from pybricks.pupdevices import ColorSensor
from pybricks.tools import StopWatch, wait
from robot import Robot



# Configuración de Hardware
mi_robot = Robot(port_izq=Port.A, port_der=Port.B, port_garra=Port.C)
sensor = ColorSensor(Port.D)
sensor_trasero = ColorSensor(Port.E)

def cemento_y_llana():
    """
    Empieza: en el start point, viendo hacia los bloques de colores
    Termina: dejando el bloque de cemento, con la garra hacia arriba, viendo hacia la pared de la mesa
    """
    # 1. FLUIDEZ: El arco termina en Stop.NONE, entra al seguidor sin frenar
    mi_robot.mover_en_arco(radio_cm=15, distancia_cm=27, stop=Stop.COAST_SMART)

    # 2. INERCIA: Como viene del arco sin frenar, el tiempo_acomodo_ms se vuelve 0 para no frenarlo artificialmente
    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=62, tiempo_acomodo_ms=0)

    # Agarrar bloque
    mi_robot.giro_preciso_pd(-90)
    
    # 3. ASINCRONÍA: La garra empieza a moverse MIENTRAS el robot retrocede
    mi_robot.mover_garra(53, velocidad=1000, wait_after=False) 
    mi_robot.avanzar_recto(-12, frenado=Stop.HOLD)
    mi_robot.mover_garra(45, velocidad=2000) 

    # Empujar el otro
    mi_robot.avanzar_recto(10, frenado=Stop.BRAKE)
    mi_robot.mover_motor_derecho(-540, velocidad=1000) # Más rápido
    
    # 4. ENCADENAMIENTO: Retrocede y fluye directamente hacia la primera curva
    mi_robot.avanzar_recto(-13, frenado=Stop.NONE)

    # Se acomoda para el siguiente seguidor de línea (Ambos arcos fluyen juntos)
    mi_robot.mover_en_arco(-9, distancia_cm=4, stop=Stop.COAST)
    mi_robot.mover_en_arco(9, distancia_cm=3, stop=Stop.NONE) 
    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=71, tiempo_acomodo_ms=200)

    mi_robot.mover_motor_derecho(-440, velocidad=800)
    mi_robot.mover_garra(-50, velocidad=50)

def bloques_blancos():
    """
    Empieza: dejando el bloque de cemento, con la garra hacia arriba, viendo hacia la pared de la mesa
    Termina: en el área de los bloques blancos, en posición de 45 grados 
    """
    # 1. FLUIDEZ: Salimos del arco a toda velocidad (Stop.NONE)
    mi_robot.mover_en_arco(-17, distancia_cm=28, stop=Stop.BRAKE)

    # 2. INERCIA: Como el robot ya viene moviéndose del arco, eliminamos el tiempo de acomodo (0ms)
    mi_robot.seguidor_linea_distancia(sensor, 90, 8)

    # Gira para meterse a dejar los bloques blancos 
    mi_robot.giro_preciso_pd(-180)
    mi_robot.seguidor_linea_distancia(sensor, 60, 7, lado="izquierda", tiempo_acomodo_ms=800) 

    # Avanza para posicionarse sobre los bloques
    mi_robot.mover_garra(55, velocidad=1000)
    mi_robot.avanzar_recto(-13)

    mi_robot.mover_garra(-55, velocidad=1000)
    mi_robot.avanzar_recto(-9)

    mi_robot.mover_garra(55, velocidad=1000)


    # 3. ASINCRONÍA + VELOCIDAD: Empezamos a cerrar la garra rápido (800) y SIN esperar.
    # El robot empezará a hacer el giro de 50 grados MIENTRAS la garra se cierra, ahorrando casi un segundo.
    

    mi_robot.giro_preciso_pd(55)
    mi_robot.avanzar_recto(54, frenado=Stop.COAST_SMART) #Originalmente 61

    mi_robot.giro_preciso_pd(-45)

    mi_robot.seguidor_linea_distancia(sensor, 80, 28, lado="izquierda", tiempo_acomodo_ms=800)

    mi_robot.giro_preciso_pd(180)
    mi_robot.giro_preciso_pd(49)


    mi_robot.avanzar_recto(-16)
    mi_robot.mover_garra(-55, velocidad=800)
    mi_robot.avanzar_recto(22)
    mi_robot.giro_preciso(-45)
    mi_robot.avanzar_recto(-10)

    #Mueca para detectar los colores del mosaico 
    mi_robot.mover_garra(40, velocidad=300)

    #camino a los bloques amarillos 

    mi_robot.seguidor_linea_distancia(sensor, 100, 10, tiempo_acomodo_ms=500)
    mi_robot.giro_preciso_pd(-45)
    mi_robot.avanzar_recto(33)

    mi_robot.seguidor_linea_distancia(sensor, 90, 8, lado="izquierda")

    # Gira para meterse a dejar los bloques blancos 
    mi_robot.giro_preciso_pd(-180)
    mi_robot.seguidor_linea_distancia(sensor, 60, 7, tiempo_acomodo_ms=800) 

    # Avanza para posicionarse sobre los bloques
    mi_robot.mover_garra(5, velocidad=800)
    mi_robot.avanzar_recto(-13)

    mi_robot.mover_garra(-15, velocidad=800)
    mi_robot.avanzar_recto(-9)

    mi_robot.mover_garra(55, velocidad=1000)
    
    
def ejecutar_y_medir_tiempo():
      
    # 4. ENCADENAMIENTO: Esta es una recta larga (60cm). En lugar de frenar al final, 
    # le decimos que pase de largo (Stop.NONE) para entrar a la línea volando.
    
    
    # 5. INERCIA MAXIMIZADA: Entra al seguidor con todo el impulso de la recta anterior.
    # Redujimos el acomodo de 700ms a 0ms.
    
    
    mi_robot.giro_preciso(220, kp_nuevo=1.5)
    
    # Retrocede a dejar los bloques
    mi_robot.avanzar_recto(-17)
    
    # Abrir la garra rápido para soltar. 
    # Nota: Si esta es tu ÚLTIMA instrucción de toda la corrida, déjala con wait_after=True 
    # para que no se corte por el problema de las "instrucciones fantasma" que vimos antes.
    mi_robot.mover_garra(-105, velocidad=800)
    """
    Ejecuta las rutinas y calcula el tiempo exacto que tarda el robot en la vida real.
    """
    cronometro = StopWatch()
    
    # Reiniciamos y arrancamos el reloj justo antes de moverse
    cronometro.reset()
    cronometro.resume()
    
    print("🤖 Iniciando recorrido...")
    
    # Llamamos a tus rutinas
    cemento_y_llana()
    bloques_blancos()
    
    # Pausamos el reloj al terminar el último movimiento
    cronometro.pause()
    
    # Pybricks mide en milisegundos, lo convertimos a segundos para que sea más legible
    tiempo_ms = cronometro.time()
    tiempo_segundos = tiempo_ms / 1000
    
    # Imprimimos el resultado en la consola
    print(f"🏁 ¡Recorrido completado!")
    print(f"⏱️ Tiempo total: {tiempo_segundos} segundos.")
    
    return tiempo_segundos

if __name__ == "__main__":

    #ejecutar_y_medir_tiempo()

    cemento_y_llana()

    bloques_blancos()