from pybricks.parameters import Port, Color, Stop
from pybricks.pupdevices import ColorSensor
from pybricks.tools import StopWatch, wait
from robot import Robot

#Variables globales
mosaicos = {Color.GREEN: {Color.GREEN: 1, Color.YELLOW: 2}, Color.BLUE: 3, Color.YELLOW: 4, Color.WHITE: 5}

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
    mi_robot.mover_garra(53, velocidad=1200, wait_after=False) 
    mi_robot.avanzar_recto(-12, frenado=Stop.HOLD)
    mi_robot.mover_garra_segura(49, velocidad=2500, empuje_cm=2) 

    # Empujar el otro
    mi_robot.avanzar_recto(9, frenado=Stop.BRAKE)
    mi_robot.mover_motor_derecho(-540, velocidad=1000) # Más rápido
    
    # 4. ENCADENAMIENTO: Retrocede y fluye directamente hacia la primera curva
    mi_robot.avanzar_recto(-15, frenado=Stop.NONE)

    # Se acomoda para el siguiente seguidor de línea (Ambos arcos fluyen juntos)
    mi_robot.mover_en_arco(-9, distancia_cm=5, stop=Stop.COAST)
    mi_robot.mover_en_arco(9, distancia_cm=4, stop=Stop.NONE) 
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
    mi_robot.seguidor_linea_distancia(sensor, 50, 9, lado="izquierda", tiempo_acomodo_ms=800) 

    # Avanza para posicionarse sobre los bloques
    mi_robot.mover_garra(55, velocidad=1000)
    mi_robot.avanzar_recto(-10)

    mi_robot.mover_garra(-30, velocidad=700) #Antes eran 1000 de velocidad
    mi_robot.avanzar_recto(-11) #antes retrocedia 9cm

    mi_robot.mover_garra_dc(30, potencia=100, empuje_cm=1) #Garra abajo, bloques agarrados. antes potencia estaba en 100

    mi_robot.giro_preciso_pd(60) #Antes era 55 grados en direccion al camino del mosaico
    mi_robot.avanzar_recto(58, frenado=Stop.COAST_SMART) #Originalmente 61

    mi_robot.giro_preciso_pd(-45)

    mi_robot.seguidor_linea_color(sensor, 100, Color.GREEN, lado="derecha", distancia_cm=30)
    mi_robot.avanzar_recto(-0.5)
    mi_robot.giro_eje_puro(217, kp=5 ,kd=10, min_speed=200)
    mi_robot.avanzar_recto(-22)
    mi_robot.mover_garra(-60)
    

def detectar_mosaico():
    """
    Empieza: dejando los bloques blancos, en posición de 45 grados hacia afuera
    Termina: n/a 
    """
    #acomodarse para seguidor
    mi_robot.avanzar_recto(24)
    mi_robot.mover_motor_derecho(250)

    #seguir línea (a veces no es necesario, pero prefiero hacerlo por estabilidad)
    mi_robot.seguidor_linea_distancia(sensor, 75, 14) #velocidad antes en 80 al seguir linea en direccion de los bloques verdes
    mi_robot.avanzar_recto(-35 , velocidad=700) #velocidad anterior con 1000 de velocidad(Mucho margen de error alv)
    mi_robot.mover_garra(45, frenado=Stop.HOLD)
    wait(500)
    mosaico = mi_robot.identificar_combinacion(sensor_trasero, -5)
    print(f"{mosaico}" if mosaico != -1 else "la cagaste") #Para ver en consola la combinación detectada
    return mosaico

def rutina_bloques_amarillos(mosaico):

    if (mosaico == 1 or mosaico == 2):
        mi_robot.avanzar_recto(5)

    #Camino para ir por los bloques amarillos 
    mi_robot.mover_garra(-50)
    mi_robot.seguidor_linea_distancia(sensor, 80, 25)
    mi_robot.giro_preciso_pd(-55)
    mi_robot.avanzar_recto(25)
    mi_robot.giro_preciso_pd(45)
    mi_robot.seguidor_linea_distancia(sensor, 90, 8)

    # Gira para meterse a dejar los bloques amarillos
    mi_robot.giro_preciso_pd(-180)
    mi_robot.seguidor_linea_distancia(sensor, 50, 9, lado="izquierda", tiempo_acomodo_ms=800) 

    # Avanza para posicionarse sobre los bloques
    mi_robot.mover_garra(55, velocidad=1000)
    mi_robot.avanzar_recto(-9)
    mi_robot.avanzar_recto(-13) #antes retrocedia 9cm
    mi_robot.mover_garra(-55, velocidad=700, frenado=Stop.HOLD)


def dejar_bloques_amarillos():
    #Rutina para dejar los bloques amarillos en su lugar
    mi_robot.avanzar_recto(5)
    # mi_robot.seguidor_linea_distancia(sensor, 80, 5)
    mi_robot.giro_preciso_pd(-65)
    mi_robot.avanzar_recto(70, velocidad=1000)
    mi_robot.seguidor_linea_distancia(sensor, 100,  47, lado="izquierda")
    mi_robot.giro_preciso_pd(-90)
    mi_robot.avanzar_recto(-20)
    mi_robot.mover_garra(-55)
    mi_robot.avanzar_recto(15)
    mi_robot.giro_preciso_pd(-90)
    mi_robot.seguidor_linea_distancia(sensor, 100, 55)
    mi_robot.giro_preciso_pd(-45)
    mi_robot.avanzar_recto(17)
    mi_robot.seguidor_linea_distancia(sensor, 100, 9, lado="izquierda")
    mi_robot.giro_preciso_pd(180)
    mi_robot.seguidor_linea_distancia(sensor, 60, 9)
    mi_robot.avanzar_recto(-7)
    mi_robot.mover_garra_dc(55)

    
    


    # mi_robot.avanzar_recto(10)
    # mi_robot.giro_preciso_pd(-65)
    # mi_robot.avanzar_recto(69)
    # mi_robot.giro_preciso_pd(65)
    # mi_robot.seguidor_linea_distancia(sensor, 80, 54)
    # mi_robot.giro_preciso_pd(-90)


    # #Rutina para ir por los bloques azules y dejar la palita en el inicio 
    # mi_robot.avanzar_recto(-19)
    # mi_robot.mover_garra(200)
    # mi_robot.avanzar_recto(19)
    # mi_robot.giro_preciso_pd(-90)
    # mi_robot.seguidor_linea_distancia(sensor, 80, 42)
    # mi_robot.giro_preciso_pd(-35)
    # mi_robot.avanzar_recto(23)
    # mi_robot.giro_preciso_pd(35)
    # mi_robot.seguidor_linea_distancia(sensor, 80, 10)
    # mi_robot.giro_preciso_pd(90)
    # mi_robot.giro_preciso_pd(90)
    # mi_robot.avanzar_recto(-11)

    # #Agarra los bloques azules
    # mi_robot.mover_garra(-210)
    # mi_robot.giro_preciso_pd(-15)
    # mi_robot.avanzar_recto(65)

    # #Giro para salir con la llana
    # mi_robot.mover_motor_izquierdo(255)
    # mi_robot.avanzar_recto(31)
    # mi_robot.seguidor_linea_distancia(sensor, 80, 85)

    # #Acá deja la llana en el inicio
    # mi_robot.mover_motor_derecho(250)
    # mi_robot.mover_motor_derecho(-250)
    # mi_robot.avanzar_recto(-16)
    # mi_robot.giro_preciso_pd(95)

    # #Dejamos los bloques azules en un espacio accesible 
    # mi_robot.mover_garra(200)

    
def ejecutar_y_medir_tiempo():
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
    mosaico = detectar_mosaico()
    rutina_bloques_amarillos(mosaico)
    dejar_bloques_amarillos()