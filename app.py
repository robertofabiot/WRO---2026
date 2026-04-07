from pybricks.parameters import Port, Color, Stop
from pybricks.pupdevices import ColorSensor
from pybricks.tools import StopWatch, wait
from robot import Robot

# Variables globales
mosaicos = {Color.GREEN: {Color.GREEN: 1, Color.YELLOW: 2}, Color.BLUE: 3, Color.YELLOW: 4, Color.WHITE: 5}

# Configuración de Hardware
mi_robot = Robot(port_izq=Port.A, port_der=Port.B, port_eje_central=Port.C, port_garra_delantera=Port.F)
sensor = ColorSensor(Port.D)
sensor_trasero = ColorSensor(Port.E)

def cemento_y_llana():
    """
    Empieza: en el start point, viendo hacia los bloques de colores
    Termina: dejando el bloque de cemento, con la garra hacia arriba, viendo hacia la pared de la mesa
    """
    # Acomodarse para seguidor
    mi_robot.mover_en_arco(radio_cm=13, distancia_cm=14, stop=Stop.NONE)

    # Seguidor de línea hasta cemento
    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=83, tiempo_acomodo_ms=0, margen_cm=10)

    # Acomodarse para cemento
    mi_robot.giro_preciso_pd(-96, margen_grados=14)
    mi_robot.mover_garra_trasera(53, velocidad=1200, wait_after=False) 
    mi_robot.avanzar_recto(-13,velocidad=1000, frenado=Stop.COAST, margen_cm=6)
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)


    # Empujar la llana
    mi_robot.avanzar_recto(11, velocidad=1000, frenado=Stop.BRAKE, margen_cm=7)
    mi_robot.mover_motor_derecho(-760, velocidad=1000, margen_grados=300) 
    mi_robot.avanzar_recto(-22, velocidad=1000, frenado=Stop.NONE, margen_cm=9) # Llana dejada

    # Se acomoda para el siguiente seguidor de línea (Ambos arcos fluyen juntos)
    mi_robot.mover_en_arco(-17, distancia_cm=4, stop=Stop.NONE)
    mi_robot.mover_en_arco(20, distancia_cm=6, stop=Stop.NONE)

    # Seguidor de línea para dejar cemento
    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=62, tiempo_acomodo_ms=0, margen_cm=8)

    # Dejar cemento
    mi_robot.mover_motor_derecho(-620, velocidad=1000, margen_grados=400)
    mi_robot.mover_eje_central(-60, velocidad=1000, margen_grados=30) # Cemento dejado

def agarrar_bloques_blancos():
    """
    Empieza: dejando el bloque de cemento, con la garra hacia arriba, viendo hacia la pared de la mesa
    Termina: en el área de los bloques blancos, en posición de 45 grados viendo hacia afuera
    """
    # Sale del área de cemento para ir hacia los bloques blancos
    mi_robot.mover_en_arco(-14, distancia_cm=20, stop=Stop.BRAKE)

    # Seguidor hasta el cuadro blanco
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)
    mi_robot.seguir_hasta_interseccion(sensor, sensor_trasero, 50, kp=0.6)

    # Acomodo para agarrar bloques blancos 
    mi_robot.avanzar_recto(-10, velocidad=1000)
    mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
    mi_robot.giro_preciso(-175, margen_grados=5) #Antes no tenia margen
    mi_robot.mover_motor_derecho(30)
    mi_robot.avanzar_recto(-20, margen_cm=3) 

    # Agarrar bloques
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)

def dejar_bloques_blancos():
    mi_robot.avanzar_recto(5, velocidad=1000, frenado=Stop.NONE)
    mi_robot.motor_izquierda.run_angle(speed=1000, rotation_angle=370, then=Stop.NONE)

    mi_robot.mover_en_arco(radio_cm=-114, angulo=24, stop=Stop.NONE)

    # Seguidor hasta detectar el color verde
    mi_robot.seguidor_linea_color(sensor, 100, Color.GREEN, lado="derecha", distancia_cm=17)

    # Retrocede para no pegar en el mosaico
    mi_robot.avanzar_recto(-0.5)
    # Dejar los bloques
    mi_robot.giro_eje_puro(218, kp=5 ,kd=10, min_speed=200)
    mi_robot.avanzar_recto(-24, velocidad=1000, margen_cm=2)
    mi_robot.mover_garra_trasera(-80, margen_grados=20)

def detectar_mosaico():
    """
    Empieza: dejando los bloques blancos, en posición de 45 grados hacia afuera
    Termina: en el mosaico, con el sensor de color sobre el bloque de en medio de la primera fila
            (el que se detecta de primero)
    """
    # Acomodarse para seguidor
    mi_robot.avanzar_recto(14, velocidad=1000, frenado=Stop.NONE)
    mi_robot.mover_motor_derecho(250, margen_grados=30)

    # Seguir línea por estabilidad
    mi_robot.seguidor_linea_distancia(sensor, 60, 15, tiempo_acomodo_ms=0, kp=0.45, kd=1.8, k_freno=0.8) 

    # Retroceder hasta el mosaico
    mi_robot.avanzar_recto(-28 , velocidad=700) 

    # Escaneo y acomodo de ser necesario
    mi_robot.mover_garra_trasera(45, frenado=Stop.HOLD)
    wait(100)
    mosaico = mi_robot.identificar_combinacion(sensor_trasero, -5)
    print(f"{mosaico}" if mosaico != -1 else "la cagaste") #Para ver en consola la combinación detectada
    if (mosaico == 1 or mosaico == 2): # Condición para acomodos
        mi_robot.avanzar_recto(5)
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
    mi_robot.avanzar_recto(25)
    mi_robot.giro_preciso_pd(45)

    # Seguir hasta el cuadro amarillo
    mi_robot.seguidor_linea_color(sensor, 100, Color.YELLOW, distancia_cm=26)

    # Acomodo para agarrar bloques amarillos
    mi_robot.avanzar_recto(-10, velocidad=1000)
    mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
    mi_robot.giro_preciso(-175)
    mi_robot.mover_motor_derecho(30)
    mi_robot.avanzar_recto(-20) 

    # Agarrar bloques
    mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)

def dejar_bloques_amarillos():
    """
    Empieza: con los bloques amarillos agarrados, listo para ir a dejarlos
    Termina: dejando los bloques amarillos, viendo hacia la pared de la mesa
    """
    # Acomodarse para buscar la línea
    mi_robot.mover_motor_derecho(225, velocidad=1000, margen_grados=30)
    mi_robot.avanzar_recto(60, velocidad=1000, frenado=Stop.NONE)
    mi_robot.mover_motor_izquierdo(225, velocidad=1000, margen_grados=30)

    # Seguir la línea
    mi_robot.seguidor_linea_distancia(sensor, 100, distancia_cm=58 , lado="izquierda", tiempo_acomodo_ms=800)
    wait(500)

    # Dejar los bloques
    mi_robot.giro_preciso_pd(-90)
    mi_robot.avanzar_recto(-17, velocidad=1000)
    mi_robot.mover_garra_trasera(-55)

def recoger_bloques_azules():
    """
    Empieza: dejando los bloques amarillos, viendo hacia la pared de la mesa
    Termina: con los bloques azules agarrados, listo para salir
    """
    # Acomodarse para seguidor de línea
    mi_robot.avanzar_recto(15, velocidad=1000)
    mi_robot.giro_preciso_pd(-90)

    # Seguir línea
    mi_robot.seguidor_linea_distancia(sensor, 100, 50)

    # Buscar el siguiente seguidor
    mi_robot.giro_preciso_pd(-45)
    mi_robot.avanzar_recto(13, velocidad=1000)
    mi_robot.mover_motor_izquierdo(300)

    # Seguidor hasta el cuadro azul
    mi_robot.seguidor_linea_color(sensor, 100, Color.BLUE, distancia_cm=25)

    # Acomodo para agarrar bloques azules
    mi_robot.avanzar_recto(-10)
    mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
    mi_robot.giro_preciso(-175)
    mi_robot.mover_motor_derecho(30)
    mi_robot.avanzar_recto(-20) 

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
    mi_robot.avanzar_recto(52, velocidad=1100)

    # Giro para salir con la pala
    mi_robot.mover_motor_izquierdo(210)
    mi_robot.avanzar_recto(-10)
    mi_robot.giro_preciso(-180)
    mi_robot.avanzar_recto(-10)
    mi_robot.mover_motor_derecho(-150)
    mi_robot.avanzar_recto(-46)
    mi_robot.mover_motor_izquierdo(-120)
    mi_robot.avanzar_recto(-80)

    # Acá deja la pala en el inicio
    
    mi_robot.avanzar_recto(20, velocidad=1000)
    mi_robot.giro_preciso_pd(-90)

    # Dejamos los bloques azules en un espacio accesible 
    mi_robot.mover_garra_trasera(-55) 

def armar_mosaico(mosaico):
    """
    Empieza: con los bloques azules cerca del inicio, viendo hacia los bloques del mosaico.
             La pala queda en el inicio.
    Termina: n/a
    """
    if mosaico == 1: # verde - verde
        # Mandar la garra central abajo si no lo está
        mi_robot.llevar_eje_central_al_tope("negativo")
        mi_robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)

        # Acomodo para agarrar dos azules y dos verdes
        mi_robot.seguidor_linea_distancia_desacelerado(sensor, 100, 51, margen_cm=5, tiempo_acomodo_ms=500)
        mi_robot.giro_preciso(-87)
        

        # Entrada
        mi_robot.abrir_garra_delantera(170, velocidad=1000)
        mi_robot.avanzar_recto(10, velocidad=1000)
        mi_robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
        mi_robot.avanzar_recto(-5)
        mi_robot.abrir_garra_delantera(170, velocidad=1000)
        mi_robot.avanzar_recto(10, velocidad=1000)
        mi_robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
        mi_robot.avanzar_recto(-25)
        mi_robot.abrir_garra_delantera(100, velocidad=1000, margen_grados=20)
        mi_robot.avanzar_recto(7)
        mi_robot.avanzar_recto(-3.5)
        mi_robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
    
        mi_robot.girar_sobre_eje(200, margen_grados=20)
        mi_robot.mover_motor_izquierdo(180, velocidad=1000, margen_grados=10)
        mi_robot.avanzar_recto(3, velocidad=1000, frenado=Stop.NONE)
        mi_robot.mover_motor_derecho(250, velocidad=1000, margen_grados=10)

        mi_robot.seguidor_linea_color(sensor, 100, Color.BLUE, lado="izquierda", distancia_cm=20)

        # Subida de garra
        mi_robot.llevar_eje_central_al_tope("positivo", velocidad=400, limite_potencia=100)

        # Acomodo para que queden en su lugar
        mi_robot.mover_en_arco(-9, distancia_cm=3.6, stop=Stop.COAST)
        mi_robot.mover_en_arco(9.5, distancia_cm=2, stop=Stop.NONE) 
        mi_robot.avanzar_recto(7)
        mi_robot.mover_motor_izquierdo(30)

        # Soltar
        # 1. Bajar la garra hasta la altura de "jaula" (sin aplastar la impresión 3D)
        mi_robot.mover_garra_trasera(-110)

        # 2. Abrir la garra delantera ligeramente para dar holgura a los bloques
        mi_robot.abrir_garra_delantera(grados=50, velocidad=400) 

        # 3. ¡Vibrar!
        mi_robot.sacudir(iteraciones=3, potencia=60, tiempo_ms=100)

        # 4. Soltar por completo y salir
        mi_robot.abrir_garra_delantera_al_tope(velocidad=1200, limite_potencia=100) # Abre brazos
        mi_robot.sacudir(iteraciones=2, potencia=60, tiempo_ms=100)      

        # Acomodo para buscar los otros
        mi_robot.avanzar_recto(-30) # Retroceso limpio
        mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
        mi_robot.giro_preciso(155)
        mi_robot.avanzar_recto(28)
        mi_robot.mover_motor_izquierdo(120)

        #Entrar
        mi_robot.avanzar_recto(12)
        mi_robot.giro_preciso_pd(90, kp=2.5)
        #Agarrar
        mi_robot.cerrar_garra_delantera_al_tope()

        #Buscar seguidor
        mi_robot.giro_preciso(80)
        mi_robot.avanzar_recto(32) #ANTES ERA 28 
        mi_robot.mover_motor_izquierdo(150)

        mi_robot.seguidor_linea_distancia(sensor, 80, 20)

        mi_robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)
        mi_robot.mover_en_arco(9, distancia_cm=3.8, stop=Stop.COAST)
        mi_robot.mover_en_arco(-9, distancia_cm=3.8, stop=Stop.BRAKE) 
        mi_robot.avanzar_recto(14)
        mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
        mi_robot.mover_eje_central(90)
    # 2. Abrir la garra delantera ligeramente para dar holgura a los bloques
        mi_robot.abrir_garra_delantera(grados=50, velocidad=400) 
        mi_robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
        # 3. ¡Vibrar!
        mi_robot.sacudir(iteraciones=4, potencia=60, tiempo_ms=100)

        # 4. Soltar por completo y salir
        # mi_robot.abrir_garra_delantera_al_tope(velocidad=1200, limite_potencia=100) # Abre brazos

        mi_robot.llevar_eje_central_al_tope(direccion="positivo", limite_potencia=100)
        mi_robot.avanzar_recto(-30) # Retroceso limpio

    elif mosaico == 2: # verde - amarillo
        pass
    elif mosaico == 3: # azul
        pass
    elif mosaico == 4: # amarillo
        pass
    elif mosaico == 5: # blanco
        passa
    else: # si no se detectó se va al 1 por default
        mosaico = 1
        armar_mosaico(mosaico)

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
    # cemento_y_llana()
    # bloques_blancos()
    mosaico = detectar_mosaico()
    agarrar_bloques_amarillos()
    dejar_bloques_amarillos()
    recoger_bloques_azules()
    dejar_bloques_azules_y_pala()
    armar_mosaico(mosaico) 
    
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
    # cemento_y_llana()
    # agarrar_bloques_blancos()
    # dejar_bloques_blancos()
    # mosaico = detectar_mosaico()
    # agarrar_bloques_amarillos()
    # dejar_bloques_amarillos()
    # recoger_bloques_azules()
    # dejar_bloques_azules_y_pala()

    """Cuando se vayan a hacer las pruebas completas, quitar el '= 1' y usar la variable 
    mosaico almacenada después de la función 'detectar_mosaico'"""
    armar_mosaico(mosaico = 1)