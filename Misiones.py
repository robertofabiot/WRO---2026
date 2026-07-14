from pybricks.parameters import Stop, Color
from pybricks.tools import wait
import config
from robot import Robot
import Utils

class Misiones:
    def __init__(self, robot : Robot, sensor_frente):
        self.robot = robot
        self.sensor = sensor_frente

    def _identificar_combinacion(self, sensor, distancia_si_verde):
        color_principal = sensor.color()
        if color_principal not in config.MOSAICOS: 
            return -1  
            
        decision = config.MOSAICOS[color_principal]
        
        # Si la decisión es un diccionario, significa que leyó Verde y necesita desempatar
        if type(decision) is dict:
            # 1. Avanza para buscar el color secundario
            self.robot.chasis.avanzar_recto(distancia_si_verde)
            wait(50) # Micro-pausa para que el sensor lea sin vibraciones del motor
            color_anterior = sensor.color()
            
            # 2. Restaura la posición exacta retrocediendo la misma distancia
            self.robot.chasis.avanzar_recto(-distancia_si_verde)
            
            # 3. Evalúa la lectura secundaria
            if color_anterior not in decision: 
                return -1
            return decision[color_anterior]
            
        # Si no era verde, devuelve el color directamente y se queda en su lugar original
        return decision
    
    def agarrar_bloques_blancos(self):
        self.robot.garra_trasera.ir_a_porcentaje(90, velocidad=1000)
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=300, potencia=80, angulo_referencia=90)
        self.robot.navegacion.giro_absoluto_motor_izquierdo(180, max_speed=1000, min_speed=800, kp=2.5, kd=28, encadenado=True)
        self.robot.garra_trasera.subir(185, velocidad=1000, wait_after=False)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 157, tiempo_acomodo_ms=0, encadenado=True, margen_cm=7)
        self.robot.chasis.mover_motor_izquierdo(300, velocidad=1000, margen_grados=50, encadenado=True)
        self.robot.chasis.avanzar_recto(12, 1000, margen_cm=7, encadenado=True)
        self.robot.navegacion.giro_absoluto_pd(2, max_speed=200, min_speed=100, kp=3.5, kd=22) # Cierra el combo fluido
        self.__recoger_bloques(25, 100, bajar=160)

    def detectar_mosaico(self):
        self.robot.chasis.avanzar_recto(7, velocidad=1000, margen_cm=7, encadenado=True)
        
        # ACELERADO: Subimos de 600 a 1000. Al estar encadenado, no hay latigazo mecánico.
        self.robot.navegacion.giro_absoluto_motor_izquierdo(55, max_speed=600, min_speed=500, kp=7, kd=24)
        
        self.robot.chasis.avanzar_recto(55, velocidad=1000, margen_cm=7, encadenado=True)        
        self.robot.chasis.mover_motor_derecho(370, velocidad=1000, encadenado=True)
        
        # OPTIMIZACIÓN DE TIEMPO: Subimos velocidad a 100 (ya que le pasas distancia_cm=10, no se capea a 70).
        # CLAVE: tiempo_acomodo_ms=0 para anular la espera de 800ms y salir disparado.
        self.robot.navegacion.seguidor_linea_color(
            self.sensor, 
            velocidad_max=100, 
            color_objetivo=Color.GREEN, 
            lado="derecha", 
            distancia_cm=10, 
            tiempo_acomodo_ms=0, 
            encadenado=True
        )
        
        self.robot.chasis.mover_motor_derecho(90, velocidad=1000)
        
        # AJUSTE PD EXTREMO: Giro rapidísimo. 
        # min_speed alto (150) amortiguado por un kd muy agresivo (26.0).
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=150, kp=3.0, kd=26.0)
        
        self.robot.chasis.avanzar_recto(14, 1000)
        mosaico = self._identificar_combinacion(self.sensor, 5)
        print(f"Mosaico detectado: {mosaico}" if mosaico != -1 else "Error en escaneo")
        
        return mosaico

    def dejar_bloques_blancos(self):
        self.robot.chasis.avanzar_recto(-17, encadenado=False)
        self.robot.navegacion.giro_absoluto_pd(228, max_speed=400, ruta_corta=False, encadenado=False)
        self.robot.chasis.avanzar_recto(-16, velocidad=1000, margen_cm=7)
        self.robot.garra_trasera.ir_a_porcentaje(0, velocidad=1000, wait_after=False)

    def agarrar_bloques_verdes(self):
        # OPTIMIZADO: Agregamos encadenado=True y un margen para fusionar la recta con el giro del motor
        self.robot.chasis.avanzar_recto(9, velocidad=500, margen_cm=2, encadenado=True)
        self.robot.chasis.mover_motor_derecho(300, velocidad=500, encadenado=True)
        
        # ACELERADO: tiempo_acomodo_ms=0 porque el robot ya trae inercia rotacional del movimiento anterior
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 40, tiempo_acomodo_ms=0, encadenado=False, margen_cm=7)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 70, 10, tiempo_acomodo_ms=0, encadenado=False, margen_cm=7)
        
        # # ACELERADO: Subimos la velocidad de 600 a 1000. El margen_grados absorbe el latigazo.
        # self.robot.chasis.mover_motor_derecho(110, velocidad=1000, margen_grados=50, encadenado=True)
        
        # # FLUIDEZ: Cambiamos encadenado=False a True. Este avance de 2cm ahora es una transición rápida, no una pausa.
        # self.robot.chasis.avanzar_recto(2, velocidad=1000, encadenado=True)
        
        # AJUSTE PD EXTREMO: Giro de cuadratura violento pero estable.
        # max_speed=800, min_speed=120 para mucha fuerza, kd=26.0 para frenar en seco y kp=3.0 para evitar correcciones nerviosas.
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=120, kp=3.0, kd=26.0)
        
        self.__recoger_bloques(14, wait_ms=300)
    
    def dejar_bloques_verdes(self):
        self.robot.chasis.cuadrar_contra_pared(tiempo_ms=400, potencia=70)
        
        # La garra baja en segundo plano mientras el chasis empieza la secuencia de escape
        self.robot.garra_trasera.ir_a_porcentaje(95, velocidad=1000, wait_after=False)
        
        # AJUSTE PD EXTREMO 1 (Despegue de pared): 
        # min_speed alto (150) para salir rápido, kp=3.0 y kd=26.0 para frenar exacto en 0 grados.
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=150, kp=3.0, kd=26.0, encadenado=True)             
        
        # ACELERADO: Eliminamos el tiempo_acomodo_ms pasándolo a 0. Entrará a velocidad tope al instante.
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 10, lado="izquierda", tiempo_acomodo_ms=0, encadenado=True)
        
        # AJUSTE PD EXTREMO 2 (Vuelta en U): 
        # Mucha energía inercial. Necesitamos un kd súper agresivo (28.0 o 30.0) para frenar esa inercia de 180 grados.
        self.robot.navegacion.giro_absoluto_pd(180, max_speed=800, min_speed=150, kp=3.0, kd=28.0, encadenado=True)             
        
        # SPRINT: Máxima velocidad en reversa. El margen de 7cm absorberá la transición final.
        self.robot.chasis.avanzar_recto(-55, velocidad=1000, encadenado=True, margen_cm=7)
        
        # Tope final por detección de corriente (asegura que guarda la garra por completo)
        self.robot.garra_trasera.subir_al_tope(1000, limite_potencia=100)
    
    def agarrar_bloques_amarillos(self):
        # OPTIMIZADO: Velocidad en 100 (límite físico DC). 
        # Ponemos tiempo_acomodo_ms=150 porque arranca desde cero (misión anterior terminó bloqueante).
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 28, tiempo_acomodo_ms=150, encadenado=True)
        
        # ACELERADO: Subimos a velocidad máxima (1000). El margen absorbe la transición.
        self.robot.chasis.mover_motor_derecho(500, velocidad=600, margen_grados=50, encadenado=True)
        
        # FLUIDEZ: Se le agrega encadenado=True para eliminar la necesidad de frenar manualmente.
        # Subimos levemente la velocidad_escaneo a 200 para no perder mucho tiempo buscando la línea negra.
        self.robot.navegacion.avanzar_tiempo_luego_color(
            self.sensor, 
            tiempo_ciego_s=0.3, 
            color_objetivo=Color.BLACK, 
            distancia_extra_cm=4, 
            velocidad_alta=1000, 
            velocidad_escaneo=200, 
            encadenado=True
        )
        
        # ELIMINADO: self.robot.drive_base.brake() -> Ya no se necesita gracias al encadenamiento.
        
        # AJUSTE PD EXTREMO: El cierre del combo asume toda la inercia anterior.
        # max_speed 800, min_speed 120 para encuadre violento, con kd de 26.0 para frenado en seco.
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=120, kp=3.0, kd=26.0) 
        
        self.__recoger_bloques(30, 200)

    def dejar_bloques_amarillos(self):
        # FLUIDEZ: Encadenamos el giro inicial para entrar con aceleración a la recta
        self.robot.chasis.girar_sobre_eje(-60, encadenado=True)
        
        # SPRINT ENCADENADO: El robot corre los 77cm a máxima velocidad.
        # CLAVE: Al poner margen_cm=5, EVITAMOS la frenada a cero y usamos el impulso 
        # sobrante para alimentar el giro del motor izquierdo que te molestaba.
        self.robot.chasis.avanzar_recto(83, 1000, margen_cm=5, encadenado=True)
        
        # SOLUCIÓN: Giro monomotor ultra veloz. 
        # Subimos max_speed a 1000 y min_speed a 400 para eliminar cualquier retraso.
        # kp=2.5 y kd=28.0 absorben el impacto para clavar el rumbo exacto en 0°.
        # Activamos encadenado=True para enlazar directamente con el lazo del seguidor.
        self.robot.navegacion.giro_absoluto_motor_izquierdo(
            0, 
            max_speed=1000, 
            min_speed=400, 
            kp=2.5, 
            kd=20.0, 
            encadenado=True
        )
        
        # OPTIMIZADO: Como el paso anterior ya viene encadenado y con rumbo corregido,
        # eliminamos la espera pasando tiempo_acomodo_ms a 0. Tracción DC activa al 100% de inmediato.
        # Lo dejamos en encadenado=True para conectar con el giro final de escape.
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(
            self.sensor, 
            velocidad_max=100, 
            cruces_objetivo=1, 
            distancia_extra_cm=15, 
            distancia_inicial_cm=12,
            lado="izquierda", 
            tiempo_acomodo_ms=0,
            encadenado=True
        )
        
        # AJUSTE PD SINGLE-MOTOR 2: Tenías min_speed=1000 (lo que desactivaba el decaimiento de Pybricks de los últimos 8°).
        # Lo regulamos de forma segura: max_speed=1000, min_speed=500 y un kd de 32.0 para evitar que el chasis chicotee al llegar a 270°.
        self.robot.navegacion.giro_absoluto_motor_izquierdo(
            270, 
            max_speed=1000, 
            min_speed=500, 
            kp=2.5, 
            kd=25.0, 
            encadenado=False
        )
        
        # Escape final de la zona de descarga mientras la jaula trasera sube de forma asíncrona
        self.robot.garra_trasera.ir_a_porcentaje(30, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-8, velocidad=1000, encadenado=False)

    def cemento_y_llana(self):
        # FLUIDEZ: Rompemos la primera parada total agregando margen y encadenado
        self.robot.chasis.avanzar_recto(10, velocidad=1000, margen_cm=2, encadenado=True)
        
        # AJUSTE PD 1 (Despliegue de garra): max_speed=800 y un min_speed rápido pero controlable (150).
        self.robot.navegacion.giro_absoluto_pd(90, max_speed=800, min_speed=150, kp=3.0, kd=26.0, encadenado=True)
        self.robot.garra_trasera.ir_a_porcentaje(90, velocidad=500, wait_after=False)
        
        self.robot.chasis.avanzar_recto(-7.5, velocidad=1000, margen_cm=2, encadenado=True)
        self.robot.chasis.mover_motor_derecho(-100, velocidad=1000, encadenado=True)
        
        # AJUSTE PD EXTREMO (El culpable de la pared): 
        # Conservamos tu min_speed=600 para que sea violento, pero bajamos kp a 2.0 (para no acelerar más) 
        # e inyectamos un kd masivo de 35.0. Esto absorberá el impacto en seco, evitando que derrape hacia el muro.
        self.robot.navegacion.giro_absoluto_pd(172, max_speed=800, min_speed=600, kp=2.0, kd=35.0, encadenado=True)
        
        # FLUIDEZ: Eliminamos la detención en reversa. Usará el impulso sobrante para entrar al siguiente giro.
        self.robot.chasis.avanzar_recto(-37, velocidad=1000, margen_cm=5, encadenado=True)
        
        # AJUSTE PD 3: min_speed=400 requiere un freno agresivo. kd=32.0 asegura la estabilidad antes del escaneo.
        self.robot.navegacion.giro_absoluto_pd(160, max_speed=800, min_speed=400, kp=2.5, kd=32.0, encadenado=True)
        
        self.robot.navegacion.avanzar_tiempo_luego_color(self.sensor, 0.5, Color.BLACK, encadenado=True)
        
        # OPTIMIZADO: tiempo_acomodo_ms pasa a 0 porque ya trae inercia, exigiendo tracción máxima de inmediato.
        self.robot.navegacion.seguidor_linea_cruces_y_distancia(self.sensor, 100, 2, 22, distancia_inicial_cm=7, tiempo_acomodo_ms=0, encadenado=True)
        
        self.robot.chasis.mover_motor_derecho(440, velocidad=1000, margen_grados=50, encadenado=True)
        self.robot.garra_trasera.subir(100, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(-13, velocidad=1000, encadenado=True)
        
        # AJUSTE PD MONOMOTOR: min_speed=300 requiere control. 
        self.robot.navegacion.giro_absoluto_motor_derecho(175, max_speed=1000, min_speed=300, kp=2.5, kd=28.0, encadenado=True)
        
        # PAUSA ESTRATÉGICA: Esta línea NO DEBE tener wait_after=False.
        # Al dejar el bloqueo por defecto de la clase Garra, nos aseguramos que el robot
        # no se mueva hasta que la pala esté firmemente atrapada.
        self.robot.garra_trasera.bajar(100, velocidad=1000)
        
        self.robot.chasis.mover_motor_derecho(400, velocidad=1000, encadenado=True)
        self.robot.chasis.avanzar_y_accionar_en_recorrido(
            20, 
            15,  
            lambda: self.robot.garra_trasera.subir(185, velocidad=1000, wait_after=False),
            encadenado=True)

    def agarrar_bloques_azules(self):
        # SPRINT INICIAL: Subimos de 400 a 1000. 
        # Al estar encadenado, pivota rapidísimo y fluye hacia la línea sin frenar.
        self.robot.chasis.mover_motor_izquierdo(630, velocidad=1000, encadenado=True)
        
        # ACELERADO: Eliminamos el tiempo_acomodo_ms (a 0) porque el chasis ya viene con impulso.
        # Entra directo a tracción 100.
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 10, lado="izquierda", tiempo_acomodo_ms=0, encadenado=True)
        
        # AJUSTE PD EXTREMO: Giro agresivo para cuadrarse perfecto antes de la recolección.
        # max_speed=800 y min_speed=150 para que no dude, con un kd=26.0 para frenarlo en 0°.
        self.robot.navegacion.giro_absoluto_pd(0, max_speed=800, min_speed=150, kp=3.0, kd=26.0, encadenado=True)
        
        self.__recoger_bloques(27, 100, bajar=185)

    def dejar_bloques_azules_y_pala(self):
        self.robot.navegacion.curva_coordenada_local(40, -15)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor, 100, 130, lado="izquierda", encadenado=True)

    def __recoger_bloques(self, distancia, wait_ms=200, bajar=165):
        self.robot.chasis.avanzar_recto(-distancia, 1000, wait_after=False)
        wait(wait_ms)
        self.robot.garra_trasera.ir_a_porcentaje(97, velocidad=600)