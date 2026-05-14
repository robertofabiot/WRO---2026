from pybricks.parameters import Stop
from pybricks.tools import wait
import config

class Chasis:
    def __init__(self, drive_base, motor_izq, motor_der, hub, velocidad_base):
        self.drive_base = drive_base
        self.motor_izquierda = motor_izq
        self.motor_derecha = motor_der
        self.hub = hub
        self.velocidad_base = velocidad_base

    def avanzar_recto(self, distancia_cm, velocidad=None, frenado=Stop.BRAKE, wait_after=True, margen_cm=0):
        if velocidad is None:
            velocidad = self.velocidad_base
        velocidad = max(min(velocidad, 976), -976)
        distancia_mm = distancia_cm * 10
        
        if wait_after and margen_cm > 0:
            distancia_inicial = self.drive_base.distance()
            margen_mm = abs(margen_cm * 10)
            self.drive_base.straight(distancia_mm, then=frenado, wait=False)
            
            while abs(self.drive_base.distance() - distancia_inicial) < (abs(distancia_mm) - margen_mm):
                if self.drive_base.stalled():
                    break
                wait(2)
        else:
            self.drive_base.straight(distancia_mm, then=frenado, wait=wait_after)
    
    def avance_milimetrico(self, milimetros, velocidad=400):
        """
        Da un micro-paso preciso. Incluye relajación mecánica previa para 
        evitar saltos (overshoot) cuando se usa después de chocar contra paredes.
        """
        # 1. IMPORTANTE: Relajamos los motores (Coast) para liberar la tensión 
        # del choque previo en los engranajes y desatorar los PIDs.
        self.motor_izquierda.stop()
        self.motor_derecha.stop()
        wait(100) # Damos 100ms para que el plástico regrese a su forma natural
        
        # 2. Calculamos los grados exactos (Usando tu variable de config)
        circunferencia = config.DIAMETRO_RUEDA * 3.1416
        grados_a_mover = (milimetros / circunferencia) * 360
        
        # 3. Movemos ambos motores simultáneamente
        self.motor_izquierda.run_angle(velocidad, grados_a_mover, then=Stop.HOLD, wait=False)
        self.motor_derecha.run_angle(velocidad, grados_a_mover, then=Stop.HOLD, wait=True)
        
        wait(50)
    
    def chocar_inteligente(self, distancia_acercamiento_cm, velocidad_acercamiento=800, potencia_choque=35, timeout_choque_ms=1500):
        """
        Avanza rápido la mayor parte del trayecto y luego reduce la potencia 
        drásticamente para asegurar un choque mecánico sin derrape de llantas.
        
        Si distancia_acercamiento_cm es negativa, el acercamiento y el choque se hacen en reversa.
        """
        # 1. Ajustar los signos automáticamente
        es_reversa = distancia_acercamiento_cm < 0
        potencia_real = -abs(potencia_choque) if es_reversa else abs(potencia_choque)
        
        # 2. Fase de "Vuelo": Acercamiento a alta velocidad usando tu PID normal
        self.avanzar_recto(distancia_acercamiento_cm, velocidad=velocidad_acercamiento, wait_after=True)
        
        # 3. Fase de "Toque": Potencia baja para evitar derrape. 
        # Umbral bajo (20) porque al ir lento, el ruido de velocidad es menor.
        self.avanzar_hasta_choque(
            potencia=potencia_real, 
            umbral_velocidad=20, 
            tiempo_arranque_ms=150, 
            timeout_ms=timeout_choque_ms
        )
            
    def mover_en_arco(self, radio_cm, angulo=None, distancia_cm=None, stop=Stop.HOLD, wait_after=True, margen_grados=0, margen_cm=0):
        radio_mm = radio_cm * 10
        distancia_mm = distancia_cm * 10 if distancia_cm is not None else None

        if wait_after and (margen_grados > 0 or margen_cm > 0):
            self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=False)
            
            if distancia_cm is not None and margen_cm > 0:
                dist_inicial = self.drive_base.distance()
                margen_mm_real = abs(margen_cm * 10)
                meta_mm = abs(distancia_mm)
                while abs(self.drive_base.distance() - dist_inicial) < (meta_mm - margen_mm_real):
                    if self.drive_base.stalled(): break
                    wait(2)
            elif angulo is not None and margen_grados > 0:
                ang_inicial = self.drive_base.angle()
                meta_ang = abs(angulo)
                while abs(self.drive_base.angle() - ang_inicial) < (meta_ang - margen_grados):
                    if self.drive_base.stalled(): break
                    wait(2)
        else:
            self.drive_base.arc(radio_mm, angle=angulo, distance=distancia_mm, then=stop, wait=wait_after)

    def girar_sobre_eje(self, grados, wait_after=True, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_inicial = self.drive_base.angle()
            self.drive_base.turn(grados, wait=False)
            while abs(self.drive_base.angle() - angulo_inicial) < (abs(grados) - margen_grados):
                if self.drive_base.stalled(): break
                wait(2)
        else:
            self.drive_base.turn(grados, wait=wait_after)

    def giro_preciso(self, angulo_objetivo, kp_nuevo=2.5, tolerancia=1, margen_grados=0):
        self.hub.imu.reset_heading(0) 
        kp = kp_nuevo
        min_speed = 50 
        while True:
            angulo_actual = self.hub.imu.heading()
            error = angulo_objetivo - angulo_actual
            if abs(error) <= max(tolerancia, margen_grados):
                break
            turn_rate = error * kp
            if turn_rate > 0:
                turn_rate = max(turn_rate, min_speed)
            else:
                turn_rate = min(turn_rate, -min_speed)
            self.drive_base.drive(0, turn_rate)
            wait(10)
        self.drive_base.stop()

    def mover_motor_izquierdo(self, grados, velocidad=500, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_izquierda.angle() + grados
            self.motor_izquierda.run_angle(velocidad, grados, then=frenado, wait=False) # <- Añadido then=frenado
            while abs(angulo_meta - self.motor_izquierda.angle()) > margen_grados:
                if self.motor_izquierda.stalled(): break
                wait(2)
        else:
            self.motor_izquierda.run_angle(velocidad, grados, then=frenado, wait=wait_after) # <- Añadido then=frenado

    def mover_motor_derecho(self, grados, velocidad=800, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor_derecha.angle() + grados
            self.motor_derecha.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor_derecha.angle()) > margen_grados:
                if self.motor_derecha.stalled(): break
                wait(2)
        else:
            self.motor_derecha.run_angle(velocidad, grados, then=frenado, wait=wait_after)

    def avanzar_hasta_choque(self, potencia=100, umbral_velocidad=50, tiempo_arranque_ms=300, timeout_ms=None):
        """
        Avanza aplicando voltaje directo (.dc) hasta detectar un choque mecánico.
        Puedes usar potencia negativa (-100) para chocar de reversa.
        
        :param timeout_ms: Tiempo máximo en milisegundos antes de abortar. Si es None, no hay límite.
        """
        self.drive_base.stop() # Desactivamos temporalmente el control de DriveBase
        
        # Inyectar la potencia cruda a los motores
        self.motor_izquierda.dc(potencia)
        self.motor_derecha.dc(potencia)
        
        # Pequeña pausa para que el robot venza la inercia inicial y no detecte 
        # un falso choque al estar arrancando desde velocidad 0.
        wait(tiempo_arranque_ms)
        
        # Llevamos el registro del tiempo total (iniciando con el tiempo de arranque ya consumido)
        tiempo_transcurrido = tiempo_arranque_ms
        paso_ms = 10 # Los milisegundos que esperamos en cada ciclo
        
        while True:
            vel_izq = abs(self.motor_izquierda.speed())
            vel_der = abs(self.motor_derecha.speed())
            
            # Si la velocidad cae por debajo del umbral a pesar de tener máxima potencia,
            # significa que físicamente chocó contra un obstáculo.
            if vel_izq < umbral_velocidad and vel_der < umbral_velocidad:
                break
            
            # Si definimos un timeout y el tiempo transcurrido lo supera, salimos del bucle
            if timeout_ms is not None and tiempo_transcurrido >= timeout_ms:
                print("Advertencia: Timeout alcanzado en avanzar_hasta_choque")
                break
                
            wait(paso_ms)
            tiempo_transcurrido += paso_ms
            
        # Freno mecánico firme para no rebotar
        self.motor_izquierda.hold()
        self.motor_derecha.hold()
    
    def latigazo(self, grados=45, velocidad_giro=1000, aceleracion=3000):
        """
        Da un giro rápido y violento para aventar un bloque, y regresa a la 
        posición original compensando el deslizamiento de las llantas usando el IMU.
        """
        # 1. Guardamos tu configuración normal de velocidad
        settings_originales = self.drive_base.settings()
        
        # 2. Tomamos una "fotografía" de tu ángulo actual exacto
        angulo_inicial = self.hub.imu.heading()
        
        # 3. Inyectamos esteroides temporales al DriveBase
        self.drive_base.settings(
            turn_rate=velocidad_giro,
            turn_acceleration=aceleracion
        )
        
        # 4. ¡Latigazo! (Ida)
        self.drive_base.turn(grados)
        
        # 5. Medimos dónde quedamos realmente tras el derrape
        angulo_post_golpe = self.hub.imu.heading()
        
        # 6. Calculamos la compensación y regresamos
        grados_de_regreso = angulo_inicial - angulo_post_golpe
        self.drive_base.turn(grados_de_regreso)
        
        # 7. Restauramos la configuración original del chasis para no arruinar tu navegación
        self.drive_base.settings(
            straight_speed=settings_originales[0],
            straight_acceleration=settings_originales[1],
            turn_rate=settings_originales[2],
            turn_acceleration=settings_originales[3]
        )
            
    def sacudir(self, iteraciones=5, potencia=100, tiempo_ms=60):
        self.drive_base.stop() 
        for _ in range(iteraciones):
            self.motor_izquierda.dc(potencia)
            self.motor_derecha.dc(-potencia)
            wait(tiempo_ms)
            self.motor_izquierda.dc(-potencia)
            self.motor_derecha.dc(potencia)
            wait(tiempo_ms)
        self.motor_izquierda.brake()
        self.motor_derecha.brake()
        wait(100)

    def compensar_voltaje(self, potencia_deseada):
        voltaje_actual = self.hub.battery.voltage()
        if voltaje_actual == 0: return potencia_deseada
        potencia_compensada = potencia_deseada * (8000 / voltaje_actual)
        return max(-100, min(100, potencia_compensada))