from pybricks.parameters import Color, Stop
from robot import Robot # Importamos la clase para que el editor sepa qué es

class ArmadorMosaicos:
    def __init__(self, robot_instancia: Robot, sensor_color):
        self.robot = robot_instancia
        self.sensor_color = sensor_color

    def armar(self, numero_mosaico: int):
        """Método principal que decide qué rutina ejecutar."""
        if numero_mosaico == 1:
            self._armar_verde_verde()
        elif numero_mosaico == 2:
            self._armar_verde_amarillo()
        elif numero_mosaico == 3:
            self._armar_azul()
        elif numero_mosaico == 4:
            self._armar_amarillo()
        elif numero_mosaico == 5:
            self._armar_blanco()
        else:
            # Default
            self._armar_verde_verde()

    # Usamos un guión bajo al inicio para indicar que son métodos "privados" 
    # (solo se usan dentro de esta clase)
    def _armar_verde_verde(self):
        # Mandar la garra central abajo si no lo está
        self.robot.llevar_eje_central_al_tope("negativo")
        self.robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
        # Acomodo para agarrar dos azules y dos verdes
        self.robot.navegacion.seguidor_linea_distancia_desacelerado(self.sensor_color, 100, 52, margen_cm=5, tiempo_acomodo_ms=500)
        self.robot.chasis.giro_preciso(-87)
        # Entrada
        self.robot.abrir_garra_delantera(170, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(13, velocidad=1000, margen_cm=3)
        self.robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.abrir_garra_delantera(170, velocidad=1000, wait_after=True, margen_grados=100)
        self.robot.chasis.avanzar_recto(10, velocidad=1000)
        self.robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(-25, velocidad=1000)
        self.robot.abrir_garra_delantera(100, velocidad=1000, margen_grados=20)
        self.robot.chasis.avanzar_recto(6)
        self.robot.chasis.avanzar_recto(-3.5)
        self.robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)


        self.robot.chasis.girar_sobre_eje(200, margen_grados=20)
        self.robot.chasis.avanzar_recto(5, 1000, frenado=Stop.NONE)
        self.robot.navegacion.seguidor_linea_color(self.sensor_color, 100, Color.BLUE, lado="izquierda", tiempo_acomodo_ms=0, distancia_cm=20)
        # Subida de garra
        self.robot.llevar_eje_central_al_tope("positivo", velocidad=400, limite_potencia=100)
        # Acomodo para que queden en su lugar
        self.robot.chasis.mover_en_arco(-9, distancia_cm=3.6, stop=Stop.COAST)
        self.robot.chasis.mover_en_arco(9.5, distancia_cm=2, stop=Stop.NONE) 
        self.robot.chasis.avanzar_recto(7)
        self.robot.chasis.mover_motor_izquierdo(30)
        # Soltar
        # 1. Bajar la garra hasta la altura de "jaula" (sin aplastar la impresión 3D)
        self.robot.mover_garra_trasera(-110)
        # 2. Abrir la garra delantera ligeramente para dar holgura a los bloques
        self.robot.abrir_garra_delantera(grados=50, velocidad=400) 
        # 3. ¡Vibrar!
        self.robot.chasis.sacudir(iteraciones=3, potencia=60, tiempo_ms=100)
        # 4. Soltar por completo y salir
        self.robot.abrir_garra_delantera_al_tope(velocidad=1200, limite_potencia=100) # Abre brazos
        self.robot.chasis.sacudir(iteraciones=2, potencia=60, tiempo_ms=100)      
        # Acomodo para buscar los otros
        self.robot.chasis.avanzar_recto(-30) # Retroceso limpio
        self.robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
        self.robot.chasis.giro_preciso(155)
        self.robot.chasis.avanzar_recto(28)
        self.robot.chasis.mover_motor_izquierdo(120)
        #Entrar
        self.robot.chasis.avanzar_recto(12)
        # self.robot.navegacion.giro_preciso_pd(90, kp=2.5)
        self.robot.chasis.avanzar_recto(10, velocidad=1000)
        self.robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.abrir_garra_delantera(170, velocidad=1000)
        self.robot.chasis.avanzar_recto(10, velocidad=1000)
        self.robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(-25)
        self.robot.abrir_garra_delantera(100, velocidad=1000, margen_grados=20)
        self.robot.chasis.avanzar_recto(7)
        self.robot.chasis.avanzar_recto(-3.5)
        self.robot.cerrar_garra_delantera_al_tope(velocidad=1000, limite_potencia=100)
        #Agarrar
        #Buscar seguidor
        self.robot.chasis.giro_preciso(80)
        self.robot.chasis.avanzar_recto(32) #ANTES ERA 28 
        self.robot.chasis.mover_motor_izquierdo(150)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor_color, 80, 20)
        self.robot.llevar_eje_central_al_tope("positivo", limite_potencia=100)
        self.robot.chasis.mover_en_arco(9, distancia_cm=3.8, stop=Stop.COAST)
        self.robot.chasis.mover_en_arco(-9, distancia_cm=3.8, stop=Stop.BRAKE) 
        self.robot.chasis.avanzar_recto(14)
        self.robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
        self.robot.mover_eje_central(90)
        # 2 Abrir la garra delantera ligeramente para dar holgura a los bloques
        self.robot.abrir_garra_delantera(grados=50, velocidad=400) 
        self.robot.llevar_eje_central_al_tope("negativo", limite_potencia=100)
        # 3. ¡Vibrar!
        self.robot.chasis.sacudir(iteraciones=4, potencia=60, tiempo_ms=100)
        # 4. Soltar por completo y salir
        self.robot.abrir_garra_delantera_al_tope(velocidad=1200, limite_potencia=100) # Abre brazos
        self.robot.llevar_eje_central_al_tope(direccion="positivo", limite_potencia=100)
        self.robot.chasis.avanzar_recto(-30) # Retroceso limpio

        """
        Acá se pretende agarrar los bloques azul y verde y dejarlos en la fila de atrás de la garra.
        """
        self.robot.chasis.giro_preciso(180)
        self.robot.chasis.avanzar_recto(40)
        self.robot.abrir_garra_delantera_al_tope(170, limite_potencia=100)
        self.robot.chasis.avanzar_recto(20)
        self.robot.cerrar_garra_delantera_al_tope()
        self.robot.chasis.avanzar_recto(-10)
        self.robot.abrir_garra_delantera(170, 1000)
        self.robot.chasis.avanzar_recto(10)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.cerrar_garra_delantera_al_tope()
        
        """"
        Acá retroceder y girar para agarrar los otros dos verdes
        """
        self.robot.chasis.avanzar_recto(-10)
        self.robot.chasis.giro_preciso(90)
        self.robot.chasis.avanzar_recto(-15)
        self.robot.chasis.mover_motor_derecho(180, 1000)
        self.robot.chasis.avanzar_recto(10)
        self.robot.chasis.mover_motor_izquierdo(180, 1000)
        self.robot.abrir_garra_delantera(170, 1000)
        self.robot.chasis.avanzar_recto(10)
        self.robot.cerrar_garra_delantera_al_tope(1000, 100)

        """Acá agarra los 2 bloques restantes con la garra de atras"""
        self.robot.chasis.avanzar_recto(-30)
        self.robot.chasis.giro_preciso(180)
        self.robot.llevar_eje_central_al_tope(direccion="positivo", limite_potencia=100)
        self.robot.chasis.avanzar_recto(-30)
        self.robot.llevar_eje_central_al_tope(direccion="negativo", limite_potencia=100)

        """Acá irlo a dejar al mosaico desde el espacio de los verdes"""
        self.robot.chasis.avanzar_recto(30)
        self.robot.chasis.giro_preciso(90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor_color, 1000, 200)
        self.robot.chasis.giro_preciso(-90)
        self.robot.chasis.avanzar_recto(30)
        self.robot.navegacion.seguidor_linea_color(self.sensor_color, 1000, Color.GREEN, distancia_cm=40)

        """Acá el robot hp se acomoda para quedar recto con los colores"""
        self.robot.llevar_eje_central_al_tope("positivo", velocidad=400, limite_potencia=100)
        self.robot.chasis.mover_motor_derecho(180, 1000)
        self.robot.chasis.avanzar_recto(3)
        self.robot.chasis.mover_motor_izquierdo(180, 1000)

        """Acá los deja"""
        # 1. Bajar la garra hasta la altura de "jaula" (sin aplastar la impresión 3D)
        self.robot.mover_garra_trasera(-110)
        # 2. Abrir la garra delantera ligeramente para dar holgura a los bloques
        self.robot.abrir_garra_delantera(grados=50, velocidad=400) 
        # 3. ¡Vibrar!
        self.robot.chasis.sacudir(iteraciones=3, potencia=60, tiempo_ms=100)
        # 4. Soltar por completo y salir
        self.robot.abrir_garra_delantera_al_tope(velocidad=1200, limite_potencia=100) # Abre brazos
        self.robot.chasis.sacudir(iteraciones=2, potencia=60, tiempo_ms=100)   

    def _armar_verde_amarillo(self):
        pass

    def _armar_azul(self):
        pass

    def _armar_amarillo(self):
        pass

    def _armar_blanco(self):
        pass