from pybricks.parameters import Color, Stop
from pybricks.pupdevices import ColorSensor
from robot import Robot # Para el Type Hinting

class ArmadorMosaicos:
    def __init__(self, robot_instancia: Robot, sensor_color: ColorSensor):
        self.robot = robot_instancia
        self.sensor_color = sensor_color
        
        self.rutinas = {
            1: self._armar_verde_verde,
            2: self._armar_verde_amarillo,
            3: self._armar_azul,
            4: self._armar_amarillo,
            5: self._armar_blanco
        }

    def armar(self, numero_mosaico: int):
        rutina_a_ejecutar = self.rutinas.get(numero_mosaico, self._armar_verde_verde)
        print(f"Ejecutando rutina de armado para mosaico: {numero_mosaico}")
        rutina_a_ejecutar()

    def _armar_verde_verde(self):
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo")
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)
        
        self.robot.navegacion.seguidor_linea_distancia_desacelerado(self.sensor_color, 100, 52, margen_cm=5, tiempo_acomodo_ms=500)
        self.robot.chasis.giro_preciso(-87)
        
        self.robot.mecanismos.garra_delantera.abrir(170, velocidad=1000, wait_after=False)
        self.robot.chasis.avanzar_recto(13, velocidad=1000, margen_cm=3)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.mecanismos.garra_delantera.abrir(170, velocidad=1000, wait_after=True, margen_grados=10)
        self.robot.chasis.avanzar_recto(10, velocidad=1000)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(-25, velocidad=1000)
        self.robot.mecanismos.garra_delantera.abrir(100, velocidad=1000, margen_grados=20)
        self.robot.chasis.avanzar_recto(6)
        self.robot.chasis.avanzar_recto(-3.5)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.girar_sobre_eje(200, margen_grados=20)
        self.robot.chasis.avanzar_recto(5, 1000, frenado=Stop.NONE)
        self.robot.navegacion.seguidor_linea_color(self.sensor_color, 100, Color.BLUE, lado="izquierda", tiempo_acomodo_ms=0, distancia_cm=20)
        
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", velocidad=400, limite_potencia=100)
        
        self.robot.chasis.mover_en_arco(-9, distancia_cm=3.6, stop=Stop.COAST)
        self.robot.chasis.mover_en_arco(9.5, distancia_cm=2, stop=Stop.NONE)
        self.robot.chasis.avanzar_recto(7)
        self.robot.chasis.mover_motor_izquierdo(30)
        
        self.robot.mecanismos.garra_trasera.mover(-110)
        self.robot.mecanismos.garra_delantera.abrir(grados=50, velocidad=400)
        self.robot.chasis.sacudir(iteraciones=3, potencia=60, tiempo_ms=100)
        self.robot.mecanismos.garra_delantera.abrir_al_tope(velocidad=1200, limite_potencia=100)
        self.robot.chasis.sacudir(iteraciones=2, potencia=60, tiempo_ms=100)
        
        self.robot.chasis.avanzar_recto(-30) 
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
        self.robot.chasis.giro_preciso(155)
        self.robot.chasis.avanzar_recto(28)
        self.robot.chasis.mover_motor_izquierdo(120)
        
        self.robot.chasis.avanzar_recto(12)
        self.robot.chasis.avanzar_recto(10, velocidad=1000)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.mecanismos.garra_delantera.abrir(170, velocidad=1000)
        self.robot.chasis.avanzar_recto(10, velocidad=1000)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(-25)
        self.robot.mecanismos.garra_delantera.abrir(100, velocidad=1000, margen_grados=20)
        self.robot.chasis.avanzar_recto(7)
        self.robot.chasis.avanzar_recto(-3.5)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(velocidad=1000, limite_potencia=100)
        
        self.robot.chasis.giro_preciso(80)
        self.robot.chasis.avanzar_recto(32)
        self.robot.chasis.mover_motor_izquierdo(150)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor_color, 80, 20)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", limite_potencia=100)
        self.robot.chasis.mover_en_arco(9, distancia_cm=3.8, stop=Stop.COAST)
        self.robot.chasis.mover_en_arco(-9, distancia_cm=3.8, stop=Stop.BRAKE)
        self.robot.chasis.avanzar_recto(14)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
        self.robot.mecanismos.elevador_delantero.mover(90)
        self.robot.mecanismos.garra_delantera.abrir(grados=50, velocidad=400)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("negativo", limite_potencia=100)
        self.robot.chasis.sacudir(iteraciones=4, potencia=60, tiempo_ms=100)
        self.robot.mecanismos.garra_delantera.abrir_al_tope(velocidad=1200, limite_potencia=100)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope(direccion="positivo", limite_potencia=100)
        
        self.robot.chasis.avanzar_recto(-30)
        
        self.robot.chasis.giro_preciso(180)
        self.robot.chasis.avanzar_recto(40)
        self.robot.mecanismos.garra_delantera.abrir_al_tope(velocidad=1000, limite_potencia=100)
        self.robot.chasis.avanzar_recto(20)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope()
        self.robot.chasis.avanzar_recto(-10)
        self.robot.mecanismos.garra_delantera.abrir(170, 1000)
        self.robot.chasis.avanzar_recto(10)
        self.robot.chasis.avanzar_recto(-5)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope()
        
        self.robot.chasis.avanzar_recto(-10)
        self.robot.chasis.giro_preciso(90)
        self.robot.chasis.avanzar_recto(-15)
        self.robot.chasis.mover_motor_derecho(180, 1000)
        self.robot.chasis.avanzar_recto(10)
        self.robot.chasis.mover_motor_izquierdo(180, 1000)
        self.robot.mecanismos.garra_delantera.abrir(170, 1000)
        self.robot.chasis.avanzar_recto(10)
        self.robot.mecanismos.garra_delantera.cerrar_al_tope(1000, 100)
        
        self.robot.chasis.avanzar_recto(-30)
        self.robot.chasis.giro_preciso(180)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope(direccion="positivo", limite_potencia=100)
        self.robot.chasis.avanzar_recto(-30)
        self.robot.mecanismos.elevador_delantero.llevar_al_tope(direccion="negativo", limite_potencia=100)
        
        self.robot.chasis.avanzar_recto(30)
        self.robot.chasis.giro_preciso(90)
        self.robot.navegacion.seguidor_linea_distancia(self.sensor_color, 1000, 200)
        self.robot.chasis.giro_preciso(-90)
        self.robot.chasis.avanzar_recto(30)
        self.robot.navegacion.seguidor_linea_color(self.sensor_color, 1000, Color.GREEN, distancia_cm=40)
        
        self.robot.mecanismos.elevador_delantero.llevar_al_tope("positivo", velocidad=400, limite_potencia=100)
        self.robot.chasis.mover_motor_derecho(180, 1000)
        self.robot.chasis.avanzar_recto(3)
        self.robot.chasis.mover_motor_izquierdo(180, 1000)
        
        self.robot.mecanismos.garra_trasera.mover(-110)
        self.robot.mecanismos.garra_delantera.abrir(grados=50, velocidad=400)
        self.robot.chasis.sacudir(iteraciones=3, potencia=60, tiempo_ms=100)
        self.robot.mecanismos.garra_delantera.abrir_al_tope(velocidad=1200, limite_potencia=100)
        self.robot.chasis.sacudir(iteraciones=2, potencia=60, tiempo_ms=100)

    def _armar_verde_amarillo(self): pass
    def _armar_azul(self): pass
    def _armar_amarillo(self): pass
    def _armar_blanco(self): pass