"""
Medidor de limites reales del chasis. CORRELO SOBRE EL TAPETE DE COMPETENCIA.

La aceleracion maxima no es una propiedad del robot sino de la superficie:
depende de cuando la rueda patina contra ESE tapete. Medirlo en una mesa o en
el piso da un numero equivocado, y si queda alto el patinaje corrompe la
odometria, que es de lo que dependen todas las distancias del recorrido.

Cabe en 90 cm: el patinaje ocurre en el transitorio de arranque, no hace
falta pista larga. Lo que hace falta son repeticiones, porque 2 mm de
deslizamiento por viaje son invisibles sueltos y obvios acumulados.

COMO CORRERLO
  1. Robot sobre el tapete, 70 cm libres por delante.
  2. Cinta en el piso marcando el borde delantero del robot.
  3. Correlo. Entre bloque y bloque espera que aprietes el boton central:
     aprovecha para mirar cuanto se corrio el robot respecto de la cinta.

QUE MIRAR
  - Deriva de rumbo: el patinaje en recta casi nunca es simetrico, asi que
    deja el robot girado. Es la medicion automatica, no depende de tu ojo.
  - Distancia contra la cinta: mide el deslizamiento longitudinal.
  Elegi la aceleracion mas alta que todavia vuelva limpia en las dos.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
import config

PISTA_MM = 600           # ida y vuelta de cada viaje
PISTA_TECHO_MM = 400     # para medir la velocidad maxima
REPETICIONES = 4         # viajes de ida y vuelta por cada aceleracion
ACELERACIONES = (700, 1500, 2000, 3000, 4000)

hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
izq = Motor(config.PORT_MOTOR_IZQ, Direction.COUNTERCLOCKWISE)
der = Motor(config.PORT_MOTOR_DER, Direction.CLOCKWISE)


def esperar_boton(mensaje):
    print("")
    print(">>> %s" % mensaje)
    print(">>> Apreta el boton central para seguir.")
    hub.speaker.beep(500, 120)
    while Button.CENTER in hub.buttons.pressed():
        wait(50)
    while Button.CENTER not in hub.buttons.pressed():
        wait(50)
    while Button.CENTER in hub.buttons.pressed():
        wait(50)


print("")
print("==========================================")
print(" 1. LIMITES DECLARADOS POR LOS MOTORES")
print("==========================================")
print("motor izquierdo (vel, acel, torque):", izq.control.limits())
print("motor derecho   (vel, acel, torque):", der.control.limits())

circunferencia = 3.1416 * config.DIAMETRO_RUEDA
tope_motor = izq.control.limits()[0]
print("")
print("Circunferencia de rueda: %d mm" % circunferencia)
print("Techo teorico:           %d mm/s" % (tope_motor / 360.0 * circunferencia))

base = DriveBase(izq, der, config.DIAMETRO_RUEDA, config.SEPARACION_RUEDAS)
base.use_gyro(True)

por_defecto = base.settings()
print("")
print("settings() automaticos de Pybricks:", por_defecto)
print("Pybricks los calibra al ~40%% del maximo -> maximo ~= %d mm/s" % (por_defecto[0] / 0.4))
print("Tu config.py fuerza straight_speed = %d mm/s" % config.STRAIGHT_SPEED)

print("")
print("==========================================")
print(" 2. TECHO REAL DE VELOCIDAD (%d mm)" % PISTA_TECHO_MM)
print("==========================================")
print("Pidiendo una velocidad imposible para ver donde satura el motor...")

base.settings(straight_speed=2000, straight_acceleration=4000)
base.straight(PISTA_TECHO_MM, then=Stop.BRAKE, wait=False)

velocidad_pico = 0
while not base.done():
    v = abs(base.state()[1])
    if v > velocidad_pico:
        velocidad_pico = v
    wait(5)

print("")
print(">>> VELOCIDAD MAXIMA MEDIDA: %d mm/s <<<" % velocidad_pico)
if config.STRAIGHT_SPEED > velocidad_pico:
    print("    Tu STRAIGHT_SPEED (%d) esta POR ENCIMA del techo fisico." % config.STRAIGHT_SPEED)
    print("    Subirlo no hace nada: la palanca de velocidad es la aceleracion.")
else:
    print("    Tu STRAIGHT_SPEED (%d) esta por debajo del techo." % config.STRAIGHT_SPEED)
    print("    Subirlo hasta %d si te sobra pista de aceleracion." % velocidad_pico)

base.settings(straight_speed=velocidad_pico, straight_acceleration=4000)
base.straight(-PISTA_TECHO_MM)
wait(500)

esperar_boton("Volve a poner el robot exactamente sobre la cinta.")

print("")
print("==========================================")
print(" 3. BARRIDO DE ACELERACION")
print("    %d viajes de %d mm por cada valor" % (REPETICIONES, PISTA_MM))
print("==========================================")

reloj = StopWatch()
resultados = []

for accel in ACELERACIONES:
    base.settings(straight_speed=velocidad_pico, straight_acceleration=accel)
    wait(300)

    hub.imu.reset_heading(0)
    wait(200)

    reloj.reset()
    for _ in range(REPETICIONES):
        base.straight(PISTA_MM, then=Stop.BRAKE)
        base.straight(-PISTA_MM, then=Stop.BRAKE)
    total_ms = reloj.time()

    wait(300)
    deriva = hub.imu.heading()
    por_viaje = total_ms / (REPETICIONES * 2.0)
    resultados.append((accel, total_ms, por_viaje, deriva))

    print("")
    print("  accel %4d mm/s2" % accel)
    print("     tiempo por tramo : %d ms" % por_viaje)
    print("     deriva de rumbo  : %.1f grados  %s"
          % (deriva, "<-- PATINA" if abs(deriva) > 2 else "ok"))

    esperar_boton("Mira la cinta: cuanto se corrio el robot con accel %d?" % accel)

print("")
print("==========================================")
print(" RESUMEN")
print("==========================================")
referencia = resultados[0][2]
for accel, total, por_viaje, deriva in resultados:
    print("  accel %4d -> %4d ms/tramo | ahorro %3d ms | deriva %5.1f deg %s"
          % (accel, por_viaje, referencia - por_viaje, deriva,
             "PATINA" if abs(deriva) > 2 else ""))

print("")
print("Elegi la aceleracion mas alta que cumpla LAS DOS cosas:")
print("  - deriva de rumbo por debajo de 2 grados")
print("  - el robot vuelve sobre la cinta")
print("Y ponela en config.py como STRAIGHT_ACCEL.")
hub.speaker.beep(900, 300)
