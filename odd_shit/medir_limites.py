"""Medidor de límites reales del chasis para el robot migrado.

CORRER SOBRE EL TAPETE DE COMPETENCIA.

La aceleración máxima no es una propiedad intrínseca del robot sino de la
superficie: depende del punto en el que la rueda patina contra el tapete.
Medirlo en una mesa o piso da un valor equivocado; si la aceleración queda
demasiado alta, el patinaje corrompe la odometría y descalibra los recorridos.

CÓMO CORRERLO:
  1. Colocar el robot sobre el tapete con al menos 70 cm libres al frente.
  2. Colocar una cinta en el suelo marcando el borde delantero del robot.
  3. Ejecutar este script. Entre bloque y bloque espera pulsar el botón central
     del Hub: aprovecha para revisar cuánto se desplazó el robot de la cinta.

QUÉ OBSERVAR:
  - Deriva de rumbo: el patinaje longitudinal rara vez es simétrico, por lo que
    deja el robot girado. Es una medición automática del IMU.
  - Distancia contra la cinta: mide el deslizamiento longitudinal real.
  Elige la aceleración más alta que conserve ambas lecturas limpias (deriva < 2°).
"""

import sys
try:
    import config
except ImportError:
    sys.path.append("..")
    sys.path.append(".")
    import config

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Direction, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

PISTA_MM = 600           # Distancia de ida y vuelta de cada viaje
PISTA_TECHO_MM = 400     # Distancia para medir la velocidad máxima real
REPETICIONES = 4         # Cantidad de viajes de ida y vuelta por cada aceleración
ACELERACIONES = (700, 1500, 2000, 3000, 4000)

# Inicialización con el cableado del robot configurado
hub = PrimeHub()
izq = Motor(config.PORT_MOTOR_IZQ, config.DIRECCION_MOTOR_IZQ)
der = Motor(config.PORT_MOTOR_DER, config.DIRECCION_MOTOR_DER)


def esperar_boton(mensaje):
    print("")
    print(">>> %s" % mensaje)
    print(">>> Presiona el botón central del Hub para continuar...")
    try:
        hub.speaker.beep(500, 120)
    except Exception:
        pass

    while Button.CENTER in hub.buttons.pressed():
        wait(50)
    while Button.CENTER not in hub.buttons.pressed():
        wait(50)
    while Button.CENTER in hub.buttons.pressed():
        wait(50)


print("")
print("==========================================")
print(" 1. LÍMITES DECLARADOS POR LOS MOTORES")
print("==========================================")
print("Motor izquierdo Port %s (vel, acel, torque):" % config.PORT_MOTOR_IZQ, izq.control.limits())
print("Motor derecho   Port %s (vel, acel, torque):" % config.PORT_MOTOR_DER, der.control.limits())

circunferencia = 3.1416 * config.DIAMETRO_RUEDA
tope_motor = izq.control.limits()[0]
print("")
print("Circunferencia de rueda: %d mm" % circunferencia)
print("Ancho de vía (axle track): %d mm" % config.SEPARACION_RUEDAS)
print("Techo teórico de tracción: %d mm/s" % (tope_motor / 360.0 * circunferencia))

base = DriveBase(izq, der, config.DIAMETRO_RUEDA, config.SEPARACION_RUEDAS)
base.use_gyro(True)

por_defecto = base.settings()
print("")
print("settings() automáticos de Pybricks:", por_defecto)
print("Pybricks calibra al ~40%% del máximo -> Máximo estimado ~= %d mm/s" % (por_defecto[0] / 0.4))
print("Tu config.py tiene configurado VELOCIDAD_MAX_RECTA = %d mm/s" % config.VELOCIDAD_MAX_RECTA)

print("")
print("==========================================")
print(" 2. TECHO REAL DE VELOCIDAD (%d mm)" % PISTA_TECHO_MM)
print("==========================================")
print("Exigiendo velocidad máxima para medir saturación del motor en carga...")

base.settings(straight_speed=2000, straight_acceleration=4000)
base.straight(PISTA_TECHO_MM, then=Stop.BRAKE, wait=False)

velocidad_pico = 0
while not base.done():
    v = abs(base.state()[1])
    if v > velocidad_pico:
        velocidad_pico = v
    wait(5)

print("")
print(">>> VELOCIDAD MÁXIMA MEDIDA: %d mm/s <<<" % velocidad_pico)
if config.VELOCIDAD_MAX_RECTA > velocidad_pico:
    print("    AVISO: Tu VELOCIDAD_MAX_RECTA (%d mm/s) supera el techo físico real." % config.VELOCIDAD_MAX_RECTA)
    print("    El motor no podrá alcanzarla; la palanca de tiempo es la aceleración.")
else:
    print("    Tu VELOCIDAD_MAX_RECTA (%d mm/s) está dentro del límite seguro." % config.VELOCIDAD_MAX_RECTA)
    print("    Podrías incrementarla hasta %d mm/s si el tramo es largo." % velocidad_pico)

base.settings(straight_speed=velocidad_pico, straight_acceleration=4000)
base.straight(-PISTA_TECHO_MM)
wait(500)

esperar_boton("Coloca nuevamente el robot exactamente sobre la cinta testigo.")

print("")
print("==========================================")
print(" 3. BARRIDO DE ACELERACIÓN")
print("    %d ciclos ida y vuelta de %d mm por cada valor" % (REPETICIONES, PISTA_MM))
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
    print("  Aceleración %4d mm/s²" % accel)
    print("     Tiempo medio por tramo : %d ms" % por_viaje)
    print("     Deriva de rumbo final  : %.1f grados  %s"
          % (deriva, "<-- ¡PATINA!" if abs(deriva) > 2.0 else "OK (estable)"))

    esperar_boton("Revisa la cinta: ¿cuánto se desplazó el robot con accel %d mm/s²?" % accel)

print("")
print("==========================================")
print(" RESUMEN COMPARATIVO")
print("==========================================")
referencia = resultados[0][2]
for accel, total, por_viaje, deriva in resultados:
    print("  accel %4d mm/s² -> %4d ms/tramo | Ahorro: %3d ms | Deriva: %5.1f° %s"
          % (accel, por_viaje, referencia - por_viaje, deriva,
             "¡PATINA!" if abs(deriva) > 2.0 else "OK"))

print("")
print("CONCLUSIÓN:")
print("  Elige la aceleración más alta donde:")
print("    1. La deriva de rumbo permanezca por debajo de 2.0°.")
print("    2. El robot regrese alineado sobre la cinta testigo.")
print("  Y anótala en config.py como la aceleración recomendada.")

try:
    hub.speaker.beep(900, 300)
except Exception:
    pass
