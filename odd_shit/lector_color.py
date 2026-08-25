"""
Lector de Color en Tiempo Real
Imprime toda la info del sensor de color en loop para comparar superficies.
Útil para distinguir entre dos tipos de negro (o cualquier color similar).
"""

from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# --- CONFIG ---
PUERTO_SENSOR = Port.D
INTERVALO_MS = 200  # cada cuántos ms imprime

# --- INIT ---
sensor = ColorSensor(PUERTO_SENSOR)

print("=== LECTOR DE COLOR ===")
print("Poné la superficie abajo del sensor.")
print("Ctrl+C para salir.\n")

while True:
    color = sensor.color()
    h, s, v = sensor.hsv()
    reflejo = sensor.reflection()

    print(
        "Color: {:10s} | H: {:3d}  S: {:3d}  V: {:3d} | Reflejo: {:3d}%".format(
            str(color), h, s, v, reflejo
        )
    )

    wait(INTERVALO_MS)
