"""
Calibrador de detectar_color_preciso()
Pide el nombre del color por input, escanea, y repite.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

# --- CONFIG ---
PUERTO_SENSOR = Port.D
TIEMPO_POSICIONAMIENTO_MS = 5000
TIEMPO_ESCANEO_MS = 8000
INTERVALO_LECTURA_MS = 250

# --- Copia local de la función para referencia ---
def detectar_color_preciso(sensor):
    color_hsv = sensor.hsv()
    h, s, v = color_hsv.h, color_hsv.s, color_hsv.v
    if s < 35:
        if v > 65:
            return "WHITE"
        elif v >= 40:
            return "GRAY"
        else:
            return "BLACK"
    else:
        if h < 95 or h > 310:
            return "YELLOW"
        elif h < 185:
            return "GREEN"
        else:
            return "BLUE"

# --- INIT ---
hub = PrimeHub()
sensor = ColorSensor(PUERTO_SENSOR)

print("=" * 50)
print("  CALIBRADOR DE detectar_color_preciso()")
print("=" * 50)

while True:
    print("")
    nombre = input("Que color vas a escanear? (o 'salir'): ")

    if nombre.strip().lower() == "salir":
        print("Chau.")
        break

    # Cuenta regresiva
    print("Posiciona el sensor sobre: {}".format(nombre))
    crono = StopWatch()
    ultimo = -1
    while crono.time() < TIEMPO_POSICIONAMIENTO_MS:
        seg = (TIEMPO_POSICIONAMIENTO_MS - crono.time()) // 1000
        if seg != ultimo:
            ultimo = seg
            print("  {}...".format(seg + 1))
        wait(100)

    # Escaneo
    print("")
    print(">>> ESCANEANDO: {} <<<".format(nombre))
    print("  H     S     V   | Reflejo | Detectado")
    print("  ----+-----+-----+---------+----------")

    h_min, h_max = 999, -1
    s_min, s_max = 999, -1
    v_min, v_max = 999, -1
    r_min, r_max = 999, -1

    crono.reset()
    while crono.time() < TIEMPO_ESCANEO_MS:
        hsv = sensor.hsv()
        h, s, v = hsv.h, hsv.s, hsv.v
        reflejo = sensor.reflection()
        det = detectar_color_preciso(sensor)

        if h < h_min: h_min = h
        if h > h_max: h_max = h
        if s < s_min: s_min = s
        if s > s_max: s_max = s
        if v < v_min: v_min = v
        if v > v_max: v_max = v
        if reflejo < r_min: r_min = reflejo
        if reflejo > r_max: r_max = reflejo

        print("  {:3d}   {:3d}   {:3d} |  {:3d}%   | {}".format(
            h, s, v, reflejo, det
        ))
        wait(INTERVALO_LECTURA_MS)

    print("")
    print("  Rango H: {}-{}  S: {}-{}  V: {}-{}  R: {}-{}%".format(
        h_min, h_max, s_min, s_max, v_min, v_max, r_min, r_max
    ))
    print("-" * 50)
