"""
Diagnostico de cuadratura contra pared.

Nova reporto que al empujar contra la pared las ruedas NO se frenan: siguen
rodando patinando sobre el tapete. Por eso cualquier deteccion basada en
motor.speed() falla y hay que usar un timeout fijo.

Este script no propone una solucion: mide cual de las senales candidatas
realmente reacciona al choque en TU tapete, para poder elegir con datos.

  motor.speed()              cae solo si se traba de verdad
  motor.load()               deberia subir aunque patine (sigue forzando)
  motor.stalled()            probablemente sea False siempre con dc()
  imu.angular_velocity(Z)    el rumbo deja de cambiar al quedar plano

Ademas barre la potencia para responder la pregunta clave:
  A que potencia deja de patinar y empieza a trabarse?
Si existe una potencia baja donde se traba, sirve un empuje en dos etapas.

COMO CORRERLO
  Etapa A: el robot YA pegado a la pared, en la orientacion de la mision.
  Etapa B: el robot a unos 15 cm de la pared, mirando hacia ella.
  Entre etapa y etapa espera el boton central.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
import config

POTENCIAS = (20, 30, 40, 60, 80, 100)
MS_POR_POTENCIA = 500
REVERSA = True                # como lo usa agarrar_bloques_blancos
POTENCIA_APROXIMACION = 80
MS_APROXIMACION = 1500
PASO_MS = 20

hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
izq = Motor(config.PORT_MOTOR_IZQ, Direction.COUNTERCLOCKWISE)
der = Motor(config.PORT_MOTOR_DER, Direction.CLOCKWISE)
base = DriveBase(izq, der, config.DIAMETRO_RUEDA, config.SEPARACION_RUEDAS)

signo = -1 if REVERSA else 1


def esperar_boton(mensaje):
    print("")
    print(">>> %s" % mensaje)
    print(">>> Apreta el boton central.")
    hub.speaker.beep(500, 120)
    while Button.CENTER in hub.buttons.pressed():
        wait(50)
    while Button.CENTER not in hub.buttons.pressed():
        wait(50)
    while Button.CENTER in hub.buttons.pressed():
        wait(50)


def frenar():
    izq.brake()
    der.brake()
    wait(200)


esperar_boton("ETAPA A: pone el robot YA pegado a la pared.")

print("")
print("==================================================")
print(" ETAPA A: a que potencia deja de patinar?")
print("==================================================")
print(" pot | vel_izq | vel_der |  load_izq | load_der | trabado")
print(" ----+---------+---------+-----------+----------+--------")

resultados_a = []
for pot in POTENCIAS:
    izq.dc(signo * pot)
    der.dc(signo * pot)
    wait(150)                      # que arranquen

    v_izq = v_der = c_izq = c_der = 0
    n = 0
    reloj = StopWatch()
    while reloj.time() < MS_POR_POTENCIA:
        v_izq += abs(izq.speed())
        v_der += abs(der.speed())
        c_izq += abs(izq.load())
        c_der += abs(der.load())
        n += 1
        wait(PASO_MS)

    trabado = izq.stalled() or der.stalled()
    frenar()

    v_izq, v_der = v_izq / n, v_der / n
    c_izq, c_der = c_izq / n, c_der / n
    resultados_a.append((pot, v_izq, v_der, c_izq, c_der, trabado))
    print(" %3d | %7.0f | %7.0f | %9.0f | %8.0f | %s"
          % (pot, v_izq, v_der, c_izq, c_der, trabado))

print("")
print("Velocidad en grados/s, carga en mNm.")
print("Una potencia con velocidad cercana a 0 = se traba (sirve para detectar).")
print("Velocidad alta = patina.")

umbral = None
for pot, v_izq, v_der, _, _, _ in resultados_a:
    if max(v_izq, v_der) < 40:
        umbral = pot
if umbral is None:
    print(">>> Patina en TODAS las potencias probadas. Descartar deteccion por velocidad.")
else:
    print(">>> Se traba hasta %d de potencia. Un empuje en dos etapas es viable." % umbral)

esperar_boton("ETAPA B: pone el robot a ~15 cm de la pared, mirando hacia ella.")

print("")
print("==================================================")
print(" ETAPA B: que senal delata el momento del choque?")
print("==================================================")

hub.imu.reset_heading(0)
wait(200)
dist_inicial = base.distance()

muestras = []
izq.dc(signo * POTENCIA_APROXIMACION)
der.dc(signo * POTENCIA_APROXIMACION)

reloj = StopWatch()
while reloj.time() < MS_APROXIMACION:
    muestras.append((
        reloj.time(),
        abs(izq.speed()),
        abs(izq.load()),
        abs(der.load()),
        hub.imu.angular_velocity(Axis.Z),
        abs(base.distance() - dist_inicial),
    ))
    wait(PASO_MS)

frenar()

print("")
print("   ms | vel_izq | load_izq | load_der | giro_z | recorrido_mm")
print(" -----+---------+----------+----------+--------+-------------")
for t, v, ci, cd, gz, d in muestras:
    print(" %4d | %7.0f | %8.0f | %8.0f | %6.1f | %d" % (t, v, ci, cd, gz, d))

print("")
print("==================================================")
print(" QUE MIRAR")
print("==================================================")
print("Busca la fila donde el recorrido deja de crecer: ese es el choque.")
print("Despues fijate cual columna cambia de forma clara en ese instante:")
print("  vel_izq  cae  -> sirve deteccion por velocidad (la que ya fallo)")
print("  load     sube -> sirve deteccion por carga, aunque patine")
print("  giro_z   se va a ~0 y se queda -> sirve el giroscopo, ignora las ruedas")
print("Si ninguna cambia de forma clara, el timeout fijo es la respuesta correcta")
print("y lo dejamos como esta.")
hub.speaker.beep(900, 300)
