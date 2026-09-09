"""Calibrador interactivo de límites para mecanismos del robot migrado.

Permite encontrar los topes mecánicos físicos de los 3 actuadores
(Torque en Port D, Garra Delantera en Port B y Garra Principal en Port F)
sin forzar los engranajes, obteniendo los grados exactos para config.py.
"""

import sys
try:
    import config
    from robot import Robot
except ImportError:
    sys.path.append("..")
    sys.path.append(".")
    import config
    from robot import Robot

from pybricks.tools import wait


def calibrar_mecanismo(nombre, motor, direccion_inicio, direccion_fin, vel=250, limite_potencia=35):
    """Calibra los topes mecánicos buscando ambos extremos a baja potencia."""
    print("\n" + "-" * 50)
    print("--- Calibrando mecanismo: %s ---" % nombre)
    print("1. Buscando el TOPE INICIAL (origen 0°) a baja potencia...")

    signo_inicio = -abs(vel) if direccion_inicio == "negativo" else abs(vel)
    motor.run_until_stalled(signo_inicio, duty_limit=limite_potencia)

    print("   -> ¡Tope inicial alcanzado! Fijando encoder a 0°.")
    motor.reset_angle(0)
    wait(500)

    print("2. Buscando el TOPE FINAL (rango máximo)...")
    signo_fin = abs(vel) if direccion_fin == "positivo" else -abs(vel)
    angulo_fin = motor.run_until_stalled(signo_fin, duty_limit=limite_potencia)

    grados_totales = motor.angle()
    print("   -> ¡Tope final alcanzado!")

    print("\n" + "=" * 50)
    print("✅ CALIBRACIÓN DE %s FINALIZADA" % nombre.upper())
    print("=" * 50)
    print("  Rango angular medido : %d grados" % grados_totales)
    print("  Magnitud absoluta    : %d grados" % abs(grados_totales))
    print("=" * 50)

    # Regresar al origen
    print("Regresando a posición inicial...")
    motor.run_target(abs(vel) * 2, 0, wait=True)
    return abs(grados_totales)


def main():
    print("========================================")
    print("   CALIBRADOR DE MECANISMOS (WRO 2026)  ")
    print("========================================")
    print("Inicializando hardware del robot...")
    robot = Robot()

    while True:
        print("\nSelecciona el mecanismo a calibrar:")
        print("  1 - Mecanismo de Torque / Jaula Trasera (Puerto %s)" % config.PORT_TORQUE)
        print("  2 - Garra Delantera / Elevador (Puerto %s)" % config.PORT_GARRA_DELANTERA)
        print("  3 - Garra Principal / Pinza (Puerto %s)" % config.PORT_GARRA_PRINCIPAL)
        print("  Q - Salir")

        try:
            opcion = input("Escribe opción y pulsa Enter: ").strip().lower()
        except Exception:
            break

        if opcion == "1":
            rango = calibrar_mecanismo(
                "Mecanismo de Torque",
                robot.motor_torque,
                direccion_inicio="positivo",
                direccion_fin="negativo",
                vel=250,
                limite_potencia=40
            )
            print("🌟 VALOR RECOMENDADO EN config.py -> RANGO_MAXIMO_TORQUE = %d" % rango)

        elif opcion == "2":
            rango = calibrar_mecanismo(
                "Garra Delantera",
                robot.motor_garra_delantera,
                direccion_inicio="negativo",
                direccion_fin="positivo",
                vel=250,
                limite_potencia=40
            )
            print("🌟 VALOR RECOMENDADO EN config.py -> RANGO_MAXIMO_GARRA_DELANTERA = %d" % rango)

        elif opcion == "3":
            rango = calibrar_mecanismo(
                "Garra Principal / Pinza",
                robot.motor_garra_principal,
                direccion_inicio="negativo",
                direccion_fin="positivo",
                vel=250,
                limite_potencia=40
            )
            print("🌟 VALOR RECOMENDADO EN config.py -> RANGO_MAXIMO_GARRA_PRINCIPAL = %d" % rango)

        elif opcion == "q":
            print("Saliendo del calibrador...")
            break
        else:
            print("Opción inválida. Intenta nuevamente.")


if __name__ == "__main__":
    main()
