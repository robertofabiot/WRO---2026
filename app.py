"""Punto de entrada principal del robot para WRO 2026.

Basado en el recorrido actualizado de prueba_reto_1.
Permite ejecutar cada sección de forma independiente para pruebas en pista
o encadenarlas consecutivamente para la corrida completa.
"""

from robot import Robot
from Misiones import Misiones
from ArmadorMosaicos import ArmadorMosaicos
from RevisadorBateria import RevisadorBateria
import gc

# 1. Instanciación del robot y subsistemas
mi_robot = Robot()
misiones = Misiones(mi_robot)
armador = ArmadorMosaicos(mi_robot)
revisador_bateria = RevisadorBateria(mi_robot)

if __name__ == "__main__":
    print("--- INICIANDO ROBOT WRO 2026 (RAMA ACTUALIZADA) ---")

    # 2. Chequeo de batería
    if not revisador_bateria.revisar_bateria():
        print("Ejecución cancelada por nivel de batería.")
    else:
        print("Batería verificada. Listo para ejecutar.")

        # =====================================================================
        # ZONA DE EJECUCIÓN / PRUEBAS (Basado en prueba_reto_1):
        # Descomenta las secciones que desees probar individualmente
        # o descoméntalas todas en orden para la corrida completa.
        # =====================================================================

        # SECCIÓN 1: Salida en arco, seguidor, toma de cemento y empuje de llana
        # misiones.seccion_1_salida_y_cemento()

        # SECCIÓN 2: Dejar cemento con avance coordinado y agarrar cementos verdes
        # misiones.seccion_2_dejar_cemento_y_tomar_verdes()

        # SECCIÓN 3: Seguidor hasta matriz, escanearla, retroceso y dejar verdes
        # matriz_detectada = misiones.seccion_3_escanear_matriz_y_dejar_verdes(armador)

        # SECCIÓN 4: Navegar pasillo de amarillos y tomar bloques azules
        # misiones.seccion_4_amarillos_y_azules()

        # SECCIÓN 5: Agarrar pala con pinza/frontal y dejar bloques amarillos
        # misiones.seccion_5_tomar_pala_y_dejar_amarillos()

        # SECCIÓN 6: Retorno con pala, seguidor con torque a los 65 cm y armado de matriz
        # misiones.seccion_6_retorno_pala_y_fin(armador, matriz_detectada=None)

        # O ejecución directa de una matriz específica:
        # armador.armar(numero_mosaico=2)
