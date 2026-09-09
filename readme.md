<div align="center">

# WRO 2026

**Plataforma autónoma de navegación, seguimiento de línea por PID y control de mecanismos por porcentaje.**

*World Robot Olympiad (WRO) 2026 — Categoría Senior · Team Perrozompopo · Nicaragua*

[![Python 3](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](config.py)
[![Pybricks](https://img.shields.io/badge/Pybricks-MicroPython-008559?logo=lego&logoColor=white)](https://pybricks.com/)
[![WRO 2026](https://img.shields.io/badge/WRO-2026%20Senior-E65100)](https://wro-association.org/)
[![Hardware](https://img.shields.io/badge/Hardware-LEGO%20SPIKE%20Prime-005BBB)](config.py)
[![Arquitectura](https://img.shields.io/badge/Arquitectura-OOP%20Modular-blueviolet)](robot.py)
[![Control](https://img.shields.io/badge/Control-PID%20%2B%20Giroscopio-00C853)](Navegacion.py)

</div>

---

## Tabla de contenidos

1. [Descripción](#descripción)
2. [Arquitectura](#arquitectura)
3. [Hardware y límites](#hardware-y-límites)
4. [Recorrido de misiones](#recorrido-de-misiones)
5. [Ejecución](#ejecución)
6. [Colaboradores](#colaboradores)

---

## Descripción

Código autónomo de competencia desarrollado en MicroPython sobre Pybricks para la categoría **Senior** de la World Robot Olympiad (WRO) 2026 por el equipo **Team Perrozompopo**.

Implementa odometría con corrección continua por giroscopio (IMU), seguidor de línea mediante algoritmo predictivo PID, clasificación de color en espacio HSV con votación estática, y control de mecanismos mediante posiciones normalizadas por porcentaje (`0% a 100%`) con auto-cero en topes mecánicos.

---

## Arquitectura

Estructura modular desacoplada en subsistemas:

```text
WRO---2026/
├── app.py                  # Punto de entrada principal y ejecutor de misiones
├── config.py               # Configuración central: puertos, PID, geometría y límites
├── robot.py                # Abstracción e inyección de hardware (Hub, motores, sensores)
├── Chasis.py               # Cinemática de tracción, rampas y avance recto con giroscopio
├── Navegacion.py           # Giros IMU (punto, arco, rumbo) y seguimiento de línea PID
├── Mecanismos.py           # Actuadores con posición por porcentaje y auto-cero
├── Misiones.py             # Rutinas del recorrido oficial divididas en 6 secciones
├── ArmadorMosaicos.py      # Escaneo de color y armado de matrices 1 a 4
├── RevisadorBateria.py     # Diagnóstico de voltaje previo al arranque
├── Utils.py                # Operaciones matemáticas auxiliares y normalización
└── odd_shit/               # Herramientas de calibración (límites de motor y tracción)
```

---

## Hardware y límites

Configuración física del robot centralizada en [`config.py`](config.py):

| Componente | Puerto | Rango / Configuración |
|---|---|---|
| **Motor Izquierdo** | `Port.B` | `Direction.COUNTERCLOCKWISE` · Rueda `56 mm` |
| **Motor Derecho** | `Port.E` | `Direction.CLOCKWISE` · Rueda `56 mm` |
| **Separación entre ruedas** | — | `160 mm` (*axle track*) |
| **Jaula Trasera / Torque** | `Port.F` | `0%` (arriba) a `100%` (`-179°`) · Invertido |
| **Elevador Delantero** | `Port.C` | `0%` (arriba) a `100%` (`672°`) |
| **Pinza Principal** | `Port.A` | `0%` (cerrada con auto-cero) a `100%` (`798°`) |
| **Sensor de Color** | `Port.D` | Modos reflexión y color HSV calibrado |

---

## Recorrido de misiones

Las misiones de [`Misiones.py`](Misiones.py) ejecutan la secuencia oficial del reto:

1. **Sección 1:** Arco de salida (90°), seguidor de línea (79 cm), toma de cemento con torque al 93% y empuje de llana.
2. **Sección 2:** Seguidor (50 cm), avance coordinado subiendo torque al 0% y recolección de bloques verdes al 93.4%.
3. **Sección 3:** Llegada a zona de matriz, escaneo óptico estático y descarga de verdes con torque al 0%.
4. **Sección 4:** Navegación de pasillo, cruce de líneas y captura de bloques azules con torque al 93.4%.
5. **Sección 5:** Posicionamiento de elevador (69.4%) y pinza (47.6%), captura de pala, cierre a 0% y entrega de amarillos.
6. **Sección 6:** Retorno con pala, activación de torque a 0% en seguidor (65 cm), repliegue general y resolución de la matriz detectada vía [`ArmadorMosaicos.py`](ArmadorMosaicos.py).

---

## Ejecución

1. Abrir el proyecto en VS Code con la extensión oficial de **Pybricks**.
2. Conectar el PrimeHub por Bluetooth o USB.
3. Configurar en [`app.py`](app.py) la sección o matriz requerida y ejecutar:

```python
# app.py
from robot import Robot
from Misiones import Misiones
from ArmadorMosaicos import ArmadorMosaicos

mi_robot = Robot()
misiones = Misiones(mi_robot)
armador = ArmadorMosaicos(mi_robot)

# Ejecución completa
misiones.seccion_1_salida_y_cemento()
misiones.seccion_2_dejar_cemento_y_tomar_verdes()
matriz = misiones.seccion_3_escanear_matriz_y_dejar_verdes(armador)
misiones.seccion_4_amarillos_y_azules()
misiones.seccion_5_tomar_pala_y_dejar_amarillos()
misiones.seccion_6_retorno_pala_y_fin(armador, matriz)
```

---

## Colaboradores

<div align="center">
<table>
  <tr>
    <td align="center">
      <a href="https://github.com/robertofabiot">
        <img src="https://avatars.githubusercontent.com/u/203884931?v=4" width="100px;" alt="Roberto F. Tercero"/><br />
        <sub><b>Roberto F. Tercero</b></sub>
      </a><br />
      <sub>Team Perrozompopo</sub>
    </td>
    <td align="center">
      <a href="https://github.com/uxvcharlie">
        <img src="https://avatars.githubusercontent.com/u/211022677?v=4" width="100px;" alt="Carlos Rafael Umaña Vásquez"/><br />
        <sub><b>Carlos Rafael Umaña Vásquez</b></sub>
      </a><br />
      <sub>Team Perrozompopo</sub>
    </td>
  </tr>
</table>
</div>

---

<div align="center">
<sub>World Robot Olympiad 2026 — Categoría Senior · Team Perrozompopo. Desarrollado con <a href="https://pybricks.com/">Pybricks</a>.</sub>
</div>
