<div align="center">

# WRO 2026 - Pybricks Robot Codebase
*Navegación Absoluta y Movimiento Fluido (Encadenamiento)*

![Status: En Testeo](https://img.shields.io/badge/status-En%20Testeo-blue)
![FrameWork: Pybricks](https://img.shields.io/badge/framework-Pybricks-ED1C24)

</div>

## Propósito de esta Versión

Esta rama integra dos arquitecturas de software críticas para la velocidad y precisión a nivel competitivo: la **Navegación por Coordenadas Absolutas (Absolute Heading)** para domar el error del giroscopio a largo plazo, y el nuevo **Sistema de Encadenamiento Inercial** para suprimir los tiempos muertos entre comandos.

**Características clave implementadas:**
* **Encadenamiento Inercial (`encadenado=True`):** Bypass del perfil de desaceleración trapezoidal nativo de Pybricks. Inyecta un micro-freno electromagnético de 6 milisegundos que permite enlazar secuencias de movimiento agresivas a máxima velocidad sin cabeceo mecánico ni detención a cero.
* **Giros Absolutos (`giro_absoluto_pd`):** Capacidad de girar hacia un ángulo fijo del mapa mental del robot, absorbiendo y corrigiendo automáticamente cualquier derrape o desfase inercial heredado del movimiento anterior.
* **Cuadratura Física:** Lógica para chocar intencionalmente contra los muros y reiniciar la orientación del IMU (mitigación del *drift* y acumulación de error).
* **Optimización de Procesador:** Los sonidos de confirmación de movimiento han sido desactivados (`config.SONIDO_ACTIVO = False`) para evitar bloqueos por latencia en el procesador durante transiciones fluidas de alta velocidad.

---

## Configuración de Hardware (5 Motores)

* **Tracción:** Izquierdo (B) | Derecho (E)
* **Mecanismos:** Pinza Delantera (A) | Elevador Delantero (C) | Garra Trasera (F)
* **Sensores:** Color Frontal (D)

---

## Estado Actual

### Misiones (Recorrido completo)

Todas las misiones han sido programadas y funcionan de principio a fin. La fase actual es de **testeo exhaustivo** para afinar valores, detectar casos borde y garantizar la consistencia en competencia.

| Misión | Estado | Notas |
|---|---|---|
| 🟢 Bloques Blancos (agarrar) | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Detectar Mosaico | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Bloques Blancos (dejar) | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Bloques Verdes (agarrar) | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Bloques Verdes (dejar) | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Bloques Amarillos (agarrar) | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Bloques Amarillos (dejar) | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Cemento y Llana | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Bloques Azules (agarrar) | Completada | Estable. Pendiente de testeo exhaustivo. |
| 🟢 Bloques Azules (dejar) | Completada | Estable. Pendiente de testeo exhaustivo. |

### Matrices de Mosaicos (`ArmadorMosaicos.py`)

| # | Mosaico | Estado | Detalle |
|---|---|---|---|
| 1 | Verde-Verde | 🔴 No iniciada | Pendiente hasta completar la azul. |
| 2 | Verde-Amarillo | 🔴 No funcional | Código escrito pero no funciona correctamente. |
| 3 | Azul | 🟡 En desarrollo (~50%) | Primera mitad lista (recoger/dejar piezas). Falta la segunda mitad: posicionar y dejar las piezas finales en la matriz. |
| 4 | Amarillo | 🔴 No iniciada | Pendiente hasta completar la azul. |
| 5 | Blanco | 🔴 No iniciada | Pendiente hasta completar la azul. |

**Prioridad actual:** Completar la matriz azul → corregir verde-amarillo → desarrollar las restantes.

---

## Para ejecutar las pruebas
1. Conecta el PrimeHub vía Bluetooth.
2. Abre el proyecto en VS Code con la extensión `pybricksdev`.
3. Ejecuta la tarea (F5) apuntando al archivo `app.py`.