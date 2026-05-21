<div align="center">

# WRO 2026 - Pybricks Robot Codebase
*Navegación Absoluta y Movimiento Fluido (Encadenamiento)*

![Status: Experimental](https://img.shields.io/badge/status-Experimental-yellow)
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

## Estado Actual y Ejecución

La arquitectura de las misiones (`app.py` y `Misiones.py`) ha sido completamente refactorizada para explotar las transiciones fluidas, alterando el orden estratégico de la ronda. Los espacios de alta precisión mantienen frenos totales (`encadenado=False`).

**Progreso de las Misiones:**
* 🟢 **Recogida Bloques Blancos:** Rutina de captura completada y **completamente estable**.
* 🟡 **Misión Bloques Verdes:** Rutina de entrega integrada, pendiente de calibración física y testeo en lona.
* 🟡 **Ajuste Cinemático:** Pendiente calibración general de variables `margen_cm` y `margen_grados` para sincronizar los tiempos de corte con la velocidad de derrape del robot.

**Para ejecutar las pruebas:**
1. Conecta el PrimeHub vía Bluetooth.
2. Abre el proyecto en VS Code con la extensión `pybricksdev`.
3. Ejecuta la tarea (F5) apuntando al archivo `app.py`.