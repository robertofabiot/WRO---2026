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

La arquitectura de las misiones (`app.py` y `Misiones.py`) ha sido completamente refactorizada para explotar las transiciones fluidas. Tras las pruebas del 10/6/26, se registran 16 intentos totales con 16 fallas y 0 completaciones.

**Progreso de las Misiones:**
* 🟢 **Bloques Blancos:** Estable. 0 fallos al agarrar y dejar los bloques.
* 🟢 **Bloques Verdes:** Estable. 0 fallos al agarrar y dejar los bloques.
* 🟡 **Bloques Amarillos:** Inestable. 3 fallos registrados al intentar agarrarlos y 5 fallos al intentar dejarlos.
* 🔴 **Cemento y Llana:** Crítico. 4 fallos totales acumulados en la manipulación y navegación de esta misión.
* 🔴 **Detectar Mosaico:** Inestable. 3 fallos totales registrados.
* 🔴 **Bloques Azules:** En desarrollo. 1 fallo registrado al intentar agarrarlos y 0 al dejarlos.

**Errores Frecuentes y Reporte de Fallos:**
* **Falsos positivos en el seguidor de línea:** El robot detectó un cruce de más en el seguidor y paró antes durante las misiones de dejar bloques amarillos y en cemento y llana. La hipótesis es que avanza de más, y en el tiempo que dilata en acomodarse, el sensor detecta un cruce fantasma.
* **Desalineación en transiciones:** El chasis no se acomodó bien en el inicio (seguidor y giro) al ir a agarrar bloques amarillos. La causa identificada es que, al realizar el giro de la misión anterior (dejar bloques verdes), el robot queda posicionado al otro lado de la línea del seguidor.
* **Problemas con el giro inercial (Cemento y Llana):** El robot no pudo agarrar la pala en múltiples ocasiones. En uno de los intentos, agarró la pala pero giró hacia el otro lado. La hipótesis sugiere que el robot quedó muy pegado a la pared, afectando severamente el giro posterior del seguidor.
* **Desviación de trayectoria:** Durante la misión de agarrar bloques azules, el robot quedó muy a la izquierda de la línea y no agarró el seguidor. Asimismo, en la detección del mosaico, se falló en múltiples intentos porque el chasis directamente no llegó al seguidor.

**Problemas Conocidos:**
* **Método `latigazo` (en `Chasis.py`):** Actualmente no funcional debido a un error `ValueError: Invalid argument` al intentar configurar los límites del control de giro. Se está investigando la compatibilidad con el firmware actual de Pybricks.

**Para ejecutar las pruebas:**
1. Conecta el PrimeHub vía Bluetooth.
2. Abre el proyecto en VS Code con la extensión `pybricksdev`.
3. Ejecuta la tarea (F5) apuntando al archivo `app.py`.