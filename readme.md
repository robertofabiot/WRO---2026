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
* **Giros sobre un único lazo PD (`giro_absoluto` / `giro_relativo`):** Control DC directo a 500 Hz, ganancias adaptativas según el error restante, filtro EMA en la derivada, rampa de arranque y detección de patinaje. `giro_absoluto` apunta a un rumbo fijo del mapa y absorbe el derrape heredado; `giro_relativo` gira desde donde esté el robot. El parámetro `rueda_pivote` (`None`, `"izquierda"` o `"derecha"`) elige si el giro es sobre el eje o pivotando sobre una rueda quieta.
* **Cuadratura Física:** Lógica para chocar intencionalmente contra los muros y reiniciar la orientación del IMU (mitigación del *drift* y acumulación de error).
* **Optimización de Procesador:** Los sonidos de confirmación de movimiento han sido desactivados (`config.SONIDO_ACTIVO = False`) para evitar bloqueos por latencia en el procesador durante transiciones fluidas de alta velocidad.

---

## Estructura del Código

| Archivo | Responsabilidad |
|---|---|
| `app.py` | Punto de entrada. Arma el hardware y ejecuta la corrida o una misión suelta. |
| `config.py` | Puertos, medidas físicas, velocidades, rangos calibrados y timeouts de seguridad. |
| `robot.py` | Único lugar donde se instancian los motores; cablea el resto de las clases. |
| `Chasis.py` | Movimientos a ciegas del tren de tracción: rectas, cuadratura y motores sueltos. |
| `Navegacion.py` | Todo lo que cierra un lazo con el IMU o el sensor de color: giros, seguidores de línea y avances hasta color. |
| `Mecanismos.py` | Garra delantera con pinza y jaula trasera, por porcentaje del rango calibrado. |
| `Misiones.py` | Una misión del recorrido por método, en orden de ejecución. |
| `ArmadorMosaicos.py` | Rutina de armado de la matriz, una por mosaico posible. |
| `RevisadorBateria.py` | Chequeo de batería previo a la corrida. |
| `odd_shit/` | Herramientas sueltas de calibración y diagnóstico, fuera del flujo de competencia. |

---

## Configuración de Hardware (5 Motores)

* **Tracción:** Izquierdo (B) | Derecho (E)
* **Mecanismos:** Pinza Delantera (A) | Elevador Delantero (C) | Garra Trasera (F)
* **Sensores:** Color Frontal (D)

---

## Estado Actual

### Misiones (Recorrido completo)

Todas las misiones han sido programadas y funcionan de principio a fin. La fase actual es de **testeo exhaustivo** para afinar valores, detectar casos borde y garantizar la consistencia en competencia.

Cada misión arranca donde termina la anterior, así que para probar una sola hay que dejar el robot en la posición en la que la previa lo dejaría.

| # | Misión (`Misiones.py`) | Estado | Notas |
|---|---|---|---|
| 1 | 🟢 `cemento_y_llana` | Completada | Estable. Pendiente de testeo exhaustivo. |
| 2 | 🟢 `recoger_pala` | Completada | Estable. Pendiente de testeo exhaustivo. |
| 3 | 🟢 `dejar_cemento` | Completada | Estable. Pendiente de testeo exhaustivo. |
| 4 | 🟢 `recoger_verdes` | Completada | Estable. Pendiente de testeo exhaustivo. |
| 5 | 🟢 `detectar_mosaico` | Completada | Devuelve el número de mosaico que consume el armador. |
| 6 | 🟢 `dejar_verdes` | Completada | Estable. Pendiente de testeo exhaustivo. |
| 7 | 🟢 `agarrar_amarillos` | Completada | Estable. Pendiente de testeo exhaustivo. |
| 8 | 🟢 `agarrar_azules` | Completada | Estable. Pendiente de testeo exhaustivo. |
| 9 | 🟢 `dejar_amarillos` | Completada | Estable. Pendiente de testeo exhaustivo. |
| 10 | 🟢 `dejar_pala` | Completada | Estable. Pendiente de testeo exhaustivo. |

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
