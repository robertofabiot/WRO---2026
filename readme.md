<div align="center">

# WRO 2026 - Pybricks Robot Codebase
Lógica de control, navegación avanzada y rutinas para la World Robot Olympiad.

![Language: MicroPython](https://img.shields.io/badge/language-MicroPython-306998)
![FrameWork: Pybricks](https://img.shields.io/badge/framework-Pybricks-ED1C24)
![Status: In Development](https://img.shields.io/badge/status-In_Development-orange)
![License: MIT](https://img.shields.io/badge/license-MIT-blue)

</div>

## Descripción y Propósito

Este repositorio contiene el código fuente completo para la operación del robot en la competencia WRO 2026. El código está escrito en Python y utiliza el framework Pybricks (MicroPython) para la comunicación con el PrimeHub.

El proyecto está estructurado de manera modular para separar el control de bajo nivel del hardware (`robot.py`) de la orquestación de las misiones específicas en la pista (`app.py`). El enfoque principal del equipo es la estabilidad de navegación mediante algoritmos PID y un sistema de detección de color preciso y anti-rebote.

---

## Configuración de Hardware

El robot está construido sobre la plataforma LEGO Education SPIKE Prime, utilizando un PrimeHub con la siguiente distribución de puertos:

### Tracción (Chasis)
* Puerto A: Motor Izquierdo
* Puerto B: Motor Derecho

### Mecanismos
* Puerto C: Eje Central / Garra Trasera
* Puerto F: Garra Delantera / Pala

### Sensores
* Puerto D: Sensor de Color Principal (Frontal)
* Puerto E: Sensor de Color Trasero

> **Nota técnica sobre potencia:** Las rutinas de las garras que utilizan topes mecánicos (funciones `run_until_stalled`) deben ejecutarse con potencia y velocidad al máximo para vencer la resistencia de los hules tensores e inercia del mecanismo.

---

## Estado del Proyecto y Misiones

Se ha estandarizado la recolección de bloques utilizando un flujo directo con el eje central, eliminando secuencias complejas. El estado actual de las tareas es el siguiente:

### Funcionalidades Implementadas
* Construcción Inicial: [OK] Despliegue estable del bloque de cemento y la llana.
* Bloques Blancos: [OK] Rutina de recolección unificada completada.
* Mosaico: [OK] Escaneo exitoso y detección de combinaciones de colores.
* Bloques Amarillos y Azules: [OK] Recolección y reubicación operativa.
* Armado de Mosaico: [Parcial] Soporte inicial (Caso 1: verde-verde, 4/12 bloques).

### Limitaciones Conocidas / Pendientes
* Llevada de la pala: Está pendiente desarrollar una mejor solución para el transporte de la pala en el inicio, debido a la ausencia de los brazos frontales con los que se agarraba en iteraciones anteriores del diseño.

---

## Lógica de Control y Mejoras Técnicas

El núcleo del robot (`robot.py`) ha sido optimizado con las siguientes funciones personalizadas para superar las limitaciones del hardware estándar:

* **Detección de Color Avanzada (HSV):** Se migró a un sistema de detección basado en valores HSV (`detectar_color_preciso`) con lógica anti-rebote (*debouncing*). Esto elimina lecturas inconsistentes (especialmente en los bordes de la línea negra/blanca) y hace al robot mucho más predecible.
* **Seguidores de Línea PID:** Implementación de múltiples seguidores de línea (por distancia o por color) para máxima estabilidad. Incluye una función de frenado progresivo (`seguidor_linea_distancia_desacelerado`) para lograr detenciones milimétricas.
* **Navegación en Intersecciones:** El sistema ahora utiliza un sistema de doble sensor (`seguir_hasta_interseccion`) para detectar cruces de forma exacta, minimizando errores de conteo de intersecciones.
* **Manejo de Fricción (Sacudidas):** Implementación de inyección de voltaje directo (`dc`) con las funciones `sacudir` y `empuje_repetitivo` para asentar bloques pesados venciendo la fricción sin causar un estancamiento del código.

---

## Ejecución y Desarrollo

El repositorio está configurado para un flujo de trabajo ágil utilizando Visual Studio Code y la extensión `pybricksdev`.

1.  Enciende el PrimeHub y asegúrate de que el Bluetooth esté activo.
2.  Abre el repositorio en VS Code.
3.  Utiliza la configuración incluida en `.vscode/launch.json` ejecutando la tarea "Python Debugger: Module" (F5). Esto compilará y enviará el script principal vía Bluetooth.
4.  El punto de entrada es `app.py`.

*Tip para pruebas:* Para medir el rendimiento en pista de forma objetiva, puedes utilizar la función `ejecutar_y_medir_tiempo()` incluida en `app.py`, la cual cronometrará la ronda completa en segundos automáticamente.

---

## Contributors

<a href="https://github.com/robertofabiot/WRO---2026/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=robertofabiot/WRO---2026" />
</a>
