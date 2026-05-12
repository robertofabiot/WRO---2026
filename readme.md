<div align="center">

# WRO 2026 - Pybricks Robot Codebase
Lógica de control, navegación avanzada y rutinas para la World Robot Olympiad.

![Language: MicroPython](https://img.shields.io/badge/language-MicroPython-306998)
![FrameWork: Pybricks](https://img.shields.io/badge/framework-Pybricks-ED1C24)
![Status: Pending Testing](https://img.shields.io/badge/status-Pending_Testing-yellow)
![License: MIT](https://img.shields.io/badge/license-MIT-blue)

</div>

## Descripción y Propósito

Este repositorio contiene el código fuente completo para la operación del robot en la competencia WRO 2026. El código está escrito en Python y utiliza el framework Pybricks (MicroPython) para la comunicación con el PrimeHub.

El proyecto está estructurado de manera modular para separar el control de bajo nivel del hardware (`robot.py`) de la orquestación de las misiones específicas en la pista (`app.py`). El enfoque principal del equipo es la estabilidad de navegación mediante algoritmos PID y un sistema de detección de color preciso y anti-rebote.

---

## Configuración de Hardware

El robot está construido sobre la plataforma LEGO Education SPIKE Prime, utilizando un PrimeHub con la siguiente distribución de puertos actualizada (5 motores):

### Tracción (Chasis)
* Puerto B: Motor Izquierdo
* Puerto E: Motor Derecho

### Mecanismos
* Puerto A: Pinza Delantera (Abre/Cierra garra)
* Puerto C: Elevador / Garra Delantera (Sube/Baja estructura)
* Puerto F: Garra Trasera (Sube/Baja jaula)

### Sensores
* Puerto D: Sensor de Color Principal (Frontal)

> **Nota técnica sobre potencia:** Las rutinas de las garras que utilizan topes mecánicos (funciones `run_until_stalled`) deben ejecutarse con potencia y velocidad al máximo para vencer la resistencia de los hules tensores e inercia del mecanismo.

---

## Estado del Proyecto y Misiones

Se ha realizado una refactorización completa de la arquitectura orientada a objetos para simplificar la lógica de las garras y la navegación con un solo sensor. 

**Estado actual:** Todas las misiones han sido programadas bajo el nuevo flujo de trabajo, pero se encuentran **pendientes de calibración física y testeo en pista**.

### Checklist de Misiones (app.py)
- [ ] `agarrar_bloques_blancos()`
- [ ] `dejar_bloques_blancos()`
- [ ] `detectar_mosaico()`
- [ ] `agarrar_bloques_verdes()`
- [ ] `dejar_bloques_verdes()`
- [ ] `agarrar_bloques_amarillos()`
- [ ] `dejar_bloques_amarillos()`
- [ ] `cemento_y_llana()`
- [ ] `recoger_bloques_azules()`
- [ ] `dejar_bloques_azules_y_pala()`

---

## Lógica de Control y Mejoras Técnicas

El núcleo del robot (`robot.py`) ha sido optimizado con las siguientes funciones personalizadas para superar las limitaciones del hardware estándar:

* **Detección de Color Avanzada (HSV):** Se migró a un sistema de detección basado en valores HSV (`detectar_color_preciso`) con lógica anti-rebote (*debouncing*). Esto elimina lecturas inconsistentes (especialmente en los bordes de la línea negra/blanca) y hace al robot mucho más predecible.
* **Seguidores de Línea PID:** Implementación de múltiples seguidores de línea (por distancia o por color) para máxima estabilidad. Incluye una función de frenado progresivo (`seguidor_linea_distancia_desacelerado`) para lograr detenciones milimétricas.
* **Navegación y Detección de Cruces:** El sistema utiliza un modelo optimizado de seguimiento mediante un solo sensor frontal para la detección de intersecciones y finalización de tramos.
* **Manejo de Fricción (Sacudidas):** Implementación de inyección de voltaje directo (`dc`) con las funciones `sacudir` y `latigazo` para asentar o aventar bloques pesados venciendo la fricción sin causar un estancamiento del código.

---

## Ejecución y Desarrollo

El repositorio está configurado para un flujo de trabajo ágil utilizando Visual Studio Code y la extensión `pybricksdev`.

1.  Enciende el PrimeHub y asegúrate de que el Bluetooth esté activo.
2.  Abre el repositorio en VS Code.
3.  Utiliza la configuración incluida en `.vscode/launch.json` ejecutando la tarea "Python Debugger: Module" (F5). Esto compilará y enviará el script principal vía Bluetooth.
4.  El punto de entrada es `app.py`.

---

## Contributors

<a href="https://github.com/robertofabiot/WRO---2026/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=robertofabiot/WRO---2026" />
</a>