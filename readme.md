<div align="center">

# WRO 2026 - Pybricks Robot Codebase
Lógica de control, navegación avanzada y rutinas para la World Robot Olympiad.

![Language: MicroPython](https://img.shields.io/badge/language-MicroPython-306998)
![FrameWork: Pybricks](https://img.shields.io/badge/framework-Pybricks-ED1C24)
![Status: In Development](https://img.shields.io/badge/status-In_Development-orange)
![License: MIT](https://img.shields.io/badge/license-MIT-blue)
![Branch: CharlieSexPath](https://img.shields.io/badge/branch-CharlieSexPath-8A2BE2)

</div>

## Descripción y Propósito

Este repositorio contiene el código fuente completo para la operación del robot en la competencia WRO 2026. El código está escrito en Python y utiliza el framework Pybricks (MicroPython) para la comunicación con el PrimeHub.

El proyecto está estructurado de manera modular para separar el control de bajo nivel del hardware (`robot.py`), la configuración de constantes (`config.py`), y la orquestación de las misiones específicas en la pista (`app.py`, `Misiones.py`). El enfoque principal del equipo es la estabilidad de navegación mediante algoritmos PID, encadenamiento cinético para evitar latencias, y un sistema de detección de color de alta precisión.

---

## Configuración de Hardware

El robot está construido sobre la plataforma LEGO Education SPIKE Prime, utilizando un PrimeHub con la siguiente distribución exacta de puertos (basada en `config.py`):

### Tracción (Chasis)
* **Puerto B**: Motor Izquierdo
* **Puerto E**: Motor Derecho
*(Medidas: Diámetro de rueda 56mm, Separación 160mm)*

### Mecanismos
* **Puerto C**: Elevador Delantero
* **Puerto A**: Garra Delantera
* **Puerto F**: Garra Trasera

### Sensores
* **Puerto D**: Sensor de Color Principal (Frontal)

> **Nota técnica sobre potencia y control:** Las rutinas de las garras y elevadores que utilizan topes mecánicos (funciones `llevar_al_tope`) deben ejecutarse con límites de potencia altos para vencer la inercia. Además, se han integrado rutinas de voltaje directo (`dc`) y "sacudidas" para acomodar bloques superando la fricción estática del tapete.

---

## Estado del Proyecto y Misiones

Se ha optimizado drásticamente la fluidez de las misiones implementando concurrencia (movimiento de mecanismos simultáneo al chasis) y eliminando tiempos de espera rígidos (`wait()`). 

### Funcionalidades Implementadas
* **Construcción Inicial:** [OK] Rutina de cemento y llana con arcos encadenados y giro rápido.
* **Bloques Blancos:** [OK] Rutina de recolección unificada completada con cuadratura final para alineación perfecta y hack de aceleración en el giro de 180°.
* **Mosaico:** [OK] Escaneo de intersecciones, detección de combinaciones de colores (integrado con el sensor frontal) y uso de pivotes anclados (`Stop.HOLD`) para precisión milimétrica.
* **Bloques Amarillos y Azules:** [OK] Rutinas de recolección y reubicación completamente operativas.
* **Manejo de la Pala:** [OK] Rutina de reubicación y entrega final de bloques azules y pala completada.

### Armado de Mosaico (ArmadorMosaicos)
* **Caso 1 (Verde-Verde):** [OK] Rutina compleja implementada con sacudidas para asentar bloques.
* **Casos 2, 3, 4 y 5:** [Pendiente] En fase de desarrollo.

---

## Lógica de Control y Mejoras Técnicas

El núcleo del robot ha sido fuertemente refactorizado para superar las limitaciones del hardware estándar y el "tartamudeo cinético":

* **Detección de Color Avanzada (HSV):** Detección basada en valores HSV (`detectar_color_preciso`) con umbrales optimizados para diferenciar colores de competencia eliminando lecturas inconsistentes en los bordes.
* **Seguidores de Línea PID de Alta Frecuencia:** Implementación de múltiples seguidores de línea (distancia, color, desacelerado, y con **cuadratura final** para alineación perfecta perpendicular a la línea). 
* **Control de Giro Agresivo:** Uso de control por voltaje directo (`giro_eje_puro`) y manipulación temporal de la configuración del `DriveBase` en C nativo para giros súper rápidos que vencen la inercia sin sobreoscilar.
* **Encadenamiento Cinético:** Uso estratégico de `Stop.NONE`, `Stop.COAST` y márgenes de distancia para fluir entre movimientos sin frenar el chasis a cero.
* **Compensación de Voltaje:** Función `compensar_voltaje` para ajustar dinámicamente la potencia de los motores dependiendo de la carga de la batería.

---

## Ejecución y Desarrollo

El repositorio está configurado para un flujo de trabajo ágil utilizando Visual Studio Code y la extensión `pybricksdev`.

1.  Enciende el PrimeHub y asegúrate de que el Bluetooth esté activo.
2.  Abre el repositorio en VS Code.
3.  Utiliza la configuración incluida en `.vscode/launch.json` ejecutando la tarea "Python Debugger: Module" (F5). Esto compilará y enviará el script principal vía Bluetooth.
4.  El punto de entrada principal del robot es `app.py`.

---

## Contributors

<a href="https://github.com/robertofabiot/WRO---2026/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=robertofabiot/WRO---2026" />
</a>
