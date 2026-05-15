<div align="center">

# WRO 2026 - Pybricks Robot Codebase
*Pruebas de Navegación Absoluta y Refactorización*

![Status: Experimental](https://img.shields.io/badge/status-Experimental-yellow)
![FrameWork: Pybricks](https://img.shields.io/badge/framework-Pybricks-ED1C24)

</div>

## Propósito de esta Versión

El objetivo de esta versión del código es probar e implementar la **Navegación por Coordenadas Absolutas (Absolute Heading)**. A diferencia del código base que utiliza giros relativos que acumulan error (*drift*), esta variante permite que el robot mantenga un "mapa mental" constante de la pista.

**Nuevas características clave:**
* **Cuadratura Física:** Nuevo método para chocar suavemente contra las paredes y reiniciar la orientación del giroscopio a un punto cardinal fijo (ej. 0°).
* **Giros Absolutos (`giro_absoluto_pd`):** Capacidad para girar hacia un ángulo específico del mapa usando la ruta más corta (o forzar la ruta larga para esquivar), asegurando consistencia en recorridos largos.
* **Giros Relativos Seguros:** La función `giro_preciso` fue modificada para sumar o restar grados a la trayectoria actual sin reiniciar la brújula global del IMU.

---

## Configuración de Hardware (5 Motores)

* **Tracción:** Izquierdo (B) | Derecho (E)
* **Mecanismos:** Pinza Delantera (A) | Elevador Delantero (C) | Garra Trasera (F)
* **Sensores:** Color Frontal (D)

> *Nota:* Las rutinas de las garras (`run_until_stalled`) requieren la máxima potencia de los motores para vencer la inercia y los hules tensores.

---

## Estado Actual y Ejecución

La arquitectura de las misiones (`app.py`) ha sido adaptada a este nuevo flujo de trabajo y a la nueva estructura de garras. Actualmente, las misiones se encuentran **pendientes de calibración física y testeo en la pista**.

Para ejecutar las pruebas:
1. Conecta el PrimeHub vía Bluetooth.
2. Abre el proyecto en VS Code con la extensión `pybricksdev`.
3. Ejecuta la tarea (F5) apuntando al archivo `app.py`.