# WRO 2026 - Código Migrado y Modularizado

Repositorio migrado del proyecto del equipo amigo hacia la arquitectura limpia y modular de **WRO---2026 - MIO**, incorporando la versión oficial actualizada del recorrido (**`prueba_reto_1`**).

---

## 📌 Propósito de la Migración

El código original del equipo (`WRO_2026_Robotica - OTRO EQUIPO / prueba_reto_1`) contaba con el recorrido afinado y calibrado, pero concentrado de forma secuencial y dependiente de una clase base con funciones sueltas.

Esta versión refactorizada (`WRO---2026 - MIGRADO`):
1. **Preserva al 100% el comportamiento físico de `prueba_reto_1`:** Mantiene intactas las distancias, ángulos, curvas de aceleración, ganancias PID de seguidor y giros, tiempos de estabilización, filtros de color HSV y llamadas a `gc.collect()`.
2. **Estructura Orientada a Objetos:** Separa limpiamente responsabilidades entre subsistemas (`Robot`, `Chasis`, `Navegacion`, `Mecanismos`, `Misiones`, `ArmadorMosaicos`).
3. **Configuración Centralizada:** Elimina números mágicos dispersos mediante `config.py`.
4. **Control por Porcentaje 100% Integrado:** Todos los mecanismos y garras han sido migrados a `ir_a_porcentaje()` empleando los rangos máximos físicos calibrados en el robot real.

---

## 📂 Estructura del Código

| Archivo | Responsabilidad |
|---|---|
| [`app.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/app.py) | Punto de entrada principal para pruebas individuales de misiones o la corrida completa de `prueba_reto_1`. |
| [`config.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/config.py) | Puertos de hardware, dimensiones del chasis, constantes PID, librerías de color HSV y rangos máximos calibrados. |
| [`robot.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/robot.py) | Único lugar donde se instancian `PrimeHub`, motores y sensores; cablea todos los subsistemas. |
| [`Chasis.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/Chasis.py) | Tracción directa, frenado, reset de encoders, `avanzar_recto` y `avanzar_con_torque` coordinado con porcentaje. |
| [`Navegacion.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/Navegacion.py) | Giros IMU (`girar`, `giro_de_arco`, `girar_a_rumbo`, `girar_corto`), seguidores de línea y `seguir_linea_y_mover_torque`. |
| [`Mecanismos.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/Mecanismos.py) | Clases `MecanismoTorque`, `GarraDelantera` y `GarraPrincipal` con soporte para porcentaje (`ir_a_porcentaje`) y apriete. |
| [`Misiones.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/Misiones.py) | Recorrido de `prueba_reto_1` desglosado en 6 secciones modulares con todos los movimientos de garras activos por porcentaje. |
| [`ArmadorMosaicos.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/ArmadorMosaicos.py) | Lógica de lectura estática, escaneo de matriz y rutinas de resolución de matrices 1 a 4 con garras activas por porcentaje. |
| [`Utils.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/Utils.py) | Operaciones matemáticas auxiliares (`limitar`, `error_angular`, sonidos). |
| [`RevisadorBateria.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/RevisadorBateria.py) | Verificación previa de voltaje con alertas audibles y confirmación interactiva. |
| [`odd_shit/`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/odd_shit/) | Herramientas de calibración y diagnóstico (`medir_limites.py`, `calibrador_mecanismos.py`). |

---

## 🗺️ Secciones del Recorrido (`Misiones.py` basado en `prueba_reto_1`)

1. **`seccion_1_salida_y_cemento`**: Reset de encoders a 0°, giro en arco de 22 cm a 90°, seguidor de 79 cm, bajada de torque al 93.0% (-169°), avance y giro a 70° para empujar la llana (-32 cm), cruce de 2 líneas y posicionamiento con arco de 19°.
2. **`seccion_2_dejar_cemento_y_tomar_verdes`**: Seguidor de 50 cm, giro -90°, avance coordinado de 39.5 cm subiendo el torque a 0% a los 13 cm, giro 90°, seguidor a verde, reversa -7 cm, giro -188° y bajada de torque a 93.4% (-170° a -22 cm) para tomar verdes.
3. **`seccion_3_escanear_matriz_y_dejar_verdes`**: Seguidor a verde de matriz, acomodo corto, avance 14 cm, lectura de color (`escanear_matriz`), salida (-37.5 cm, giro -180°), depósito de verdes (-19.6 cm y subida de torque a 0%). Devuelve `matriz_detectada`.
4. **`seccion_4_amarillos_y_azules`**: Seguidor de 48 cm, maniobra por pasillo (-90°, 23 cm, -90°, 15 cm, -93.5°) para tomar amarillos, cruce de 2 líneas, giro 89° y toma de azules con torque a 93.4% (-170° a -22 cm).
5. **`seccion_5_tomar_pala_y_dejar_amarillos`**: Avance 20 cm, giro -38°, posicionamiento de garra delantera a 69.4% (245°) y garra principal a 47.6% (200°), cruce de 2 líneas, cierre de pinza a 0%, avance 48 cm, apertura al 71.4% (300°) para soltar amarillos, repliegue de garra delantera a 0%, alineación a 90°, avance híbrido al amarillo y salida de zona (-22.5 cm, giro -92°).
6. **`seccion_6_retorno_pala_y_fin`**: Preparación de pinza (47.6%) y garra delantera (69.4%), reversa -2 cm, seguidor de 85 cm con activación de torque (subida a 0%) a los 65 cm para colocar la pala, alineación de norte, repliegue de garras a 0% y resolución de la matriz identificada.

---

## 🔌 Configuración de Hardware y Límites Calibrados (Robot del Usuario)

* **Tracción:**
  * Motor Izquierdo: `Port.B` (`Direction.COUNTERCLOCKWISE`)
  * Motor Derecho: `Port.E` (`Direction.CLOCKWISE`)
  * Diámetro de rueda: `56 mm`
  * Ancho de eje (*axle track*): `160 mm`
* **Mecanismos:**
  * Motor de Torque / Jaula Trasera: `Port.F` (`Direction.COUNTERCLOCKWISE`) -> `RANGO_MAXIMO_TORQUE = -179` (0% = arriba, 100% = abajo al tope)
  * Garra Delantera / Elevador: `Port.C` -> `RANGO_MAXIMO_GARRA_DELANTERA = 672` (0% = arriba, 100% = abajo al tope)
  * Garra Principal / Pinza: `Port.A` -> `RANGO_MAXIMO_GARRA_PRINCIPAL = 798` (0% = cerrada, 100% = abierta al tope)
* **Sensores:**
  * Sensor de Color / Seguidor: `Port.D`

---

## 🚀 Para Ejecutar las Pruebas

1. Conecta el PrimeHub mediante Bluetooth o USB.
2. Abre el espacio de trabajo en VS Code con la extensión de Pybricks (`pybricksdev`).
3. Abre [`app.py`](file:///C:/Users/rfter/Desktop/repos%20robotica/WRO---2026%20-%20MIGRADO/app.py).
4. Descomenta la sección que desees probar y ejecuta con F5 o el botón de ejecución de Pybricks.
