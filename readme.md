## Estado del proyecto

La función principal del proyecto es un seguidor de línea correctamente calibrado, con la posibilidad de operar por distancia o por detección de colores, lo que mejora significativamente la estabilidad durante las ejecuciones.

### Actualización de Mecanismos (Garras)
En las últimas versiones, el control de las garras se ha rediseñado para ser más modular y preciso. Se ha dividido el hardware en dos mecanismos principales:
* **Garra Delantera:** Controlada a través del puerto F.
* **Garra Trasera / Eje Central:** Controlada a través del puerto C (anteriormente referida solo como "garra").

### Lógica de las nuevas funciones
Se implementaron nuevas funciones de tope mecánico (ej. `abrir_garra_delantera_al_tope`, `cerrar_garra_delantera_al_tope`, `llevar_eje_central_al_tope`). Estas funciones utilizan la instrucción `run_until_stalled` para mover los motores hasta que detectan resistencia física (el tope mecánico o al atrapar un objeto), evitando que el programa se quede esperando un ángulo que físicamente no puede alcanzar.

**Nota importante sobre la potencia:** En estas nuevas rutinas, la potencia y velocidad de los motores **deben estar siempre al máximo**. Debido a la resistencia que generan los hules tensores y el propio peso físico de las garras, si se utiliza muy poca potencia o una velocidad baja, los motores no tendrán la fuerza suficiente para vencer la inercia inicial y el mecanismo no se moverá correctamente.

## Limitaciones conocidas
Actualmente no se ha logrado recoger todos los bloques de manera consistente.
Hay que implementar una función más precisa para leer colores, especialmente por el Color.BLUE, que es devuelto al escanear negro.

## Funcionalidades implementadas

El código actual incluye solución para:
- Bloque de cemento.
- Llana.
- Bloques blancos
- Detectar el mosaico
- Bloques amarillos
- Bloques azules
- Pala
- Armar mosaico (Caso 1: 4/12 bloques)

Nota: probablemente se pueda mejorar la ejecución de las rutinas anteriores al mosaico usando la nueva función
del eje central al tope. Siento que es más estable que dar una cantidad fija de grados.