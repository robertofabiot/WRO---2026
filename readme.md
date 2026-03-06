## Estado del proyecto

Actualmente, el enfoque principal es una función de seguidor de línea que recibe como entrada una distancia en centímetros con una precisión aceptable. Se mejoró muchísimo la optimización del mismo para eliminar los tambaleos.

Aún está pendiente:
- Reorganizar el código en funciones para mejorar su claridad y mantenimiento.
- Mejorar la función de bloques blancos para que sea más estable.

## Limitaciones conocidas

La principal limitación del sistema es que no se ha encontrado una solución para llevar a cabo de manera estable el recorrido de los bloques blancos.

## Funcionalidades implementadas

El código actual incluye solución para:
- Bloque de cemento.
- Llana.
- Bloques blancos (parcialmente)