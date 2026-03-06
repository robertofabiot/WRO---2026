## Estado del proyecto

Actualmente, el enfoque principal es una función de seguidor de línea que recibe como entrada una distancia en centímetros con una precisión aceptable. Se mejoró muchísimo la optimización del mismo para eliminar los tambaleos.

Se agregó una función para bajar la garra en caso de que esté trabada. La logica es que si no se detecta movimiento en el motor, el robot avanza un poco para destrabarla (asumiendo que es porque está trabada contra la pared de la mesa)

Aún está pendiente:
- Reorganizar el código en funciones para mejorar su claridad y mantenimiento.

## Limitaciones conocidas

-El giro al entregar los bloques blancos no se logra hacer sin pasarse llevando la torre y el borde del mosaico, lo que afecta su entrega y por ende los movimientos posteriores.

## Funcionalidades implementadas

El código actual incluye solución para:
- Bloque de cemento.
- Llana.
- Bloques blancos