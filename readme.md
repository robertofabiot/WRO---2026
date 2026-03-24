### Estado del proyecto

La función principal es un seguidor de línea calibrado (por distancia o color) para máxima estabilidad. Recientemente se migró a un sistema de detección de color basado en HSV con lógica anti-rebote (debouncing), eliminando lecturas inconsistentes y haciendo el robot mucho más predecible en la pista.

### Actualización de Mecanismos (Garras)

El hardware está dividido en dos mecanismos principales:
* **Garra Delantera:** Puerto F.
* **Garra Trasera / Eje Central:** Puerto C.

*Nota importante sobre la potencia:* Las rutinas de las garras que usan topes mecánicos (`run_until_stalled`) **deben ejecutarse con potencia y velocidad al máximo** para poder vencer la inercia y la resistencia de los hules tensores.

### Lógica de las nuevas funciones

Se integraron nuevas rutinas para mejorar el control fino y la navegación del robot:
* `seguir_hasta_interseccion`: Utiliza un sistema de doble sensor para detectar cruces de forma más precisa.
* `seguidor_linea_distancia_desacelerado`: Aplica un frenado progresivo para lograr detenciones exactas.
* **Recolección unificada (Lógica):** Se reemplazaron las secuencias complejas por un flujo directo usando el eje central, estandarizando la recogida de bloques blancos, amarillos y azules.

### Limitaciones conocidas

* **Problema de llevada de la pala:** Está pendiente desarrollar una mejor solución para la pala, en ausencia de los brazos frontales con los que se agarraba antes.

### Funcionalidades implementadas

El código actual incluye solución estable para:
* Bloque de cemento.
* Llana.
* Bloques blancos.
* Detectar el mosaico.
* Bloques amarillos.
* Bloques azules.
* Pala. **(Pend)**
* Armar mosaico (Caso 1: 4/12 bloques).
