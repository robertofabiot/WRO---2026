from pybricks.parameters import Stop
from pybricks.tools import wait

class Garra:
    """Mecanismo de un solo motor. Es la base de las garras del robot."""

    def __init__(self, motor):
        """
        Argumentos:
            motor: motor del mecanismo.
        """
        self.motor = motor

    def mover(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Mueve el mecanismo una cantidad de grados relativa a donde esta.

        Argumentos:
            grados: giro del motor, con signo.
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la
                meta, para solapar el movimiento con lo que viene despues.
                0 espera el movimiento completo.
        """
        if wait_after and margen_grados > 0:
            angulo_meta = self.motor.angle() + grados
            self.motor.run_angle(velocidad, grados, then=frenado, wait=False)
            while abs(angulo_meta - self.motor.angle()) > margen_grados:
                if self.motor.stalled(): break
                wait(2)
        else:
            self.motor.run_angle(velocidad, grados, then=frenado, wait=wait_after)

    def llevar_al_tope(self, direccion, velocidad=1000, limite_potencia=60, frenado=Stop.HOLD):
        """Empuja el mecanismo hasta que se traba contra su tope fisico.

        Siempre bloquea: run_until_stalled() de Pybricks no admite wait=False.

        Argumentos:
            direccion: "positivo" o "negativo", el sentido en el que buscar
                el tope.
            velocidad: grados/s.
            limite_potencia: tope de duty-cycle, 0-100. Cuanto mas bajo, mas
                suave llega al tope y menos castiga los engranajes.
            frenado: modo de frenado de Pybricks al trabarse.

        Devuelve el angulo en el que quedo trabado el motor.
        """
        if direccion == "positivo":
            vel_real = abs(velocidad)
        elif direccion == "negativo":
            vel_real = -abs(velocidad)
        else:
            raise ValueError("direccion tiene que ser 'positivo' o 'negativo'")
        return self.motor.run_until_stalled(vel_real, then=frenado, duty_limit=limite_potencia)

    def subir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Sube esa cantidad de grados.

        Argumentos:
            grados: angulo a subir en grados (positivo).
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la
                meta. 0 espera el movimiento completo.
        """
        self.mover(-abs(grados), velocidad, wait_after, frenado, margen_grados)

    def bajar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Baja esa cantidad de grados.

        Argumentos:
            grados: angulo a bajar en grados (positivo).
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la
                meta. 0 espera el movimiento completo.
        """
        self.mover(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def subir_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        """Sube hasta el tope fisico.

        Argumentos:
            velocidad: grados/s.
            limite_potencia: tope de duty-cycle (0-100).
            frenado: modo de frenado de Pybricks al trabarse.

        Devuelve el angulo en el que quedo trabado el motor.
        """
        return self.llevar_al_tope("negativo", velocidad, limite_potencia, frenado)

    def bajar_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        """Baja hasta el tope fisico.

        Argumentos:
            velocidad: grados/s.
            limite_potencia: tope de duty-cycle (0-100).
            frenado: modo de frenado de Pybricks al trabarse.

        Devuelve el angulo en el que quedo trabado el motor.
        """
        return self.llevar_al_tope("positivo", velocidad, limite_potencia, frenado)

class GarraDelantera(Garra):
    """Elevador con pinza: dos motores independientes.

    Los metodos heredados de Garra mueven el elevador; los que llevan 'pinza'
    en el nombre, o abrir/cerrar, mueven la pinza.
    """

    def __init__(self, motor_elevador, motor_pinza, rango_maximo_grados=None, rango_maximo_pinza=None):
        """
        Argumentos:
            motor_elevador: motor que sube y baja la garra.
            motor_pinza: motor que abre y cierra la pinza.
            rango_maximo_grados: grados del elevador entre su cero y el 100%,
                calibrados en config.
            rango_maximo_pinza: idem para la pinza.
        """
        super().__init__(motor_elevador)
        self.rango_maximo = rango_maximo_grados
        self.pinza = Garra(motor_pinza)
        self.pinza.rango_maximo = rango_maximo_pinza

    def mover_pinza(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Mueve la pinza una cantidad de grados relativa.

        Argumentos:
            grados: giro del motor de la pinza en grados, con signo.
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la meta.
        """
        self.pinza.mover(grados, velocidad, wait_after, frenado, margen_grados)

    def abrir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Abre la pinza esa cantidad de grados.

        Argumentos:
            grados: angulo de apertura en grados (positivo).
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la meta.
        """
        self.mover_pinza(-abs(grados), velocidad, wait_after, frenado, margen_grados)

    def cerrar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Cierra la pinza esa cantidad de grados.

        Argumentos:
            grados: angulo de cierre en grados (positivo).
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la meta.
        """
        self.mover_pinza(abs(grados), velocidad, wait_after, frenado, margen_grados)

    def abrir_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        """Abre la pinza hasta su tope fisico.

        Argumentos:
            velocidad: grados/s.
            limite_potencia: tope de duty-cycle (0-100).
            frenado: modo de frenado de Pybricks al trabarse.

        Devuelve el angulo en el que quedo trabado el motor.
        """
        return self.pinza.llevar_al_tope("negativo", velocidad, limite_potencia, frenado)

    def cerrar_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        """Cierra la pinza hasta trabarse, tipicamente contra la pieza.

        Argumentos:
            velocidad: grados/s.
            limite_potencia: tope de duty-cycle (0-100) para regular el agarre.
            frenado: modo de frenado de Pybricks al trabarse.

        Devuelve el angulo en el que quedo trabado el motor.
        """
        return self.pinza.llevar_al_tope("positivo", velocidad, limite_potencia, frenado)

    def establecer_cero(self, velocidad=800, limite_potencia=50):
        """Lleva el elevador a su tope superior y fija ahi el cero absoluto.

        Hay que correrlo antes de usar ir_a_porcentaje(), que trabaja contra
        ese cero.

        Argumentos:
            velocidad: grados/s de la busqueda del tope.
            limite_potencia: tope de duty-cycle, 0-100.
        """
        self.subir_al_tope(velocidad=velocidad, limite_potencia=limite_potencia)
        self.motor.reset_angle(0)

    def ir_a_porcentaje(self, porcentaje, velocidad=800, wait_after=True, frenado=Stop.HOLD):
        """Lleva el elevador a una posicion absoluta de su rango calibrado.

        Va al angulo exacto sin importar donde este, asi que no acumula error
        como los movimientos relativos.

        Argumentos:
            porcentaje: 0 es arriba (el cero), 100 es abajo. Se acota al rango.
            velocidad: grados/s.
            wait_after: True bloquea hasta llegar; False deja el mecanismo
                moviendose mientras el chasis sigue su recorrido.
            frenado: modo de frenado de Pybricks al llegar.
        """
        if self.rango_maximo is None:
            raise ValueError("Debes configurar el rango_maximo_grados al instanciar la garra delantera")
            
        porcentaje_seguro = max(0.0, min(100.0, float(porcentaje)))
        angulo_objetivo = (porcentaje_seguro / 100.0) * self.rango_maximo
        
        self.motor.run_target(
            speed=velocidad, 
            target_angle=angulo_objetivo, 
            then=frenado, 
            wait=wait_after
        )

    def establecer_cero_pinza(self, velocidad=800, limite_potencia=50):
        """Abre la pinza hasta su tope fisico y fija ahi el cero absoluto.

        Argumentos:
            velocidad: grados/s de la busqueda del tope.
            limite_potencia: tope de duty-cycle, 0-100.
        """
        self.abrir_al_tope(velocidad=velocidad, limite_potencia=limite_potencia)
        self.pinza.motor.reset_angle(0)

    def ir_a_porcentaje_pinza(self, porcentaje, velocidad=800, wait_after=True, frenado=Stop.HOLD):
        """Lleva la pinza a una posicion absoluta de su rango calibrado.

        Argumentos:
            porcentaje: 0 es la posicion del cero (abierta al tope), 100 es
                cerrada al maximo. Se acota al rango.
            velocidad: grados/s.
            wait_after: True bloquea hasta llegar.
            frenado: modo de frenado de Pybricks al llegar.
        """
        if self.pinza.rango_maximo is None:
            raise ValueError("Debes configurar el rango_maximo_pinza al instanciar la garra delantera")
            
        porcentaje_seguro = max(0.0, min(100.0, float(porcentaje)))
        angulo_objetivo = (porcentaje_seguro / 100.0) * self.pinza.rango_maximo
        
        self.pinza.motor.run_target(
            speed=velocidad, 
            target_angle=angulo_objetivo, 
            then=frenado, 
            wait=wait_after
        )

class GarraTrasera(Garra):
    """Jaula vertical trasera.

    El motor esta montado invertido, asi que los metodos al tope llaman a los
    opuestos de Garra para que subir siga significando subir.
    """

    def __init__(self, motor, rango_maximo_grados=None):
        """
        Argumentos:
            motor: motor de la jaula.
            rango_maximo_grados: grados entre el cero y el 100%, calibrados en
                config. Es negativo porque el motor esta invertido.
        """
        super().__init__(motor)
        self.rango_maximo = rango_maximo_grados

    def establecer_cero(self, velocidad=800, limite_potencia=50):
        """Sube la jaula a su tope y fija ahi el cero absoluto.

        Hay que correrlo antes de usar ir_a_porcentaje().

        Argumentos:
            velocidad: grados/s de la busqueda del tope.
            limite_potencia: tope de duty-cycle, 0-100.
        """
        self.subir_al_tope(velocidad=velocidad, limite_potencia=limite_potencia)
        self.motor.reset_angle(0)

    def ir_a_porcentaje(self, porcentaje, velocidad=800, wait_after=True, frenado=Stop.HOLD):
        """Lleva la jaula a una posicion absoluta de su rango calibrado.

        Argumentos:
            porcentaje: 0 es arriba y guardada, 100 es abajo rozando el piso.
                Se acota al rango.
            velocidad: grados/s.
            wait_after: True bloquea hasta llegar; False deja la jaula bajando
                mientras el chasis sigue su recorrido.
            frenado: modo de frenado de Pybricks al llegar.
        """
        if self.rango_maximo is None:
            raise ValueError("Debes configurar el rango_maximo_grados al instanciar la garra")
            
        porcentaje_seguro = max(0.0, min(100.0, float(porcentaje)))
        angulo_objetivo = (porcentaje_seguro / 100.0) * self.rango_maximo
        
        self.motor.run_target(
            speed=velocidad, 
            target_angle=angulo_objetivo, 
            then=frenado, 
            wait=wait_after
        )

    def subir(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Sube la jaula esa cantidad de grados.

        Argumentos:
            grados: angulo a subir en grados (positivo).
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la meta.
        """
        super().bajar(grados, velocidad, wait_after, frenado, margen_grados)

    def bajar(self, grados, velocidad=600, wait_after=True, frenado=Stop.HOLD, margen_grados=0):
        """Baja la jaula esa cantidad de grados.

        Argumentos:
            grados: angulo a bajar en grados (positivo).
            velocidad: grados/s.
            wait_after: True bloquea hasta terminar el movimiento.
            frenado: modo de frenado de Pybricks al llegar.
            margen_grados: corta la espera esa cantidad de grados antes de la meta.
        """
        super().subir(grados, velocidad, wait_after, frenado, margen_grados)

    def subir_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        """Sube la jaula hasta su tope fisico.

        Argumentos:
            velocidad: grados/s.
            limite_potencia: tope de duty-cycle (0-100).
            frenado: modo de frenado de Pybricks al trabarse.

        Devuelve el angulo en el que quedo trabado el motor.
        """
        return super().bajar_al_tope(velocidad, limite_potencia, frenado)

    def bajar_al_tope(self, velocidad=800, limite_potencia=50, frenado=Stop.HOLD):
        """Baja la jaula hasta su tope fisico.

        Argumentos:
            velocidad: grados/s.
            limite_potencia: tope de duty-cycle (0-100).
            frenado: modo de frenado de Pybricks al trabarse.

        Devuelve el angulo en el que quedo trabado el motor.
        """
        return super().subir_al_tope(velocidad, limite_potencia, frenado)
