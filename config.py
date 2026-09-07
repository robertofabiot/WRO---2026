from pybricks.parameters import Port, Color

# --- HARDWARE CONFIG (Puertos) ---
PORT_MOTOR_IZQ = Port.B
PORT_MOTOR_DER = Port.E
PORT_GARRA_TRASERA = Port.F
PORT_GARRA_DELANTERA = Port.C
PORT_PINZA = Port.A
PORT_SENSOR_FRENTE = Port.D

# --- MECANISMOS CONFIG (Grados máximos calibrados) ---
RANGO_MAXIMO_GARRA_TRASERA = -179
RANGO_MAXIMO_GARRA_DELANTERA = 672
RANGO_MAXIMO_PINZA = 798

# --- CHASIS CONFIG (Medidas físicas) ---
DIAMETRO_RUEDA = 56
SEPARACION_RUEDAS = 160

# --- NAVEGACIÓN CONFIG (Velocidades) ---
# VELOCIDAD_RECTA es el techo real: cualquier pedido de velocidad se acota a
# ese valor antes de llegar al drive_base.
VELOCIDAD_BASE = 950
VELOCIDAD_RECTA = 700
ACELERACION_RECTA = 700
TASA_GIRO = 500

# --- REGLAS DEL JUEGO ---
MOSAICOS = {
    Color.GREEN: {Color.GREEN: 1, Color.YELLOW: 2}, 
    Color.BLUE: 3, 
    Color.YELLOW: 4, 
    Color.WHITE: 5
}

# --- OPCIONES DE SISTEMA ---
SONIDO_ACTIVO = False
BATERIA_MINIMA = 8150

# --- SEGURIDAD DE LOS LAZOS DE CONTROL ---
# Piso de velocidad de giro en grados/s. Por debajo de esto el motor no vence
# la friccion estatica y el lazo PD se queda pataleando sin avanzar.
PISO_VELOCIDAD_GIRO = 30

# Giros mas chicos que esto se saltan: caen dentro del ruido del IMU.
BANDA_MUERTA_GIRO = 1.5

# Cortes de emergencia. Sin esto, un color que nunca aparece cuelga el robot
# para siempre y se pierde la corrida completa.
TIMEOUT_GIRO_MS = 3000
TIMEOUT_LAZO_MS = 12000
TIMEOUT_MOVIMIENTO_MS = 8000
