from pybricks.parameters import Port, Color

# --- HARDWARE CONFIG (Puertos) ---
PORT_MOTOR_IZQ = Port.B
PORT_MOTOR_DER = Port.E
PORT_GARRA_TRASERA = Port.F
PORT_GARRA_DELANTERA = Port.C
PORT_PINZA = Port.A
PORT_SENSOR_FRENTE = Port.D

# --- CHASIS CONFIG (Medidas físicas) ---
DIAMETRO_RUEDA = 56
SEPARACION_RUEDAS = 160

# --- NAVEGACIÓN CONFIG (Velocidades) ---
VELOCIDAD_BASE = 950
STRAIGHT_SPEED = 700
STRAIGHT_ACCEL = 700
TURN_RATE = 500

# --- REGLAS DEL JUEGO ---
MOSAICOS = {
    Color.GREEN: {Color.GREEN: 1, Color.YELLOW: 2}, 
    Color.BLUE: 3, 
    Color.YELLOW: 4, 
    Color.WHITE: 5
}