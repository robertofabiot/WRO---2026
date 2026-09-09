"""Configuración centralizada para el robot WRO 2026.

Reúne puertos, dimensiones físicas, ganancias de control, calibraciones
de color y valores límite del robot del equipo migrado.
"""

from pybricks.parameters import Port, Color, Direction

# =============================================================================
# 1. PUERTOS DE HARDWARE (Configuración del robot del usuario / MIO)
# =============================================================================
# Tracción
PORT_MOTOR_IZQ = Port.B
PORT_MOTOR_DER = Port.E

# Mecanismos y actuadores
PORT_TORQUE = Port.F                  # Jaula trasera / motor de torque
PORT_GARRA_TRASERA = PORT_TORQUE      # Alias de compatibilidad MIO
PORT_GARRA_DELANTERA = Port.C         # Elevador delantero
PORT_GARRA_PRINCIPAL = Port.A         # Pinza principal
PORT_PINZA = PORT_GARRA_PRINCIPAL     # Alias de compatibilidad MIO

# Sensores
PORT_SENSOR_COLOR = Port.D            # Sensor seguidor / frente
PORT_SENSOR_FRENTE = PORT_SENSOR_COLOR # Alias de compatibilidad MIO

# Direcciones de motores
DIRECCION_MOTOR_IZQ = Direction.COUNTERCLOCKWISE
DIRECCION_MOTOR_DER = Direction.CLOCKWISE
DIRECCION_TORQUE = Direction.COUNTERCLOCKWISE

# =============================================================================
# 2. CHASIS Y DIMENSIONES FÍSICAS (en mm y cm)
# =============================================================================
DIAMETRO_RUEDA = 56              # Diámetro de la rueda en mm
SEPARACION_RUEDAS = 160          # Axle track en mm para DriveBase (robot del usuario: 160 mm)
DISTANCIA_RUEDAS_ARCO_CM = 16.0  # Distancia entre ruedas usada en giros de arco (cm)

CIRCUNFERENCIA = DIAMETRO_RUEDA * 3.14159
GRADOS_POR_MM = 360.0 / CIRCUNFERENCIA
GRADOS_POR_CM = 360.0 / (CIRCUNFERENCIA / 10.0)

# =============================================================================
# 3. RANGOS MÁXIMOS CALIBRADOS DE MECANISMOS (Robot del usuario / MIO)
# =============================================================================
# Rangos calibrados para mapeo 0.0% a 100.0%:
RANGO_MAXIMO_TORQUE = -179          # Medido robot usuario: -179° (0% = arriba/guardado, 100% = abajo/tope)
RANGO_MAXIMO_GARRA_TRASERA = RANGO_MAXIMO_TORQUE # Alias de compatibilidad MIO

RANGO_MAXIMO_GARRA_DELANTERA = 672  # Medido robot usuario: 672° (0% = arriba, 100% = abajo al tope)

RANGO_MAXIMO_GARRA_PRINCIPAL = 798  # Medido robot usuario: 798° (0% = cerrada, 100% = abierta al tope)
RANGO_MAXIMO_PINZA = RANGO_MAXIMO_GARRA_PRINCIPAL # Alias de compatibilidad MIO

# =============================================================================
# 4. CONTROL Y NAVEGACIÓN (Valores por defecto del equipo original)
# =============================================================================
# Avance recto con corrección de IMU
KP_GYRO_RECTA = 20.0
VELOCIDAD_MAX_RECTA = 900
VELOCIDAD_MIN_RECTA = 100
ZONA_RAMPA_DEFECTO_CM = 8

# Giros sobre el eje con lazo PD
KP_GIRO = 3.5
KD_GIRO = 5.0
POTENCIA_MAX_GIRO = 85
POTENCIA_MIN_GIRO = 40
TOLERANCIA_GIRO = 1.9

# Giros cortos
KP_GIRO_CORTO = 4.0
POTENCIA_MAX_GIRO_CORTO = 45
POTENCIA_MIN_GIRO_CORTO = 24
TOLERANCIA_GIRO_CORTO = 1.8
TIMEOUT_GIRO_CORTO_MS = 500

# Giros absolutos a rumbo
KP_RUMBO = 2.6
KD_RUMBO = 4.2
POTENCIA_MAX_RUMBO = 100
POTENCIA_MED_RUMBO = 55
POTENCIA_MIN_RUMBO = 45
ZONA_MEDIA_RUMBO = 35
ZONA_PRECISA_RUMBO = 9
TOLERANCIA_RUMBO = 1.0
TIMEOUT_RUMBO_MS = 3000

# Seguidor de línea (Cerebro Predictivo PID)
KP_LINEA = 1.15
KD_LINEA = 3.8
K_FRENO_LINEA = 0.05
OBJETIVO_REFLEXION = 27
CORRECCION_MAX_LINEA = 100

# Captura inicial del seguidor de línea
CAPTURA_INICIAL_LINEA = True
TIEMPO_CAPTURA_MS = 280
POTENCIA_CAPTURA = 55
KP_CAPTURA = 3.8
MARGEN_CAPTURA = 5
LECTURAS_ESTABLES_CAPTURA = 2

# =============================================================================
# 5. CALIBRACIÓN DE COLORES (Rangos HSV exactos del equipo original)
# =============================================================================
HSV_RANGOS = {
    Color.BLUE: {
        "h": (215, 220),
        "s": (86, 90),
        "v": (20, 33)
    },
    Color.GREEN: {
        "h": (149, 161),
        "s": (52, 61),
        "v": (17, 25)
    },
    Color.YELLOW: {
        "h": (39, 48),
        "s": (67, 70),
        "v": (50, 68)
    },
    Color.WHITE: {
        "h": (204, 216),
        "s": (13, 20),
        "v": (74, 80),
        "reflection": (47, 53)
    },
    Color.BLACK: {
        "s": (10, 40),
        "v": (8, 16),
        "reflection": (0, 8)
    },
    # Azul especial de la zona de matriz
    "BLUE_MATRIX": {
        "h": (221, 225),
        "s": (91, 94),
        "v": (33, 47)
    }
}

# -----------------------------------------------------------------------------
# 5.b ESCÁNER PRECISO (Calibración propia del robot del usuario / MIO)
# -----------------------------------------------------------------------------
# Umbrales de Navegacion.detectar_color_preciso(). color() de Pybricks confunde
# los tonos de la pista bajo la luz de competencia, así que se clasifica a mano
# sobre HSV: primero se separan los acromáticos por saturación y después los
# cromáticos por tono. Se recalibran con odd_shit/calibrador_color.py.
UMBRAL_SATURACION_CROMATICA = 35   # s por debajo de esto: blanco / gris / negro
UMBRAL_VALOR_BLANCO = 65           # v por encima de esto: blanco
UMBRAL_VALOR_GRIS = 40             # v por encima de esto: gris; por debajo: negro
UMBRAL_TONO_AMARILLO_BAJO = 95     # h por debajo de esto: amarillo
UMBRAL_TONO_AMARILLO_ALTO = 310    # h por encima de esto: amarillo (vuelta del círculo)
UMBRAL_TONO_VERDE = 185            # h por debajo de esto: verde; por encima: azul

# -----------------------------------------------------------------------------
# 5.c LECTURA ESTÁTICA DE LA MATRIZ (Lógica de escaneo del robot del usuario)
# -----------------------------------------------------------------------------
# El escaneo por votación del equipo original costaba ~1,25 s por celda
# (250 ms de espera + 25 lecturas x 40 ms) y hasta ~2,5 s cuando salía verde.
# Con el escáner preciso alcanza con confirmar lecturas seguidas iguales.
LECTURAS_CONFIRMACION_MATRIZ = 2   # lecturas seguidas iguales para dar el color por bueno
ESPERA_ASENTAMIENTO_MS = 50        # micro-pausa para leer sin vibraciones del motor
INTERVALO_LECTURAS_MATRIZ_MS = 5   # cadencia de muestreo, igual que los lazos de Navegacion
LECTURAS_MAXIMAS_MATRIZ = 25       # corte de emergencia si nunca aparece un color válido

# =============================================================================
# 6. OPCIONES DE SISTEMA Y SEGURIDAD
# =============================================================================
BATERIA_MINIMA = 8000           # Milivoltios mínimos recomendados
SONIDO_ACTIVO = False           # Desactivado para evitar bloqueos por latencia
TIMEOUT_MOVIMIENTO_MS = 8000
