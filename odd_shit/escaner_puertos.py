from pybricks.hubs import PrimeHub
from pybricks.parameters import Port
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.tools import wait

# Inicializamos el hub
hub = PrimeHub()

# Diccionario con todos los puertos del PrimeHub
puertos = {
    "A": Port.A, 
    "B": Port.B, 
    "C": Port.C, 
    "D": Port.D, 
    "E": Port.E, 
    "F": Port.F
}

# Lista de dispositivos que vamos a intentar instanciar
dispositivos_a_probar = [
    (Motor, "Motor (Cualquier tamaño)"),
    (ColorSensor, "Sensor de Color"),
    (UltrasonicSensor, "Sensor Ultrasónico (Distancia)"),
    (ForceSensor, "Sensor de Fuerza (Botón)")
]

print("="*35)
print("🔍 ESCÁNER DE PUERTOS INICIADO 🔍")
print("="*35)

for letra, puerto in puertos.items():
    dispositivo_encontrado = "Vacío o No Reconocido ❌"
    
    # Intentamos instanciar cada tipo de dispositivo en el puerto actual
    for clase_dispositivo, nombre_dispositivo in dispositivos_a_probar:
        try:
            # Si esto tiene éxito, significa que el dispositivo correcto está ahí
            instancia_prueba = clase_dispositivo(puerto)
            dispositivo_encontrado = f"{nombre_dispositivo} ✅"
            break  # Como ya lo encontramos, dejamos de probar otros dispositivos
            
        except Exception:
            # Si lanza error (ej. intentaste poner Motor en un ColorSensor), 
            # lo ignoramos silenciosamente y probamos el siguiente de la lista
            pass 
            
    print(f"Puerto {letra}: {dispositivo_encontrado}")
    wait(50)  # Pequeña pausa para no saturar el bus de comunicación

print("="*35)
print("Escaneo finalizado.")