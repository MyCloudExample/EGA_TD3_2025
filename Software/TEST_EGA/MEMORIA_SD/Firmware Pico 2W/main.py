from machine import UART, Pin, SPI
import sdcard, os, utime

# Configuración UART1 (TX=GP4, RX=GP5)
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
#Pin(4, Pin.IN, Pin.PULL_UP)  # RX
#Pin(5, Pin.IN, Pin.PULL_UP)  # TX
# Configuración SPI0 con pines personalizados
spi = SPI(0, baudrate=1_000_000, sck=Pin(2), mosi=Pin(3), miso=Pin(0))
sd = sdcard.SDCard(spi, Pin(1))  # CS en GP1
os.mount(sd, '/sd')

# Configuración del LED indicador
led = Pin(10, Pin.OUT)  # LED integrado en la Pico (GP25)
led.off()  # Asegurar que empiece apagado

# Función para guardar datos en la SD con indicación LED
def save_to_sd(data):
    try:
        # Encender LED al iniciar escritura
        led.on()
        
        with open('/sd/datos.csv', 'a') as f:
            f.write(data + '\n')
        print("[SD] Dato guardado:", data.strip())
        
        # Parpadear LED para confirmación visual
        for _ in range(2):
            led.off()
            utime.sleep_ms(100)
            led.on()
            utime.sleep_ms(100)
            
    except Exception as e:
        print("[SD] Error al guardar:", e)
        # LED de error (parpadeo rápido)
        for _ in range(5):
            led.off()
            utime.sleep_ms(100)
            led.on()
            utime.sleep_ms(100)
    finally:
        led.off()  # Asegurar que el LED se apague al final

# Variables para control de tiempo
last_data_time = utime.time()
HEARTBEAT_INTERVAL = 10  # Segundos entre mensajes de "no datos"

print("Iniciando sistema con configuración personalizada:")
print("- UART1: TX=GP4, RX=GP5")
print("- SPI0: SCK=GP2, MOSI=GP3, MISO=GP0, CS=GP1")
print("Esperando datos por UART1...")

while True:
    if uart.any():
        data = uart.read().decode('utf-8').strip()
        save_to_sd(data)
        last_data_time = utime.time()  # Resetear el temporizador
    else:
        # Mensaje si no hay datos después de HEARTBEAT_INTERVAL segundos
        if utime.time() - last_data_time > HEARTBEAT_INTERVAL:
            print("[UART1] No se recibieron datos en los últimos", HEARTBEAT_INTERVAL, "segundos.")
            last_data_time = utime.time()  # Evitar repetición continua del mensaje
    
    utime.sleep_ms(100)  # Pequeña pausa para evitar sobrecarga