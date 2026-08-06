import serial
import time
import yarp

# Comunicación YARP
yarp.Network.init()

port = yarp.Port()

if not port.open("/kinfu"):
    print("No se ha podido abrir el puerto")

if not yarp.Network.connect(port.getName(), "/sceneReconstruction/rpc:s"):
    print("No se ha podido establecer una conexión")

# Registro de comandos para la comunicación con la aplicación de reconstrucción
pause = yarp.Bottle("pause")
resume = yarp.Bottle("resume")

# Configuración de la conexión serie.

# Ajustamos el puerto según el puerto USB utilizado por nuestro Arduino
# y de acuerdo a la velocidad en bps establecida en su código.
ser = serial.Serial("COM4", 9600)
time.sleep(2) # Espera 2 segundos a que se establezca la conexión

# Solicitamos al usuario un número de grados
angle = int(input("Ingrese el número de grados: "))

# Solicitamos al usuario el tiempo de giro (en milisegundos)
turning_time = int(input("Ingrese el tiempo de giro en milisegundos: "))

# Ahora construimos el mensaje que enviaremos al Arduino.
mensaje = f"G{angle}T{turning_time}\n"

# Se imprime por pantalla el mensaje a enviar
print(f"Enviando mensaje: {mensaje.strip()}")

# Se envía hacia KinFu
if not port.write(resume):
    print("Ha fallado el comando resume")

# Se envía por USB el mensaje codificado al Arduino
ser.write(mensaje.encode())

# Espera un tiempo (2 segundos) para que Arduino procese y responda
time.sleep(2)

# Lee la respuesta de Arduino si la hubiera
while ser.in_waiting > 0:
    line = ser.readline().decode("utf-8").rstrip()
    print(f"Respuesta de Arduino: {line}") # Se imprime por pantalla

# Esperamos a que el motor termine de girar para cerrar los recursos
time.sleep(turning_time/1000)

if not port.write(pause):
    print("Ha fallado el comando pause")

# Cierre de los canales de comunicación
port.close()
ser.close()
