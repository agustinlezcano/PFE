import serial
import time
import threading
import main
import communication
import keyboard
from traj_utils import TrajectoryUtils as traj_utils

# Configurar los dos puertos enlazados (creados con com0com)
PORT_1 = 'COM13'
PORT_2 = 'COM14'
BAUDRATE = 9600

def receptor():
    with serial.Serial(PORT_2, BAUDRATE, timeout=10) as ser2:
        print(f"[Receptor] Esperando datos en {PORT_2}...")
        while True:
            data = ser2.readline()
            print(f"[Receptor] Recibido: {data.decode(errors='ignore')}")
            if keyboard.is_pressed('q'):
                print("[Receptor] Tecla 'q' presionada. Saliendo...")
                break

def emisor():
    time.sleep(0.5)  # Pequeña espera para asegurar que el receptor está listo
    # with serial.Serial(PORT_1, BAUDRATE, timeout=1) as ser1:
    #     mensaje = "¡Hola desde el emisor!\n"
    #     print(f"[Emisor] Enviando: {mensaje.strip()}")
    #     ser1.write(mensaje.encode())
    # #main pero luego refacorizar
    sArray, sdArray, sddArray, t = main.initialize_and_generate_trajectory()    # TODO: Refactor sArray

    for i in range(len(sArray)):
        sArray[i] = round(traj_utils.interpolate(sArray[i],0,2), 4) # TODO: fix: add limit functions, use custom round function

    for i in range(len(t)):
        # Crear la estructura de datos para cada punto de la trayectoria
        data = communication.Communication()
        structured_data = data.createDataStructure(s=sArray[i], sd=sdArray[i], sdd=sddArray[i])
        print(f"[Main] Datos estructurados: {structured_data.strip()}")
        # Enviar los datos estructurados al puerto COM1
        with serial.Serial(PORT_1, BAUDRATE, timeout=1) as ser1:
            ser1.write(structured_data.encode())
            time.sleep(0.5)  # Esperar un poco antes de enviar el siguiente mensaje
        # Recibir la respuesta del puerto COM2

# Ejecutar en paralelo
t1 = threading.Thread(target=receptor)
t2 = threading.Thread(target=emisor)
t1.start()
t2.start()
t1.join()
t2.join()

