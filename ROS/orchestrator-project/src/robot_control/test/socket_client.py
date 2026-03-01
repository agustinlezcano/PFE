import socket
import re

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65432        # The port used by the server

pattern = r'X([-+]?[0-9]*\.?[0-9]+)Y([-+]?[0-9]*\.?[0-9]+)Z([-+]?[0-9]*\.?[0-9]+)'

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    try:
        s.connect((HOST, PORT)) # Connect to the server
    except ConnectionRefusedError:
        print("Conexion fallida. Asegurate de que el servidor este corriendo.")
        exit()
        
    message = input("Enter message -> ") #Cambiar por evento que dispare el envío
    while message.lower().strip() != 'bye':
        s.sendall(message.encode()) # Envía el mensaje al servidor
        data = s.recv(1024) # Recibe la respuesta del servidor
        print(f"Received from server: {data.decode()}")
        
        # Verifica si el mensaje recibido coincide con el patrón esperado
        match = re.search(pattern, data.decode())
        if match:
            x, y, z = match.groups()
            print(f"Received coordinates: X={x}, Y={y}, Z={z}")
        message = input("Enter message -> ")
