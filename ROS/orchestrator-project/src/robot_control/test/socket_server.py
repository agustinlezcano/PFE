import socket

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

#Doble bucle while para mantener el servidor activo y aceptar múltiples conexiones secuenciales
while True:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept() # Blocks and waits for an incoming connection
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024) # Receive data in chunks of 1024 bytes
                if not data:
                    break
                
                print(f"Received from client: {data.decode()}")
                if data.decode().casefold() == 'bye':
                    print("Connection closed by client.")
                    break
                elif data.decode().casefold() == 'tuerca':
                    raw_data = "X" + str(151.71) + "Y" + str(151.71) + "Z" + str(5.11)
                    conn.sendall(raw_data.encode()) # Example response
                elif data.decode().casefold() == 'tornillo':
                    raw_data = "X" + str(152.72) + "Y" + str(152.72) + "Z" + str(5.12)
                    conn.sendall(raw_data.encode()) # Example response
                elif data.decode().casefold() == 'llave':
                    raw_data = "X" + str(153.73) + "Y" + str(153.73) + "Z" + str(5.13)
                    conn.sendall(raw_data.encode()) # Example response
                else:
                    conn.sendall(b'Unknown command')
    
