#!/usr/bin/env python3
"""
Mock Vision Server for testing robot control socket communication.

This server listens for object name requests and returns mock coordinates
in the format: X###Y###Z###

Usage:
    python3 mock_vision_server.py

Press Ctrl+C to stop the server.
"""

import socket
import random
import time
import sys

HOST = "127.0.0.1"  # Listen on localhost
PORT = 65432        # Port to listen on

# Mock object database with realistic coordinates (in mm)
OBJECT_DATABASE = {
    "tornillo": {"x": 150.5, "y": 200.3, "z": 50.0},
    "tuerca": {"x": 175.2, "y": 180.6, "z": 45.5},
    "llave": {"x": 190.8, "y": 210.1, "z": 52.3},
}


def generate_mock_coordinates(object_name: str) -> str:
    """
    Generate mock coordinates for an object request.
    
    Args:
        object_name: Name of the requested object
        
    Returns:
        String in format "X###Y###Z###"
    """
    # Check if object exists in database
    if object_name in OBJECT_DATABASE:
        coords = OBJECT_DATABASE[object_name]
        # Add small random variation to simulate real vision system
        x = coords["x"] + random.uniform(-5, 5)
        y = coords["y"] + random.uniform(-5, 5)
        z = coords["z"] + random.uniform(-2, 2)
    else:
        # Unknown object - generate random coordinates
        x = random.uniform(100, 250)
        y = random.uniform(150, 250)
        z = random.uniform(40, 60)
    
    # Format response
    response = f"X{x:.2f}Y{y:.2f}Z{z:.2f}"
    return response


def main():
    """Main server loop."""
    print(f"[Mock Vision Server] Starting on {HOST}:{PORT}")
    print(f"[Mock Vision Server] Known objects: {', '.join(OBJECT_DATABASE.keys())}")
    print(f"[Mock Vision Server] Press Ctrl+C to stop\n")
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        
        print(f"[Mock Vision Server] Listening for connections...\n")
        
        try:
            while True:
                # Accept connection
                conn, addr = server_socket.accept()
                print(f"[Mock Vision Server] Connected by {addr}")
                
                try:
                    with conn:
                        while True:
                            # Receive object request
                            data = conn.recv(1024)
                            if not data:
                                print(f"[Mock Vision Server] Client disconnected")
                                break
                            
                            object_name = data.decode().strip()
                            print(f"[Mock Vision Server] Received request: '{object_name}'")
                            
                            # Simulate processing delay (vision algorithm)
                            time.sleep(0.5)
                            
                            # Generate mock coordinates
                            response = generate_mock_coordinates(object_name)
                            
                            # Send response
                            conn.sendall(response.encode())
                            print(f"[Mock Vision Server] Sent response: {response}\n")
                
                except ConnectionResetError:
                    print(f"[Mock Vision Server] Connection reset by client\n")
                except Exception as e:
                    print(f"[Mock Vision Server] Error: {e}\n")
        
        except KeyboardInterrupt:
            print(f"\n[Mock Vision Server] Shutting down gracefully...")
            sys.exit(0)


if __name__ == "__main__":
    main()
