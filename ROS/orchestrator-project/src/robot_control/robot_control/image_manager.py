import socket
import threading
import queue
import re
import time
from typing import Optional, Tuple, Dict


class VisionClient:
    """
    Socket client for communicating with vision server.
    Sends object names and receives coordinates in format: X###Y###Z###
    """
    
    def __init__(self, host: str, port: int, timeout: float = 5.0, max_retries: int = 3):
        """
        Initialize vision client.
        
        Args:
            host: Vision server IP address
            port: Vision server port
            timeout: Socket timeout in seconds
            max_retries: Maximum retry attempts for failed requests
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.max_retries = max_retries
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.lock = threading.Lock()
        
        # Regex pattern to parse coordinate response: X###Y###Z###
        self.pattern = re.compile(r'X([-+]?[0-9]*\.?[0-9]+)Y([-+]?[0-9]*\.?[0-9]+)Z([-+]?[0-9]*\.?[0-9]+)')
    
    def connect(self) -> Tuple[bool, str]:
        """
        Establish connection to vision server.
        
        Returns:
            Tuple (success, message)
        """
        with self.lock:
            try:
                if self.connected and self.socket:
                    return True, "Already connected"
                
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(self.timeout)
                self.socket.connect((self.host, self.port))
                self.connected = True
                return True, f"Connected to vision server at {self.host}:{self.port}"
            
            except ConnectionRefusedError:
                self.connected = False
                return False, f"Connection refused. Ensure vision server is running at {self.host}:{self.port}"
            except socket.timeout:
                self.connected = False
                return False, f"Connection timeout to {self.host}:{self.port}"
            except Exception as e:
                self.connected = False
                return False, f"Connection error: {str(e)}"
    
    def close(self):
        """Close socket connection gracefully."""
        with self.lock:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                finally:
                    self.socket = None
                    self.connected = False
    
    def request_object(self, object_name: str) -> Tuple[Optional[Tuple[float, float, float]], str]:
        """
        Request object coordinates from vision server.
        Implements retry logic for failed requests.
        
        Args:
            object_name: Name of the object to locate (e.g., "tornillo", "tuerca", "llave")
            
        Returns:
            Tuple ((x, y, z) or None, status_message)
        """
        for attempt in range(self.max_retries):
            with self.lock:
                # Ensure connection
                if not self.connected:
                    success, msg = self.connect()
                    if not success:
                        if attempt < self.max_retries - 1:
                            time.sleep(0.5)  # Wait before retry
                            continue
                        return None, f"Connection failed after {self.max_retries} attempts: {msg}"
                
                try:
                    # Send object request
                    message = object_name.encode()
                    self.socket.sendall(message)
                    
                    # Receive response
                    data = self.socket.recv(1024)
                    if not data:
                        raise ConnectionError("Server closed connection")
                    
                    response = data.decode().strip()
                    
                    # Parse coordinates using regex
                    match = self.pattern.search(response)
                    if match:
                        x, y, z = match.groups()
                        coordinates = (float(x), float(y), float(z))
                        return coordinates, f"Object '{object_name}' located at X={x}, Y={y}, Z={z}"
                    else:
                        # Response doesn't match expected pattern
                        if attempt < self.max_retries - 1:
                            time.sleep(0.5)
                            continue
                        return None, f"Invalid response format: {response}"
                
                except socket.timeout:
                    self.connected = False
                    if attempt < self.max_retries - 1:
                        time.sleep(0.5)
                        continue
                    return None, f"Timeout waiting for response for '{object_name}'"
                
                except (ConnectionError, BrokenPipeError) as e:
                    self.connected = False
                    self.close()
                    if attempt < self.max_retries - 1:
                        time.sleep(0.5)
                        continue
                    return None, f"Connection error: {str(e)}"
                
                except Exception as e:
                    if attempt < self.max_retries - 1:
                        time.sleep(0.5)
                        continue
                    return None, f"Error requesting object: {str(e)}"
        
        return None, f"Failed after {self.max_retries} attempts"


class VisionClientNode:
    """
    Threaded wrapper for VisionClient to enable non-blocking operation with ROS2.
    Uses daemon thread and queues for thread-safe communication.
    """
    
    def __init__(self, host: str, port: int, timeout: float = 5.0, max_retries: int = 3):
        """
        Initialize vision client node.
        
        Args:
            host: Vision server IP address
            port: Vision server port
            timeout: Socket timeout in seconds
            max_retries: Maximum retry attempts for failed requests
        """
        self.client = VisionClient(host, port, timeout, max_retries)
        self.request_queue: queue.Queue = queue.Queue()
        self.response_queue: queue.Queue = queue.Queue()
        self.thread: Optional[threading.Thread] = None
        self.running = False
    
    def start(self) -> Tuple[bool, str]:
        """
        Start the vision client thread.
        
        Returns:
            Tuple (success, message)
        """
        if self.running:
            return True, "Vision client already running"
        
        # Connect to server
        success, msg = self.client.connect()
        if not success:
            return False, msg
        
        # Start daemon thread
        self.running = True
        self.thread = threading.Thread(target=self._process_loop, daemon=True)
        self.thread.start()
        
        return True, "Vision client started"
    
    def stop(self):
        """Stop the vision client thread gracefully."""
        if not self.running:
            return
        
        self.running = False
        
        # Wait for thread to finish (with timeout)
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        # Close socket connection
        self.client.close()
    
    def request_object_async(self, object_name: str):
        """
        Queue an async request for object coordinates.
        Non-blocking call.
        
        Args:
            object_name: Name of the object to locate
        """
        self.request_queue.put(object_name)
    
    def get_response(self) -> Optional[Dict]:
        """
        Get the next response from the response queue (non-blocking).
        
        Returns:
            Dict with keys: 'name', 'coordinates' (tuple or None), 'success' (bool), 'message'
            or None if queue is empty
        """
        try:
            return self.response_queue.get_nowait()
        except queue.Empty:
            return None
    
    def _process_loop(self):
        """Internal thread loop to process requests and responses."""
        
        while self.running:
            try:
                # Check for new requests (blocking with timeout)
                try:
                    object_name = self.request_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                # Process request
                coordinates, message = self.client.request_object(object_name)
                
                # Put response in queue
                response = {
                    'name': object_name,
                    'coordinates': coordinates,
                    'success': coordinates is not None,
                    'message': message
                }
                self.response_queue.put(response)
                
            except Exception as e:
                # Handle unexpected errors
                response = {
                    'name': '',
                    'coordinates': None,
                    'success': False,
                    'message': f"Internal error: {str(e)}"
                }
                self.response_queue.put(response)
