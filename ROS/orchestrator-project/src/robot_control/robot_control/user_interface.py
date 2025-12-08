import threading
import time
from typing import Optional


class UserInterface:
    """
    Interfaz de usuario interactiva para controlar el robot.
    Permite al usuario ingresar comandos para:
    - Homing
    - Movimiento directo (XYZ)
    - Cinemática inversa (XYZ)
    """

    def __init__(self, publisher=None):
        """
        Inicializa la interfaz de usuario.
        
        Args:
            publisher: Instancia de MinimalPublisher para enviar comandos.
        """
        self.publisher = publisher
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # Rangos válidos para el robot 3DOF (ajustar según especificaciones)
        self.min_xyz = 0.0
        self.max_xyz = 1.0

    def set_publisher(self, publisher):
        """
        Establece la referencia al publisher después de inicialización.
        
        Args:
            publisher: Instancia de MinimalPublisher.
        """
        self.publisher = publisher

    def start(self):
        """Inicia el thread de la interfaz de usuario."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run_interactive_loop, daemon=True)
            self.thread.start()
            if self.publisher:
                self.publisher.get_logger().info("UserInterface iniciada")

    def stop(self):
        """Detiene el thread de la interfaz de usuario."""
        self.running = False
        if self.thread and self.thread != threading.current_thread():
            self.thread.join(timeout=2.0)
            if self.publisher:
                self.publisher.get_logger().info("UserInterface detenida")
        elif self.thread == threading.current_thread():
            # Si se llama desde el mismo thread, solo marca como no corriendo
            if self.publisher:
                self.publisher.get_logger().info("UserInterface detenida")

    def _validate_xyz(self, x: float, y: float, z: float) -> bool:
        """
        Valida que los valores XYZ estén dentro de los rangos permitidos.
        
        Args:
            x, y, z: Coordenadas a validar.
            
        Returns:
            True si los valores son válidos, False en caso contrario.
        """
        return (
            self.min_xyz <= x <= self.max_xyz
            and self.min_xyz <= y <= self.max_xyz
            and self.min_xyz <= z <= self.max_xyz
        )

    def _parse_xyz_input(self, input_str: str) -> Optional[tuple]:
        """
        Parsea entrada de usuario en formato "x y z".
        
        Args:
            input_str: String con valores separados por espacios.
            
        Returns:
            Tupla (x, y, z) si es válida, None en caso contrario.
        """
        try:
            parts = input_str.strip().split()
            if len(parts) != 3:
                if self.publisher:
                    self.publisher.get_logger().warn(
                        "Ingreso inválido. Usa formato: x y z (ej: 0.1 0.2 0.3)"
                    )
                return None
            
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            
            if not self._validate_xyz(x, y, z):
                if self.publisher:
                    self.publisher.get_logger().warn(
                        f"Valores fuera de rango [{self.min_xyz}, {self.max_xyz}]"
                    )
                return None
            
            return (x, y, z)
        
        except ValueError:
            if self.publisher:
                self.publisher.get_logger().warn("Error: Ingresa valores numéricos válidos")
            return None

    def _display_menu(self):
        """Muestra el menú de opciones al usuario."""
        print("\n" + "="*60)
        print("           INTERFAZ DE CONTROL DE ROBOT")
        print("="*60)
        print(f"Rango válido para X, Y, Z: [{self.min_xyz}, {self.max_xyz}]")
        print(f"Estado E-Stop: {'ACTIVADO' if self.publisher and self.publisher.emergency_stop else '🟢 Inactivo'}")
        print("-"*60)
        print("1. Homing")
        print("2. Movimiento Directo (XYZ)")
        print("3. Cinemática Inversa (XYZ)")
        print("4. Cargar Trayectoria (CSV)")
        print("5. Ejecutar Trayectoria")
        print("6. Parada de Emergencia")
        print("7. Cambiar rango de validación")
        print("8. Salir")
        print("="*60)

    def _run_interactive_loop(self):
        """Loop principal interactivo de la interfaz de usuario."""
        print("\n[UserInterface] Iniciando interfaz interactiva...")
        time.sleep(0.5)  # Pequeña pausa para que se inicialize el publisher
        
        while self.running:
            try:
                self._display_menu()
                choice = input("Selecciona una opción (1-8): ").strip()

                if choice == "1":
                    self._handle_homing()
                
                elif choice == "2":
                    self._handle_direct_movement()
                
                elif choice == "3":
                    self._handle_inverse_kinematics()
                
                elif choice == "4":
                    self._handle_load_trajectory()
                
                elif choice == "5":
                    self._handle_execute_trajectory()
                
                elif choice == "6":
                    self._handle_emergency_stop()
                
                elif choice == "7":
                    self._handle_change_range()
                
                elif choice == "8":
                    print("\n[UserInterface] Saliendo...")
                    self.stop()
                    break
                
                else:
                    print("Opción inválida. Por favor, intenta de nuevo (1-8).")
                
                time.sleep(0.1)
            
            except KeyboardInterrupt:
                print("\n\n[UserInterface] Interrupción del usuario detectada. Saliendo...")
                self.stop()
                break
            
            except Exception as e:
                if self.publisher:
                    self.publisher.get_logger().error(f"Error en interfaz: {str(e)}")
                else:
                    print(f"[ERROR] {str(e)}")

    def _handle_homing(self):
        """Maneja el comando de homing."""
        if not self.publisher:
            print("[ERROR] Publisher no inicializado")
            return
        
        try:
            confirm = input("¿Ejecutar homing? (s/n): ").strip().lower()
            if confirm == 's':
                self.publisher.doHoming()
                print("[OK] Comando de homing enviado")
            else:
                print("[Cancelado] Operación cancelada")
        
        except Exception as e:
            if self.publisher:
                self.publisher.get_logger().error(f"Error en homing: {str(e)}")

    def _handle_direct_movement(self):
        """Maneja el movimiento directo XYZ."""
        if not self.publisher:
            print("[ERROR] Publisher no inicializado")
            return
        
        try:
            print("\n--- Movimiento Directo ---")
            xyz_input = input("Ingresa coordenadas (x y z): ").strip()
            coords = self._parse_xyz_input(xyz_input)
            
            if coords:
                x, y, z = coords
                self.publisher.doCmd(x, y, z)
                print(f"[OK] Comando directo enviado: X={x}, Y={y}, Z={z}")
            
        except Exception as e:
            if self.publisher:
                self.publisher.get_logger().error(f"Error en movimiento directo: {str(e)}")

    def _handle_inverse_kinematics(self):
        """Maneja la cinemática inversa XYZ."""
        if not self.publisher:
            print("[ERROR] Publisher no inicializado")
            return
        
        try:
            print("\n--- Cinemática Inversa ---")
            xyz_input = input("Ingresa coordenadas (x y z): ").strip()
            coords = self._parse_xyz_input(xyz_input)
            
            if coords:
                x, y, z = coords
                self.publisher.doInvKin(x, y, z)
                print(f"[OK] Comando de inv. kinematics enviado: X={x}, Y={y}, Z={z}")
            
        except Exception as e:
            if self.publisher:
                self.publisher.get_logger().error(f"Error en cinemática inversa: {str(e)}")

    def _handle_change_range(self):
        """Permite cambiar los rangos de validación de coordenadas."""
        try:
            print("\n--- Cambiar Rango de Validación ---")
            min_input = input(f"Ingresa valor mínimo (actual: {self.min_xyz}): ").strip()
            max_input = input(f"Ingresa valor máximo (actual: {self.max_xyz}): ").strip()
            
            min_val = float(min_input) if min_input else self.min_xyz
            max_val = float(max_input) if max_input else self.max_xyz
            
            if min_val < max_val:
                self.min_xyz = min_val
                self.max_xyz = max_val
                print(f"[OK] Rango actualizado a [{self.min_xyz}, {self.max_xyz}]")
            else:
                print("[ERROR] El valor mínimo debe ser menor al máximo")
        
        except ValueError:
            print("[ERROR] Ingresa valores numéricos válidos")
        
        except Exception as e:
            if self.publisher:
                self.publisher.get_logger().error(f"Error al cambiar rango: {str(e)}")
    
    def _handle_load_trajectory(self):
        """Maneja la carga de trayectoria desde CSV."""
        if not self.publisher:
            print("[ERROR] Publisher no inicializado")
            return
        
        try:
            print("\n--- Cargar Trayectoria (CSV) ---")
            filepath = input("Ingresa la ruta del archivo CSV: ").strip()
            
            if not filepath:
                print("[Cancelado] No se ingresó ruta")
                return
            
            success, message = self.publisher.load_trajectory_from_csv(filepath)
            if success:
                print(f"[OK] {message}")
            else:
                print(f"[ERROR] {message}")
        
        except Exception as e:
            if self.publisher:
                self.publisher.get_logger().error(f"Error cargando trayectoria: {str(e)}")
    
    def _handle_execute_trajectory(self):
        """Maneja la ejecución de trayectoria."""
        if not self.publisher:
            print("[ERROR] Publisher no inicializado")
            return
        
        try:
            print("\n--- Ejecutar Trayectoria ---")
            delay_input = input("Ingresa delay entre puntos en segundos (default 0.1): ").strip()
            
            try:
                delay = float(delay_input) if delay_input else 0.1
            except ValueError:
                print("[ERROR] Ingresa un valor numérico válido")
                return
            
            confirm = input("¿Ejecutar trayectoria? (s/n): ").strip().lower()
            if confirm == 's':
                self.publisher.execute_trajectory(delay=delay)
                print("[OK] Trayectoria ejecutada")
            else:
                print("[Cancelado] Ejecución cancelada")
        
        except Exception as e:
            if self.publisher:
                self.publisher.get_logger().error(f"Error ejecutando trayectoria: {str(e)}")
    
    def _handle_emergency_stop(self):
        """Maneja la parada de emergencia."""
        if not self.publisher:
            print("[ERROR] Publisher no inicializado")
            return
        
        try:
            print("\n--- Parada de Emergencia (E-Stop) ---")
            
            if self.publisher.emergency_stop:
                print("E-Stop actualmente ACTIVADO")
                choice = input("¿Desactivar E-Stop? (s/n): ").strip().lower()
                if choice == 's':
                    self.publisher.release_emergency_stop()
                    print("[OK] E-Stop desactivado")
                else:
                    print("[Cancelado]")
            else:
                print("E-Stop actualmente INACTIVO")
                choice = input("¿Activar E-Stop? (s/n): ").strip().lower()
                if choice == 's':
                    self.publisher.trigger_emergency_stop()
                    print("[OK] E-Stop activado")
                else:
                    print("[Cancelado]")
        
        except Exception as e:
            if self.publisher:
                self.publisher.get_logger().error(f"Error en E-Stop: {str(e)}")

