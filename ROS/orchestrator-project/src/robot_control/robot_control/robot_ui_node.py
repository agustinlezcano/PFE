"""
Nodo ROS independiente para interfaz de usuario del robot.

Actúa como cliente de servicios ROS que comunica con el nodo publisher
para controlar el robot sin crear threads adicionales.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
import time


class RobotUINode(Node):
    """Nodo ROS para interfaz de usuario interactiva."""

    def __init__(self):
        """Inicializa el nodo de interfaz de usuario."""
        super().__init__('robot_ui_node')
        
        # Publishers para comunicar con el nodo publisher
        self.cmd_publisher = self.create_publisher(Point, '/robot_ui/cmd', 10)
        self.invkin_publisher = self.create_publisher(Point, '/robot_ui/invkin', 10)
        self.homing_publisher = self.create_publisher(Bool, '/robot_ui/homing', 10)
        self.estop_publisher = self.create_publisher(Bool, '/robot_ui/emergency_stop', 10)
        self.load_csv_publisher = self.create_publisher(String, '/robot_ui/load_csv', 10)
        
        # Estado de la interfaz
        self.min_xyz = 0.0
        self.max_xyz = 1.0
        self.emergency_stop = False
        
        self.get_logger().info("RobotUINode iniciado - Interfaz de usuario lista")

    def run_interactive_loop(self):
        """Ejecuta el loop principal de la interfaz interactiva."""
        self.get_logger().info("Iniciando interfaz interactiva...")
        
        while rclpy.ok():
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
                    break
                else:
                    print("Opción inválida. Por favor, intenta de nuevo (1-8).")
                
                time.sleep(0.1)
            
            except KeyboardInterrupt:
                print("\n\n[UserInterface] Interrupción del usuario detectada. Saliendo...")
                break
            
            except Exception as e:
                self.get_logger().error(f"Error en interfaz: {str(e)}")

    def _display_menu(self):
        """Muestra el menú de opciones al usuario."""
        print("\n" + "="*60)
        print("           INTERFAZ DE CONTROL DE ROBOT")
        print("="*60)
        print(f"Rango válido para X, Y, Z: [{self.min_xyz}, {self.max_xyz}]")
        print(f"Estado E-Stop: {'🔴 ACTIVADO' if self.emergency_stop else '🟢 Inactivo'}")
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

    def _validate_xyz(self, x: float, y: float, z: float) -> bool:
        """Valida que los valores XYZ estén dentro de los rangos permitidos."""
        return (
            self.min_xyz <= x <= self.max_xyz
            and self.min_xyz <= y <= self.max_xyz
            and self.min_xyz <= z <= self.max_xyz
        )

    def _parse_xyz_input(self, input_str: str) -> tuple:
        """Parsea entrada de usuario en formato 'x y z'."""
        try:
            parts = input_str.strip().split()
            
            if len(parts) == 3:
                # Solo XYZ
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                
                if not self._validate_xyz(x, y, z):
                    print(f"Valores fuera de rango [{self.min_xyz}, {self.max_xyz}]")
                    return None
                
                return (x, y, z, None, None, None)
            
            elif len(parts) == 6:
                # XYZ + Q1Q2Q3
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                q1, q2, q3 = float(parts[3]), float(parts[4]), float(parts[5])
                
                if not self._validate_xyz(x, y, z):
                    print(f"XYZ fuera de rango [{self.min_xyz}, {self.max_xyz}]")
                    return None
                
                return (x, y, z, q1, q2, q3)
            
            else:
                print("Ingreso inválido. Usa formato: 'x y z' o 'x y z q1 q2 q3'")
                return None
        
        except ValueError:
            print("Error: Ingresa valores numéricos válidos")
            return None

    def _handle_homing(self):
        """Maneja el comando de homing."""
        try:
            confirm = input("¿Ejecutar homing? (s/n): ").strip().lower()
            if confirm == 's':
                msg = Bool()
                msg.data = True
                self.homing_publisher.publish(msg)
                print("[OK] Comando de homing enviado")
                self.get_logger().info("Homing solicitado vía UI")
            else:
                print("[Cancelado] Operación cancelada")
        
        except Exception as e:
            self.get_logger().error(f"Error en homing: {str(e)}")

    def _handle_direct_movement(self):
        """Maneja el movimiento directo XYZ."""
        try:
            print("\n--- Movimiento Directo ---")
            xyz_input = input("Ingresa coordenadas (x y z) o (x y z q1 q2 q3): ").strip()
            coords = self._parse_xyz_input(xyz_input)
            
            if coords:
                x, y, z, q1, q2, q3 = coords
                msg = Point()
                msg.x = x
                msg.y = y
                msg.z = z
                self.cmd_publisher.publish(msg)
                
                log_msg = f"CMD directo: X={x:.4f}, Y={y:.4f}, Z={z:.4f}"
                if q1 is not None:
                    log_msg += f", Q1={q1:.4f}, Q2={q2:.4f}, Q3={q3:.4f}"
                
                print(f"[OK] Comando directo enviado: {log_msg}")
                self.get_logger().info(log_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error en movimiento directo: {str(e)}")

    def _handle_inverse_kinematics(self):
        """Maneja la cinemática inversa XYZ."""
        try:
            print("\n--- Cinemática Inversa ---")
            xyz_input = input("Ingresa coordenadas (x y z) o (x y z q1 q2 q3): ").strip()
            coords = self._parse_xyz_input(xyz_input)
            
            if coords:
                x, y, z, q1, q2, q3 = coords
                msg = Point()
                msg.x = x
                msg.y = y
                msg.z = z
                self.invkin_publisher.publish(msg)
                
                log_msg = f"InvKin: X={x:.4f}, Y={y:.4f}, Z={z:.4f}"
                if q1 is not None:
                    log_msg += f", Q1={q1:.4f}, Q2={q2:.4f}, Q3={q3:.4f}"
                
                print(f"[OK] Comando de cinemática inversa enviado: {log_msg}")
                self.get_logger().info(log_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error en cinemática inversa: {str(e)}")

    def _handle_load_trajectory(self):
        """Maneja la carga de trayectoria desde CSV."""
        try:
            print("\n--- Cargar Trayectoria (CSV) ---")
            filepath = input("Ingresa la ruta del archivo CSV: ").strip()
            
            if filepath:
                # Publicar solicitud de carga de CSV
                msg = String()
                msg.data = filepath
                self.load_csv_publisher.publish(msg)
                time.sleep(0.5)  # Dar tiempo a ROS de procesar
                print(f"[OK] Solicitud de carga enviada: {filepath}")
                self.get_logger().info(f"CSV load request: {filepath}")
            else:
                print("[Cancelado] No se ingresó ruta")
        
        except Exception as e:
            self.get_logger().error(f"Error cargando CSV: {str(e)}")

    def _handle_execute_trajectory(self):
        """Maneja la ejecución de trayectoria."""
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
                print(f"[OK] Ejecutando trayectoria con delay de {delay}s")
                self.get_logger().info(f"Trajectory execution requested with delay={delay}")
            else:
                print("[Cancelado] Ejecución cancelada")
        
        except Exception as e:
            self.get_logger().error(f"Error ejecutando trayectoria: {str(e)}")

    def _handle_emergency_stop(self):
        """Maneja la parada de emergencia."""
        try:
            print("\n--- Parada de Emergencia (E-Stop) ---")
            
            if self.emergency_stop:
                print("E-Stop actualmente ACTIVADO")
                choice = input("¿Desactivar E-Stop? (s/n): ").strip().lower()
                if choice == 's':
                    self.emergency_stop = False
                    msg = Bool()
                    msg.data = False
                    self.estop_publisher.publish(msg)
                    print("[OK] E-Stop desactivado")
                    self.get_logger().info("E-Stop RELEASED")
                else:
                    print("[Cancelado]")
            else:
                print("E-Stop actualmente INACTIVO")
                choice = input("¿Activar E-Stop? (s/n): ").strip().lower()
                if choice == 's':
                    self.emergency_stop = True
                    msg = Bool()
                    msg.data = True
                    self.estop_publisher.publish(msg)
                    print("[OK] E-Stop activado")
                    self.get_logger().info("E-Stop TRIGGERED")
                else:
                    print("[Cancelado]")
        
        except Exception as e:
            self.get_logger().error(f"Error en E-Stop: {str(e)}")

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
                self.get_logger().info(f"Validation range changed to [{self.min_xyz}, {self.max_xyz}]")
            else:
                print("[ERROR] El valor mínimo debe ser menor al máximo")
        
        except ValueError:
            print("[ERROR] Ingresa valores numéricos válidos")
        
        except Exception as e:
            self.get_logger().error(f"Error al cambiar rango: {str(e)}")


def main(args=None):
    """Función principal del nodo UI."""
    rclpy.init(args=args)
    
    ui_node = RobotUINode()
    
    # Ejecutar la interfaz interactiva en el nodo
    try:
        ui_node.run_interactive_loop()
    except KeyboardInterrupt:
        print("\n[RobotUINode] Apagando...")
    finally:
        ui_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
