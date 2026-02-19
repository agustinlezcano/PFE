"""
Nodo ROS independiente para interfaz de usuario del robot.

Actúa como cliente de servicios ROS que comunica con el nodo publisher
para controlar el robot sin crear threads adicionales.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from extra_interfaces.msg import Trama
import time
import math
from .limits_validator import LimitsValidator
import os


class RobotUINode(Node):
    """Nodo ROS para interfaz de usuario interactiva."""

    def __init__(self):
        """Inicializa el nodo de interfaz de usuario."""
        super().__init__('robot_ui_node')
        
        # Inicializar validador de límites
        self.limits_validator = LimitsValidator()
        
        # Publishers para comunicar con el nodo publisher
        self.cmd_publisher = self.create_publisher(Trama, '/robot_ui/cmd', 10)
        self.invkin_publisher = self.create_publisher(Point, '/robot_ui/invkin', 10)
        self.homing_publisher = self.create_publisher(Bool, '/robot_ui/homing', 10)
        self.estop_publisher = self.create_publisher(Bool, '/robot_ui/emergency_stop', 10)
        self.load_csv_publisher = self.create_publisher(String, '/robot_ui/load_csv', 10)
        self.request_angles_publisher = self.create_publisher(Bool, '/robot_ui/request_current_angles', 10)
        self.objects_list_publisher = self.create_publisher(String, '/robot_ui/load_objects', 10)
        
        # Subscriber para recibir los ángulos actuales
        self.current_angles_subscriber = self.create_subscription(
            String,
            '/microROS/string_publisher',
            self.current_angles_callback,
            10)
        
        # Obtener límites articulares desde limits_validator
        self.q1_min = self.limits_validator.joint_limits.q1_min
        self.q1_max = self.limits_validator.joint_limits.q1_max
        self.q2_min = self.limits_validator.joint_limits.q2_min
        self.q2_max = self.limits_validator.joint_limits.q2_max
        self.q3_min = self.limits_validator.joint_limits.q3_min
        self.q3_max = self.limits_validator.joint_limits.q3_max
        
        # Obtener límites Cartesianos desde limits_validator
        self.x_min = self.limits_validator.workspace_limits.x_min
        self.x_max = self.limits_validator.workspace_limits.x_max
        self.y_min = self.limits_validator.workspace_limits.y_min
        self.y_max = self.limits_validator.workspace_limits.y_max
        self.z_min = self.limits_validator.workspace_limits.z_min
        self.z_max = self.limits_validator.workspace_limits.z_max
        
        # Estado local de E-Stop para display en UI
        # NOTA: Este flag solo refleja lo que el usuario solicitó desde esta UI.
        # El estado real del robot está en publisher.py (RobotState.EMERGENCY_STOP).
        # Puede desincronizarse si el E-Stop se activa desde otra fuente.
        # TODO: Suscribirse a un topic de estado del robot para sincronizar.
        self.estop_requested = False
        
        self.get_logger().info("RobotUINode iniciado - Interfaz de usuario lista")

    def run_interactive_loop(self):
        """Ejecuta el loop principal de la interfaz interactiva."""
        self.get_logger().info("Iniciando interfaz interactiva...")
        
        while rclpy.ok():
            try:
                self._display_menu()
                choice = input("Selecciona una opción (1-10): ").strip()

                # Dictionary mapping choices to handler methods
                handlers = {
                    "1": self._handle_homing,
                    "2": self._handle_direct_movement,
                    "3": self._handle_inverse_kinematics,
                    "4": self._handle_load_trajectory,
                    "5": self._handle_execute_trajectory,
                    "6": self._handle_emergency_stop,
                    "7": self._handle_change_range,
                    "8": self._handle_request_current_angles,
                    "9": self._handle_load_objects_list,
                    "10": lambda: self._exit_program()
                }

                if choice in handlers:
                    handlers[choice]()
                else:
                    print("Opción inválida. Por favor, intenta de nuevo (1-10).")
                
                time.sleep(0.1)
            
            except KeyboardInterrupt:
                print("\n\n[UserInterface] Interrupción del usuario detectada. Saliendo...")
                break
            
            except Exception as e:
                self.get_logger().error(f"Error en interfaz: {str(e)}")


    def _display_menu(self):
        # 1. Limpiar pantalla (opcional pero recomendado para menús)
        os.system('cls' if os.name == 'nt' else 'clear')

        # 2. Definir colores (ANSI escape codes)
        ROJO = "\033[91m"
        VERDE = "\033[92m"
        RESET = "\033[0m"
        NEGRITA = "\033[1m"
        
        # 3. Determinar estado visual del E-Stop (basado en último request de esta UI)
        estado_estop = f"{ROJO}🔴 ACTIVADO{RESET}" if self.estop_requested else f"{VERDE}🟢 INACTIVO{RESET}"

        # 4. Imprimir con f-string multilínea (más limpio que muchos prints)
        print(f"""
    {NEGRITA}{'='*60}
            INTERFAZ DE CONTROL DE ROBOT
    {'='*60}{RESET}
    Estado E-Stop: {estado_estop}
    {'-'*60}
    Límites Articulares (rad)      |   Límites Cartesianos (m)
    Q1: [{self.q1_min:7.4f}, {self.q1_max:7.4f}]   |   X: [{self.x_min:7.4f}, {self.x_max:7.4f}]
    Q2: [{self.q2_min:7.4f}, {self.q2_max:7.4f}]   |   Y: [{self.y_min:7.4f}, {self.y_max:7.4f}]
    Q3: [{self.q3_min:7.4f}, {self.q3_max:7.4f}]   |   Z: [{self.z_min:7.4f}, {self.z_max:7.4f}]
    {'-'*60}
    1. Homing                        6. Parada de Emergencia
    2. Movimiento Directo (Joints)   7. Cambiar rangos validación
    3. Cinemática Inversa (XYZ)      8. Solicitar Ángulos Actuales
    4. Cargar Trayectoria (CSV)      9. Cargar Lista Objetos (Visión)
    5. Ejecutar Trayectoria          10. Salir
    {'='*60}
    """)

    def current_angles_callback(self, msg: String):
        """Recibe los ángulos actuales desde el microcontrolador."""
        self.get_logger().info(f'Ángulos actuales recibidos: {msg.data}')
        print(f"[INFO] Ángulos actuales: {msg.data}")

    def _validate_joint_position(self, q1: float, q2: float, q3: float) -> bool:
        """Valida que los valores articulares estén dentro de los rangos permitidos."""
        return (
            self.q1_min <= q1 <= self.q1_max
            and self.q2_min <= q2 <= self.q2_max
            and self.q3_min <= q3 <= self.q3_max
        )
    
    def _validate_cartesian_position(self, x: float, y: float, z: float) -> bool:
        """Valida que los valores Cartesianos estén dentro de los rangos permitidos."""
        return (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
            and self.z_min <= z <= self.z_max
        )

    def _parse_joint_input(self, input_str: str) -> tuple:
        """Parsea entrada de usuario en formato 'q1 q2 q3 qd1 qd2 qd3'."""
        try:
            parts = input_str.strip().split()
            
            if len(parts) != 6:
                print("Ingreso inválido. Usa formato: 'q1 q2 q3 qd1 qd2 qd3'")
                return None
            
            q1, q2, q3 = float(parts[0]), float(parts[1]), float(parts[2])
            qd1, qd2, qd3 = float(parts[3]), float(parts[4]), float(parts[5])
            
            if not self._validate_joint_position(q1, q2, q3):
                print(f"Valores fuera de rango:")
                print(f"  Q1: {q1:.4f} debe estar en [{self.q1_min:.4f}, {self.q1_max:.4f}]")
                print(f"  Q2: {q2:.4f} debe estar en [{self.q2_min:.4f}, {self.q2_max:.4f}]")
                print(f"  Q3: {q3:.4f} debe estar en [{self.q3_min:.4f}, {self.q3_max:.4f}]")
                return None
            
            return (q1, q2, q3, qd1, qd2, qd3)
        
        except ValueError:
            print("Error: Ingresa valores numéricos válidos")
            return None
    
    def _parse_cartesian_input(self, input_str: str) -> tuple:
        """Parsea entrada de usuario en formato 'x y z'."""
        try:
            parts = input_str.strip().split()
            
            if len(parts) != 3:
                print("Ingreso inválido. Usa formato: 'x y z'")
                return None
            
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            
            if not self._validate_cartesian_position(x, y, z):
                print(f"Valores fuera de rango:")
                print(f"  X: {x:.4f} debe estar en [{self.x_min:.4f}, {self.x_max:.4f}]")
                print(f"  Y: {y:.4f} debe estar en [{self.y_min:.4f}, {self.y_max:.4f}]")
                print(f"  Z: {z:.4f} debe estar en [{self.z_min:.4f}, {self.z_max:.4f}]")
                return None
            
            return (x, y, z)
        
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
        """Maneja el movimiento directo Q1,Q2,Q3."""
        try:
            print("\n--- Movimiento Directo (Q1 Q2 Q3) ---")
            joint_input = input("Ingresa coordenadas articulares (q1 q2 q3) en grados y sus velocidades (qd1 qd2 qd3): ").strip()
            time_iter_input = input("Ingresa Tiempo Total T y cantidad de Iteraciones N: ").strip()
            coords = self._parse_joint_input(joint_input)
            
            if coords:
                q1, q2, q3, qd1, qd2, qd3 = coords
                msg = Trama()
                msg.q = [q1, q2, q3]
                msg.qd = [qd1, qd2, qd3]
                try:
                    if time_iter_input.strip():
                        t_total, n_iter = map(float, time_iter_input.split())
                    else:
                        t_total, n_iter = 5.0, 50  # Default values
                    msg.t_total = t_total
                    msg.n_iter = int(n_iter)
                except ValueError:
                    print("Error: Ingresa valores numéricos válidos para tiempo e iteraciones")
                    return
                print(f"[OK] Parametros: T={t_total:.4f}, N={n_iter:.4f}")
                self.cmd_publisher.publish(msg)
                time.sleep(0.25)  # Dar tiempo a ROS de procesar
                
                print(f"[OK] Comando directo enviado: Q1={q1:.4f}, Q2={q2:.4f}, Q3={q3:.4f}")
                self.get_logger().info(f"CMD directo: Q1={q1:.4f}, Q2={q2:.4f}, Q3={q3:.4f}")
        
        except Exception as e:
            self.get_logger().error(f"Error en movimiento directo: {str(e)}")

    def _handle_inverse_kinematics(self):
        """Maneja la cinemática inversa XYZ."""
        try:
            print("\n--- Cinemática Inversa (X Y Z) ---")
            xyz_input = input("Ingresa coordenadas Cartesianas (x y z) en metros: ").strip()
            coords = self._parse_cartesian_input(xyz_input)
            
            if coords:
                x, y, z = coords
                msg = Point()
                msg.x = x
                msg.y = y
                msg.z = z
                self.invkin_publisher.publish(msg)
                time.sleep(0.25)  # Dar tiempo a ROS de procesar
                
                print(f"[OK] Comando de cinemática inversa enviado: X={x:.4f}, Y={y:.4f}, Z={z:.4f}")
                self.get_logger().info(f"InvKin: X={x:.4f}, Y={y:.4f}, Z={z:.4f}")
        
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
            print("NOTA: Este estado es local a la UI. El estado real está en el nodo publisher.")
            
            if self.estop_requested:
                print("E-Stop actualmente ACTIVADO (según esta UI)")
                choice = input("¿Desactivar E-Stop? (s/n): ").strip().lower()
                if choice == 's':
                    self.estop_requested = False
                    msg = Bool()
                    msg.data = False
                    self.estop_publisher.publish(msg)
                    print("[OK] E-Stop desactivado")
                    self.get_logger().info("E-Stop RELEASED")
                else:
                    print("[Cancelado]")
            else:
                print("E-Stop actualmente INACTIVO (según esta UI)")
                choice = input("¿Activar E-Stop? (s/n): ").strip().lower()
                if choice == 's':
                    self.estop_requested = True
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
            print("\n--- Cambiar Rangos de Validación ---")
            print("¿Qué deseas cambiar?")
            print("1. Límites de Q1")
            print("2. Límites de Q2")
            print("3. Límites de Q3")
            print("4. Límites de X")
            print("5. Límites de Y")
            print("6. Límites de Z")
            print("7. Volver")
            
            choice = input("Selecciona una opción (1-7): ").strip()
            
            if choice == '7':
                return
            
            # Mapeo de opciones a límites
            options = {
                '1': ('Q1', lambda: (self.q1_min, self.q1_max), 
                      lambda min_v, max_v: (setattr(self, 'q1_min', min_v), setattr(self, 'q1_max', max_v))),
                '2': ('Q2', lambda: (self.q2_min, self.q2_max),
                      lambda min_v, max_v: (setattr(self, 'q2_min', min_v), setattr(self, 'q2_max', max_v))),
                '3': ('Q3', lambda: (self.q3_min, self.q3_max),
                      lambda min_v, max_v: (setattr(self, 'q3_min', min_v), setattr(self, 'q3_max', max_v))),
                '4': ('X', lambda: (self.x_min, self.x_max),
                      lambda min_v, max_v: (setattr(self, 'x_min', min_v), setattr(self, 'x_max', max_v))),
                '5': ('Y', lambda: (self.y_min, self.y_max),
                      lambda min_v, max_v: (setattr(self, 'y_min', min_v), setattr(self, 'y_max', max_v))),
                '6': ('Z', lambda: (self.z_min, self.z_max),
                      lambda min_v, max_v: (setattr(self, 'z_min', min_v), setattr(self, 'z_max', max_v))),
            }
            
            if choice not in options:
                print("[ERROR] Opción inválida")
                return
            
            var_name, getter, setter = options[choice]
            current_min, current_max = getter()
            
            min_input = input(f"Ingresa valor mínimo para {var_name} (actual: {current_min:.4f}): ").strip()
            max_input = input(f"Ingresa valor máximo para {var_name} (actual: {current_max:.4f}): ").strip()
            
            min_val = float(min_input) if min_input else current_min
            max_val = float(max_input) if max_input else current_max
            
            if min_val < max_val:
                setter(min_val, max_val)
                print(f"[OK] Límites de {var_name} actualizado a [{min_val:.4f}, {max_val:.4f}]")
                self.get_logger().info(f"Límites de {var_name} actualizados a [{min_val:.4f}, {max_val:.4f}]")
            else:
                print("[ERROR] El valor mínimo debe ser menor al máximo")
        
        except ValueError:
            print("[ERROR] Ingresa valores numéricos válidos")
        
        except Exception as e:
            self.get_logger().error(f"Error al cambiar rango: {str(e)}")

    def _handle_request_current_angles(self):
        """Solicita los ángulos actuales de los motores."""
        try:
            confirm = input("¿Solicitar ángulos actuales? (s/n): ").strip().lower()
            if confirm == 's':
                msg = Bool()
                msg.data = True
                self.request_angles_publisher.publish(msg)
                time.sleep(0.25)
                print("[OK] Solicitud de ángulos enviada")
                self.get_logger().info("Solicitud de ángulos actuales enviada")
            else:
                print("[Cancelado] Operación cancelada")
        
        except Exception as e:
            self.get_logger().error(f"Error solicitando ángulos: {str(e)}")

    def _handle_load_objects_list(self):
        """Carga lista de objetos para procesamiento con visión."""
        try:
            print("\n--- Cargar Lista de Objetos (Visión) ---")
            filepath = input("Ingresa la ruta del archivo CSV con objetos: ").strip()
            
            if not filepath:
                print("[Cancelado] No se ingresó ruta")
                return
            
            # Verificar si el archivo existe
            if not os.path.exists(filepath):
                print(f"[ERROR] Archivo no encontrado: {filepath}")
                return
            
            # Publicar solicitud de carga de lista de objetos
            msg = String()
            msg.data = filepath
            self.objects_list_publisher.publish(msg)
            time.sleep(0.5)  # Dar tiempo a ROS de procesar
            
            print(f"[OK] Lista de objetos cargada: {filepath}")
            print("    El robot procesará automáticamente cada objeto:")
            print("    1. Solicita coordenadas al servidor de visión")
            print("    2. Valida coordenadas recibidas")
            print("    3. Envía objetivo al planificador de trayectorias")
            print("    4. Repite para el siguiente objeto")
            self.get_logger().info(f"Objects list load request: {filepath}")
        
        except Exception as e:
            self.get_logger().error(f"Error cargando lista de objetos: {str(e)}")

    def _exit_program(self):
        """Finaliza el programa de manera ordenada."""
        confirm = input("¿Salir del programa? (s/n): ").strip().lower()
        if confirm == 's':
            print("[OK] Saliendo del programa...")
            self.get_logger().info("RobotUINode finalizando")
            # Usar rclpy.shutdown() para salir del loop
            rclpy.shutdown()
        else:
            print("[Cancelado] Operación cancelada")


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
