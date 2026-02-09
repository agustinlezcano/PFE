import rclpy
from rclpy.node import Node
from .main import initialize_and_generate_trajectory
from .limits_validator import LimitsValidator
from .trajectory_reader import CSVTrajectoryReader

from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from example_interfaces.msg import String as StringInterface
from example_interfaces.msg import WString
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
import time
from extra_interfaces.msg import Trama


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Inicializar validador de límites y reader de trayectorias
        self.limits_validator = LimitsValidator()
        self.trajectory_reader = CSVTrajectoryReader(self.limits_validator)
        
        # Flag de parada de emergencia
        self.emergency_stop = False

        # Flag de Homing
        self.homing = False
        
        # Flag para solicitar ángulos actuales
        self.request_current_angles = False
        
        # Publishers
        self.homing_publisher_ = self.create_publisher(Bool, '/microROS/homing', 10)
        self.current_angles_publisher_ = self.create_publisher(Bool, '/microROS/request_current_angles', 10)
        self.joint_states_publisher_ = self.create_publisher(JointTrajectoryPoint, '/microROS/joint_states', 10)    # TODO: delete
        self.publisher_ = self.create_publisher(Point, '/microROS/int32_subscriber', 10)    # TODO: delete (only used for logs)
        self.str_publisher_ = self.create_publisher(String, '/microROS/str_subscriber', 10) # TODO: delete
        self.cmd_publisher_ = self.create_publisher(Point, '/microROS/cmd', 10)
        self.inv_publisher_ = self.create_publisher(Point, '/microROS/inverse', 10)
        self.estop_publisher_ = self.create_publisher(Bool, '/microROS/emergency_stop', 10) # TODO: make callback
        
        # Subscribers para tópicos de UI
        self.ui_cmd_subscriber = self.create_subscription(
            Point,
            '/robot_ui/cmd',
            self.ui_cmd_callback,
            10)
        
        self.ui_invkin_subscriber = self.create_subscription(
            Point,
            '/robot_ui/invkin',
            self.ui_invkin_callback,
            10)
        
        self.ui_homing_subscriber = self.create_subscription(
            Bool,
            '/robot_ui/homing',
            self.ui_homing_callback,
            10)
        
        self.ui_estop_subscriber = self.create_subscription(
            Bool,
            '/robot_ui/emergency_stop',
            self.ui_estop_callback,
            10)
        
        self.ui_csv_subscriber = self.create_subscription(
            String,
            '/robot_ui/load_csv',
            self.ui_csv_callback,
            10)
        
        self.ui_request_angles_subscriber = self.create_subscription(
            Bool,
            '/robot_ui/request_current_angles',
            self.ui_request_angles_callback,
            10)
        
        # Subscribers para tópicos de micro-ROS
        self.subscriber = self.create_subscription(
            String,
            '/microROS/string_publisher',
            self.subscriber_callback,
            10)

        # TODO: delete -> now used to test timer publishing
        self.in_subscriber = self.create_subscription(
            String,
            '/microROS/tim_publisher',
            self.in_subscriber_callback,
            10)

        # TODO: delete
        self.cmd_string_subscriber = self.create_subscription(
            String,
            '/microROS/cmd_publisher',
            self.cmd_subscriber_callback,
            10)

        timer_period = 0.5  # seconds [0.01] -- Usado en timers [TEST]
        
        # Initialize trajectory arrays for LSPB trajectory generation
        # Used by custom_callback() for publishing trajectory data
        # These arrays store position (s), velocity (sd), and acceleration (sdd) profiles
        self.sArray = []    # type: list[float] - Position array
        self.sdArray = []   # type: list[float] - Velocity array  
        self.sddArray = []  # type: list[float] - Acceleration array
        self.t = []         # type: list[float] - Time array
        # Uncomment if trajectory generation is needed at init:
        # self.sArray, self.sdArray, self.sddArray, self.t = initialize_and_generate_trajectory()
        
        try:
            self.doHoming()
        except Exception as e:
            self.get_logger().error(f'Homing failed during initialization: {e}')
        
        # Create timers for periodic tasks [TEST]
        # self.doTimers(0.5)

        self.i = 0
        # For use in Point Array
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # For use in Joint States and Direct Commands
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        
        self.get_logger().info("Publisher node initialized. Waiting for UI node commands...")

    def timer_callback(self):
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado Timer x={msg.x}, y={msg.y}, z={msg.z}')
        # Actualizo valores de ejemplo
        self.x += 0.1
        self.y += 0.1
        self.z += 0.1

    def test_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.i+=1
        self.str_publisher_.publish(msg)
        self.get_logger().info(f'Publicado Timer {msg.data}')

    def inv_timer_callback(self):

        msg = Point()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z

        self.inv_publisher_.publish(msg)
        self.get_logger().info(f'Publicado Inv K: x={msg.x}, y={msg.y}, z={msg.z}')

        # Actualizo valores de ejemplo
        self.x += 0.1
        self.y += 0.1
        self.z += 0.1

    def publish_joint_states(self):
        # Only used in timer [TEST]
        msg = JointTrajectoryPoint()
        msg.positions = [self.q1, self.q2, self.q3]  # Example joint positions
        self.joint_states_publisher_.publish(msg)
        self.get_logger().info('Published joint states: "%s"' % msg.positions)
        self.q1 += 0.1
        self.q2 += 0.1
        self.q3 += 0.1

    def subscriber_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        # add logic to handle the received message if needed

    def in_subscriber_callback(self, msg):
        # TODO: delete
        self.get_logger().info('I heard: "%s"' % msg.data)
    
    def cmd_subscriber_callback(self, msg):
        # TODO: delete
        self.get_logger().info('I heard cmd: "%s"' % msg.data)
        # add logic to handle the received message if needed

    def doHoming(self):
        do_homing = Bool()
        self.homing = True
        do_homing.data = self.homing    #Asignar flag de homing a mensaje
        self.homing_publisher_.publish(do_homing)
        self.get_logger().info(f'Publicado homing = {do_homing.data}')
        self.homing = False

    
    def doRequestCurrentAngles(self):
        """Solicita los ángulos actuales de los motores."""
        request_msg = Bool()
        self.request_current_angles = True
        request_msg.data = self.request_current_angles
        self.current_angles_publisher_.publish(request_msg)
        self.get_logger().info(f'Publicado solicitud de ángulos actuales = {request_msg.data}')
        self.request_current_angles = False

    def doCmd(self,q1: float = None, q2: float = None, q3: float = None):
        """Publica comando directo con coordenadas articulares."""
        # Validar coordenadas articulares si se proporcionan
        if q1 is not None and q2 is not None and q3 is not None:
            valid, msg = self.limits_validator.validate_joint_position(q1, q2, q3)
            if not valid:
                self.get_logger().error(f'Joint validation error: {msg}')
                return
            # Actualizar variables articulares
            self.q1 = q1
            self.q2 = q2
            self.q3 = q3
        
            # Publicar comando
            cmd_msg = Point()
            cmd_msg.x = q1
            cmd_msg.y = q2
            cmd_msg.z = q3
            self.cmd_publisher_.publish(cmd_msg)  # Usa el mismo publisher que los timers
        
            if q1 is not None:
                log_msg = f'Publicado CMD: q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}'
                self.get_logger().info(log_msg)
        else:
            self.get_logger().error('doCmd requiere q1, q2, q3 como argumentos.')

    def doInvKin(self, x, y, z):
        """Publica comando de cinemática inversa con validación de límites."""
        # Validar coordenadas Cartesianas
        valid, msg = self.limits_validator.validate_cartesian_position(x, y, z)
        if not valid:
            self.get_logger().error(f'InvKin validation error: {msg}')
            return
        
        # Publicar comando de cinemática inversa
        cmd_msg = Point()
        cmd_msg.x = x
        cmd_msg.y = y
        cmd_msg.z = z
        self.inv_publisher_.publish(cmd_msg)
        
        log_msg = f'Publicado INV: x={cmd_msg.x}, y={cmd_msg.y}, z={cmd_msg.z}'
        self.get_logger().info(log_msg)
    
    # ======================== UI Callbacks ========================
    
    def ui_cmd_callback(self, msg: Point):
        """Procesa comandos de movimiento directo desde el nodo UI."""
        if self.emergency_stop:
            self.get_logger().warn('E-Stop activado: Comando bloqueado')
            return
        self.doCmd(msg.x, msg.y, msg.z)
    
    def ui_invkin_callback(self, msg: Point):
        """Procesa comandos de cinemática inversa desde el nodo UI."""
        if self.emergency_stop:
            self.get_logger().warn('E-Stop activado: Comando bloqueado')
            return
        self.doInvKin(msg.x, msg.y, msg.z)
    
    def ui_homing_callback(self, msg: Bool):
        """Procesa comando de homing desde el nodo UI."""
        if msg.data:
            self.doHoming()
    
    def ui_estop_callback(self, msg: Bool):
        """Procesa comando de parada de emergencia desde el nodo UI."""
        if msg.data:
            self.trigger_emergency_stop()
        else:
            self.release_emergency_stop()
    
    def ui_csv_callback(self, msg: String):
        """Procesa comando de cargar CSV desde el nodo UI."""
        filepath = msg.data
        success, message = self.load_trajectory_from_csv(filepath)
        if success:
            self.execute_trajectory()
    
    def ui_request_angles_callback(self, msg: Bool):
        """Procesa solicitud de ángulos actuales desde el nodo UI."""
        if msg.data:
            self.doRequestCurrentAngles()

    def invKin_callback(self):
        # Usado en callback de timer [TEST]
        self.doInvKin(self.x, self.y, self.z)
        # Actualizo valores de ejemplo
        self.x += 0.1
        self.y += 0.1
        self.z += 0.1

    def cmd_callback(self):
        """Callback periodico para publicar comandos (solo se ejecuta sin UI)."""
        if not self.emergency_stop:
            self.doCmd(self.q1, self.q2, self.q3)
        else:
            self.get_logger().warn('E-Stop activado: Comando bloqueado')
    
    def trigger_emergency_stop(self):
        """Activa la parada de emergencia."""
        self.emergency_stop = True
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_publisher_.publish(estop_msg)
        self.get_logger().critical('EMERGENCY STOP ACTIVADO')
    
    def release_emergency_stop(self):
        """Desactiva la parada de emergencia."""
        self.emergency_stop = False
        estop_msg = Bool()
        estop_msg.data = False
        self.estop_publisher_.publish(estop_msg)
        self.get_logger().info('Emergency stop desactivado')
    
    def load_trajectory_from_csv(self, filepath: str) -> tuple:
        """Carga una trayectoria desde un archivo CSV."""
        success, message = self.trajectory_reader.read_file(filepath, has_header=False)
        if success:
            self.get_logger().info(f'Trayectoria cargada: {message}')
        else:
            self.get_logger().error(f'Error cargando trayectoria: {message}')
        return success, message
    
    def execute_trajectory(self, delay: float = 0.1):
        """Ejecuta la trayectoria cargada."""
        trajectory = self.trajectory_reader.get_trajectory()
        if not trajectory:
            self.get_logger().warn('No hay trayectoria cargada')
            return
        
        self.get_logger().info(f'Ejecutando trayectoria con {len(trajectory)} puntos')
        
        for point in trajectory:
            if self.emergency_stop:
                self.get_logger().warn('E-Stop activado: Trayectoria interrumpida')
                break
            
            # Execute based on point type (joint or cartesian)
            if point.is_joint():
                self.doCmd(point.q1, point.q2, point.q3)
            elif point.is_cartesian():
                self.doInvKin(point.x, point.y, point.z)
            else:
                self.get_logger().warn(f'Punto de trayectoria sin coordenadas válidas (ni joint ni cartesian) - saltando')
                continue
            
            time.sleep(delay)

    def custom_callback(self):
        '''Ejemplo de callback personalizado. No se usa actualmente.'''
        # TODO: change msg String for custom message
        msg = Int32()
        msg.data = self.i 
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1
        msg = Trama()
        msg.data = ":M2A100V500"
    

        if (self.i<len(self.sArray)):
            msg = String()
            msg.data = str(f"{self.sArray[self.i]:08.4f}" + f"{self.sdArray[self.i]:08.4f}" + f"{self.sddArray[self.i]:08.4f}")
            # msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

    def doTimers(self, timer_period=0.5):
        time.sleep(1)
        self.doCmd(0.5, 0.5, 0.5)
        time.sleep(1)
        self.doInvKin(0.1321, 0.1321, 0.1195)
        time.sleep(2)
        self.doCmd(0.5467, 0.1321, 0.1195)
        
        # Create multiple timers with unique variable names
        self.timer1 = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.test_callback) 
        self.timer3 = self.create_timer(timer_period, self.invKin_callback) 
        self.cmd_timer = self.create_timer(timer_period, self.cmd_callback)

        self.inv_timer = self.create_timer(5, self.inv_timer_callback)  
        self.timer_joint_states = self.create_timer(10, self.publish_joint_states)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
