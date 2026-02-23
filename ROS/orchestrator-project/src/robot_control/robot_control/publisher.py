import rclpy
from rclpy.node import Node
from .main import Orchestrator as Orchestrator, generate_trajectory_profile as generate_trajectory_profile, initialize_and_generate_trajectory as initialize_and_generate_trajectory, nps as nps, plot_trajectory as plot_trajectory, plt as plt, traj_utils as traj_utils
from .limits_validator import LimitsValidator
from .trajectory_reader import CSVTrajectoryReader
from .user_interface import UserInterface
from .image_manager import VisionClientNode
from typing import Optional, Dict

from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import time
from extra_interfaces.msg import Trama
from enum import Enum

class RobotState(Enum):
    '''
    States of the robot for control flow.
    '''
    IDLE = 0                # waiting for commands (e.g., from UI or next object request)
    RUNNING = 1             # sending trajectory to micro-ROS and waiting for completion
    EMERGENCY_STOP = 2      # E-Stop activated, all commands blocked until released
    BLOCKED = 3             # waiting for vision response after requesting object coordinates

class TrajectorySegment(Enum):
    '''
    Segments for multi-segment trajectory workflow (H→A→B→H).
    '''
    NONE = 0                # No active segment
    H_TO_A = 1              # Homing to vision target (object position)
    A_TO_B = 2              # Vision target to drop-off point
    B_TO_H = 3              # Drop-off point back to homing


class MinimalPublisher(Node):

    def __init__(self, enable_ui: bool = False):
        super().__init__('minimal_publisher')
        
        # Inicializar validador de límites y reader de trayectorias
        self.limits_validator = LimitsValidator()
        self.trajectory_reader = CSVTrajectoryReader(self.limits_validator)
        
        # Fix: Initialize objects_reader to use trajectory_reader
        self.objects_reader = self.trajectory_reader
        
        # Current object being processed
        self.current_object = None

        self.request_counter = 0  # Contador de reintentos para solicitudes de visión
        self.vision_last_coordinates = None  # Coordenadas de Vision [P2P, Vision, P2P]
        # TODO: Hacer cinemática directa o partir del homing

        # Initialize vision client
        self.vision_client = VisionClientNode("127.0.0.1", 65432, timeout=5.0, max_retries=3)
        success, msg = self.vision_client.start()
        if success:
            self.get_logger().info(f'Vision client: {msg}')
        else:
            self.get_logger().warning(f'Vision client failed to start: {msg}. Vision features disabled.')

        # Flag de Homing
        self.homing = False
        
        # Flag para solicitar ángulos actuales
        self.request_current_angles = False

        # Estado inicial del robot
        self.state = RobotState.IDLE
        
        # Segment tracking for multi-segment trajectory (H→A→B→H)
        self.current_segment = TrajectorySegment.NONE
        self.homing_position = [0.1723, 0.0, 0.17185]  # Homing position in meters
        self.drop_off_point = [0.15275, -0.15275, 0.05]  # Fixed drop-off point (B)
        
        # Publishers para Hardware Interface micro-ROS
        self.homing_publisher_ = self.create_publisher(Bool, '/microROS/homing', 10)
        self.current_angles_publisher_ = self.create_publisher(Bool, '/microROS/request_current_angles', 10)
        self.cmd_publisher_ = self.create_publisher(Trama, '/microROS/cmd', 10)
        self.inv_publisher_ = self.create_publisher(Point, '/microROS/inverse', 10)
        self.estop_publisher_ = self.create_publisher(Bool, '/microROS/emergency_stop', 10) # TODO: make callback
        self.electroiman_publisher_ = self.create_publisher(Bool, '/microROS/electroiman', 10)
        
        # Publisher para modo de Planificación de Trayectorias
        # TODO: use this when receiving point from Camera node and sending to Trajectory Planner node
        self.trajectory_planning_publisher = self.create_publisher(Point, 'ROS/trajectory_planning', 10) # Currently not used, but will be needed to send objectives to the trajectory planner node
        
        # Publisher para compartir coordenadas de visión con trajectory_planner
        self.vision_position_publisher = self.create_publisher(Point, '/vision/position', 10)
        
        # Publisher para estado de trayectoria a micro-ROS (traj_active)
        self.trajectory_state_publisher = self.create_publisher(Bool, '/microROS/trajectory_state', 10)
        
        # # ======================== Subscribers para tópicos de UI ========================
        self.ui_cmd_subscriber = self.create_subscription(
            Trama,
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
        
        self.ui_e_magnet_on_subscriber = self.create_subscription(
            Bool,
            '/robot_ui/e_magnet_on',
            self.ui_e_magnet_on_callback,
            10)
        
        self.ui_objects_list_subscriber = self.create_subscription(
            String,
            '/robot_ui/load_objects',
            self.ui_objects_list_callback,
            10)
        
        #TODO: check if use this
        self.ui_request_angles_subscriber = self.create_subscription(
            Bool,
            '/robot_ui/request_current_angles',
            self.ui_request_angles_callback,
            10)
        
        # ======================== Subscribers para tópicos de micro-ROS ========================
        self.subscriber = self.create_subscription(
            String,
            '/microROS/string_publisher',
            self.subscriber_callback,
            10) # current communication made here

        
        # ======================== Subscribers para tópicos de Planner ========================
        # Subscriber for planned trajectory path from trajectory planner node
        self.planner_path_subscriber = self.create_subscription(
            Trama,
            'planner/path',
            self.planner_path_callback,
            10)
        
        # Subscriber for trajectory completion signal from planner
        self.trajectory_complete_subscriber = self.create_subscription(
            Bool,
            'planner/trajectory_complete',
            self.trajectory_complete_callback,
            10)
        
        # ======================== Timer para polling de respuestas de visión ========================
        self.vision_timer = self.create_timer(0.1, self.check_vision_response_callback)  # 10 Hz

        self.i = 0
        # For use in Point Array
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # For use in Joint States and Direct Commands
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        timer_period = 0.5  # seconds [0.01] -- Usado en timers [TEST]
        # self.sArray, self.sdArray, self.sddArray, self.t = initialize_and_generate_trajectory()
        # Removed automatic homing - now called from menu
        self.doHoming()

        self.get_logger().info("Publisher node initialized. Waiting for UI node commands...")

        # Initialize optional in-process UI when requested
        if enable_ui:
            try:
                self.user_interface = UserInterface(self)
                self.user_interface.start()
            except Exception as e:
                self.get_logger().error(f'Failed to start UserInterface: {e}')
                self.user_interface = None
        else:
            self.user_interface = None

    # TODO: delete -> now used to test timer publishing
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

    def subscriber_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        # add logic to handle the received message if needed
    
    def array_to_point(self, value) -> Point:
        """
        Convert array or Point value to Point message format.
        
        Args:
            value: Can be a Point message, list, tuple, or numpy array with [x, y, z] values
            
        Returns:
            Point message object
            
        Raises:
            ValueError: If the input format is not supported or doesn't have 3 elements
            
        NOTE: If the received parameter is different than array format (e.g., different
        axes order, different number of elements, or non-numeric values), this method
        will raise an error. Ensure the input data structure matches the expected format:
        [x, y, z] or Point(x, y, z).
        """
        point_msg = Point()
        
        # If already a Point message, return as is
        if isinstance(value, Point):
            return value
        
        # If it's an array-like (list, tuple, numpy array)
        if hasattr(value, '__len__') and hasattr(value, '__getitem__'):
            if len(value) != 3:
                raise ValueError(f"Expected array with 3 elements [x, y, z], got {len(value)} elements")
            try:
                point_msg.x, point_msg.y, point_msg.z = map(float, value)
                return point_msg
            except (TypeError, ValueError) as e:
                raise ValueError(f"Could not convert array elements to float: {e}")
        
        raise ValueError(f"Unsupported value type: {type(value)}. Expected Point, list, tuple, or numpy array")
    
    def planner_path_callback(self, msg: Trama):
        """
        Process trajectory points received from the trajectory planner node.
        
        This callback receives individual waypoints of the planned path and can be used
        to execute commands for Joint motion.
        
        Args:
            msg: Trama message with q[3], qd[3], t_total, n_iter, traj_active
        """
        # TODO: inicializar contador

        # recibir mensaje

        # leer trama


        if msg.q is None or len(msg.q) != 3:
            self.get_logger().error('Invalid Trama message: q must have 3 elements')
            return
        if msg.qd is None or len(msg.qd) != 3:
            self.get_logger().error('Invalid Trama message: qd must have 3 elements')
            return
        if msg.t_total is None:
            self.get_logger().warn('Invalid Trama message: t_total is required')
        if msg.n_iter is None:
            self.get_logger().warn('Invalid Trama message: n_iter is required')
            
        # Extract and forward trajectory active state to micro-ROS
        traj_active = msg.traj_active if hasattr(msg, 'traj_active') else True
        self.get_logger().info(f'Received planned path point: q={msg.q}, qd={msg.qd}, t_total={msg.t_total}, n_iter={msg.n_iter}, traj_active={traj_active}')
        
        # Publish trajectory state to micro-ROS so it can enter trajectoryControl()
        traj_state_msg = Bool()
        traj_state_msg.data = traj_active
        self.trajectory_state_publisher.publish(traj_state_msg)
        self.get_logger().debug(f'Published trajectory state to micro-ROS: traj_active={traj_active}')
        
        self.doCmd(msg.q[0], msg.q[1], msg.q[2], msg.qd[0], msg.qd[1], msg.qd[2], msg.t_total, msg.n_iter)
        if (self.state != RobotState.BLOCKED) and (self.state != RobotState.EMERGENCY_STOP):
            self.state = RobotState.RUNNING # TODO: ver si se pasa a IDLE

    def trajectory_complete_callback(self, msg: Bool):
        """
        Callback for trajectory completion signal from trajectory planner.
        
        Manages segment transitions for the H→A→B→H workflow:
        - H_TO_A complete → start A_TO_B (vision target to drop-off)
        - A_TO_B complete → start B_TO_H (drop-off to homing)
        - B_TO_H complete → set IDLE and request next object
        """
        if not msg.data:
            return  # Only process completion signals
        
        if self.state == RobotState.EMERGENCY_STOP:
            self.get_logger().warn('Trajectory complete received but E-Stop is active')
            return
        
        self.get_logger().info(f'Trajectory segment complete: {self.current_segment.name}')
        
        if self.current_segment == TrajectorySegment.H_TO_A:
            # H→A complete, start A→B (to drop-off point)
            self.current_segment = TrajectorySegment.A_TO_B
            self.get_logger().info('Starting segment A→B (vision target to drop-off)')
            
            # Send drop-off point as next objective
            drop_off_msg = self.array_to_point(self.drop_off_point)
            self.trajectory_planning_publisher.publish(drop_off_msg)
            self.state = RobotState.RUNNING
            
        elif self.current_segment == TrajectorySegment.A_TO_B:
            # A→B complete, start B→H (back to homing)
            self.current_segment = TrajectorySegment.B_TO_H
            self.get_logger().info('Starting segment B→H (drop-off to homing)')
            
            # Send homing position as next objective
            homing_msg = self.array_to_point(self.homing_position)
            self.trajectory_planning_publisher.publish(homing_msg)
            self.state = RobotState.RUNNING
            
        elif self.current_segment == TrajectorySegment.B_TO_H:
            # B→H complete, full cycle done - request next object
            self.current_segment = TrajectorySegment.NONE
            self.state = RobotState.IDLE
            self.get_logger().info('Full trajectory cycle H→A→B→H complete')
            
            # Clear current object and request next one
            self.current_object = None
            self.request_next_object()
        
        else:
            self.get_logger().warn(f'Unexpected segment state: {self.current_segment}')
            self.current_segment = TrajectorySegment.NONE
            self.state = RobotState.IDLE

    def load_objects_list(self, filepath: str) -> bool:
        """
        Carga la lista de objetos a recoger desde un archivo CSV.
        
        Args:
            filepath: Ruta al archivo CSV con la lista de objetos
            
        Returns:
            True si se cargó correctamente, False en caso contrario
            
        Flow:
            1. Lee CSV con lista de objetos (name, id)
            2. Si estado es IDLE, solicita automáticamente el primer objeto
            3. Envía request al servidor de visión
            4. Espera respuesta con coordenadas (procesada en check_vision_response_callback)
        """
        success, message = self.objects_reader.read_objects_list(filepath)
        if success:
            self.get_logger().info(f'Lista de objetos cargada: {message}')
            # Automatically request first object if robot is IDLE
            if self.state == RobotState.IDLE:
                self.get_logger().info('Iniciando procesamiento de lista de objetos...')
                self.request_next_object()
            return True
        else:
            self.get_logger().error(f'Error cargando lista de objetos: {message}')
            return False

    def request_next_object(self):
        """
        Solicita el siguiente objeto a recoger del servidor de visión artificial.
        
        Flujo:
        1. Obtener siguiente objeto de la lista
        2. Enviar solicitud al servidor de visión vía socket
        3. Estado BLOCKED esperando respuesta con coordenadas (x, y, z) TODO: mejorable: que no bloquee y guarde posiciones en cola
        4. Respuesta procesada por check_vision_response_callback()
        """
        next_obj = self.objects_reader.get_next_object()
        if next_obj is None:
            self.get_logger().info('Lista de objetos finalizada. Robot IDLE.')
            self.state = RobotState.IDLE
            return
        
        self.current_object = next_obj
        self.request_counter = 0  # Resetear contador para nuevo objeto
        self.state = RobotState.BLOCKED
        self.get_logger().info(f'Solicitando objeto: {next_obj["name"]} (ID: {next_obj["id"]})')
        
        # Send async request to vision server
        self.vision_client.request_object_async(next_obj['name'])

    def process_vision_response(self, coordinates: tuple[float, float, float]) -> bool:
        """
        Procesa la respuesta del servidor de visión artificial.
        
        Args:
            coordinates: Tupla con coordenadas (x, y, z) del objeto en milímetros
            
        Returns:
            True si se procesó correctamente, False en caso de error
        """
        try:
            x, y, z = coordinates
            
            # Convertir coordenadas de milímetros a metros
            x_m = x / 1000.0
            y_m = y / 1000.0
            z_m = z / 1000.0
            
            self.get_logger().info(f'Coordenadas recibidas (mm): X={x:.2f}, Y={y:.2f}, Z={z:.2f}')
            self.get_logger().info(f'Coordenadas convertidas (m): X={x_m:.4f}, Y={y_m:.4f}, Z={z_m:.4f}')
            
            # Validar coordenadas con LimitsValidator
            valid, msg = self.limits_validator.validate_cartesian_position(x_m, y_m, z_m)
            if not valid:
                self.get_logger().error(f'Coordenadas fuera de límites: {msg}')
                return False
            
            # Convertir a Point message (en metros)
            point_msg = self.array_to_point([x_m, y_m, z_m])
            
            #Almacenar coordenadas del objeto actual para referencia futura (opcional)
            self.vision_last_coordinates = [x_m, y_m, z_m]
            
            # Publicar coordenadas de visión para que trajectory_planner las almacene
            self.vision_position_publisher.publish(point_msg)
            
            self.get_logger().info(f'Respuesta de visión validada: x={point_msg.x:.4f}, y={point_msg.y:.4f}, z={point_msg.z:.4f}')
            
            # Start H→A→B→H workflow: first segment is H→A (homing to vision target)
            self.current_segment = TrajectorySegment.H_TO_A
            self.get_logger().info(f'Starting segment H→A for object: {self.current_object["name"]}')
            
            # Enviar objetivo al trajectory planner (punto A = visión)
            self.trajectory_planning_publisher.publish(point_msg)

            # Cambiar a estado RUNNING (trayectoria en ejecución)
            self.state = RobotState.RUNNING
            self.get_logger().info(f'Trajectory H→A→B→H initiated for {self.current_object["name"]}')
            
            # Resetear contador después de éxito
            self.request_counter = 0
            
            return True
        
        except Exception as e:
            self.get_logger().error(f'Error procesando respuesta de visión: {e}')
            return False
    
    def check_vision_response_callback(self):
        """
        Timer callback para polling de respuestas del servidor de visión.
        Ejecutado a 10 Hz para verificar si hay respuestas pendientes.
        
        NOTE: Este timer continúa ejecutándose durante EMERGENCY_STOP pero sale temprano.
        TODO: Cancelar timer completamente durante EMERGENCY_STOP para mayor eficiencia.
        
        NOTE: Single-threaded executor (rclpy.spin) garantiza que los callbacks
        se ejecutan secuencialmente, evitando race conditions en transiciones de estado.
        Si se usa MultiThreadedExecutor, añadir threading.Lock para proteger self.state.
        """
        max_retries = 3
        
        # Salir temprano si estamos en EMERGENCY_STOP o no esperando visión
        if self.state == RobotState.EMERGENCY_STOP:
            return
        
        # Solo verificar respuestas si estamos esperando una (estado BLOCKED)
        if self.state != RobotState.BLOCKED:
            return
        
        response = self.vision_client.get_response()
        if response is None:
            return  # No hay respuesta aún
        
        # Log respuesta
        if response['success']:
            self.get_logger().info(f"Vision response: {response['message']}")
            # Procesar coordenadas
            success = self.process_vision_response(response['coordinates'])
            if not success:
                # Validación falló - reintentar hasta max_retries veces
                self.request_counter += 1
                self.get_logger().warning(f"Reintentando objeto: {self.current_object['name']} (intento {self.request_counter}/{max_retries})")
                
                if self.request_counter >= max_retries:
                    self.get_logger().error(f"Máximo de reintentos alcanzado para {self.current_object['name']}. Skipping to next object.")
                    self.current_object = None
                    self.request_next_object()  # El contador se resetea en request_next_object()
                else:
                    # Reintentar - solicitar de nuevo al servidor de visión
                    self.vision_client.request_object_async(self.current_object['name'])
                    # Mantener estado BLOCKED para seguir esperando respuesta
            else:
                # Éxito - coordenadas validadas correctamente
                # El estado ya fue cambiado en process_vision_response
                pass
        else:
            # Error de comunicación con servidor de visión - skipear objeto
            self.get_logger().error(f"Vision communication error: {response['message']}. Skipping to next object.")
            self.current_object = None
            self.request_next_object()

    def microros_feedback_callback(self, msg):
        """
        Callback para recibir feedback de microROS.
        
        El mensaje debería contener:
        - angleDone: bool (True si completó movimiento)
        - electroIman: bool (True si electroimán está activo)
        
        Cuando angleDone=True y electroIman=False, significa que el movimiento
        terminó y puede pasar al siguiente objeto.
        """
        # TODO: Reemplazar con mensaje custom adecuado
        # Por ahora usando Point como placeholder
        
        self.get_logger().debug(f'Feedback recibido: x={msg.x}, y={msg.y}, z={msg.z}')
        
        # TODO: Implementar transición RUNNING → IDLE cuando se tenga el mensaje custom de microROS
        # Flujo esperado:
        # 1. Recibir mensaje con angleDone=True cuando el movimiento completó
        # 2. Si electroIman=True, el objeto fue capturado
        # 3. Transicionar a IDLE y pasar de estado de trayectoria (puede venir en Trama en al parte de N_iter)
        # Si N_iter == 3 --> solicitar siguiente objeto (esto afecta a la parte de lectura csv (next_object) y a la parte de UI (mostrar mensaje de éxito))
        #
        # Ejemplo de implementación:
        # if msg.angleDone and not msg.electroIman:
        #     self.state = RobotState.IDLE
        #     self.current_object = None
        #     self.request_next_object()
        # elif msg.angleDone and msg.electroIman:
        #     self.get_logger().info(f'Objeto capturado: {self.current_object["name"]}')
        #     self.state = RobotState.IDLE
        #     self.request_next_object()

    def doHoming(self):
        do_homing = Bool()
        self.homing = True
        do_homing.data = self.homing    #Asignar flag de homing a mensaje
        self.homing_publisher_.publish(do_homing)
        self.get_logger().info(f'Publicado homing = {do_homing.data}')
        self.homing = False
    
    def doElectroiman(self, activate: bool):
        """Publica comando de electroimán a micro-ROS."""
        electroiman_msg = Bool()
        electroiman_msg.data = activate
        self.electroiman_publisher_.publish(electroiman_msg)
        state_str = 'ENCENDIDO' if activate else 'APAGADO'
        self.get_logger().info(f'Publicado electroimán = {electroiman_msg.data} ({state_str})')
    
    def doRequestCurrentAngles(self):
        """Solicita los ángulos actuales de los motores."""
        request_msg = Bool()
        self.request_current_angles = True
        request_msg.data = self.request_current_angles
        self.current_angles_publisher_.publish(request_msg)
        self.get_logger().info(f'Publicado solicitud de ángulos actuales = {request_msg.data}')
        self.request_current_angles = False

    def doCmd(self,q1: float = None, q2: float = None, q3: float = None, qd1: float = None, qd2: float = None, qd3: float = None, t_total: float = 5.0, n_iter: int = 200):
        """Publica comando directo con coordenadas articulares."""
        # Validar coordenadas articulares si se proporcionan
        if self.state == RobotState.EMERGENCY_STOP:
            self.get_logger().error('Cannot execute: Emergency stop active')
            return
        
        if q1 is not None and q2 is not None and q3 is not None and qd1 is not None and qd2 is not None and qd3 is not None:
            if self.state != RobotState.BLOCKED and self.state != RobotState.EMERGENCY_STOP:
                self.state = RobotState.RUNNING
                valid, msg = self.limits_validator.validate_joint_position(q1, q2, q3)
                if not valid:
                    self.get_logger().error(f'Joint validation error: {msg}')
                    return
                # Actualizar variables articulares
                self.q1 = q1
                self.q2 = q2
                self.q3 = q3
                self.qd1 = qd1
                self.qd2 = qd2
                self.qd3 = qd3
            
                # Publicar comando
                # TODO: Change values to be dynamic
                cmd_msg = Trama()
                cmd_msg.q = [q1, q2, q3]
                cmd_msg.qd = [qd1, qd2, qd3]
                cmd_msg.t_total = t_total
                cmd_msg.n_iter = n_iter
                self.cmd_publisher_.publish(cmd_msg)  # Usa el mismo publisher que los timers
            
                log_msg = f'Publicado CMD: q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}, qd1={qd1:.4f}, qd2={qd2:.4f}, qd3={qd3:.4f}, t_total={t_total}, n_iter={n_iter}'
                self.get_logger().info(log_msg)
        else:
            self.get_logger().error('doCmd requiere q1, q2, q3, qd1, qd2, qd3 como argumentos.')

    def doInvKin(self, x, y, z):
        """Publica comando de cinemática inversa con validación de límites."""
        if self.state == RobotState.EMERGENCY_STOP:
            self.get_logger().error('Cannot execute INV KIN: Emergency stop active')
            return
        
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
        self.state = RobotState.RUNNING
        
        log_msg = f'Publicado INV: x={cmd_msg.x}, y={cmd_msg.y}, z={cmd_msg.z}'
        self.get_logger().info(log_msg)
    
    # ======================== UI Callbacks ========================
    
    def ui_cmd_callback(self, msg: Trama):
        """Procesa comandos de movimiento directo desde el nodo UI."""
        # DEBUG: Log valores recibidos
        self.get_logger().info(f'DEBUG ui_cmd_callback: t_total={msg.t_total}, n_iter={msg.n_iter}')

        if self.state == RobotState.EMERGENCY_STOP:
            self.get_logger().warn('E-Stop activado: Comando bloqueado')
            return
        self.doCmd(msg.q[0], msg.q[1], msg.q[2], msg.qd[0], msg.qd[1], msg.qd[2], msg.t_total, msg.n_iter)
    
    def ui_invkin_callback(self, msg: Point):
        """Procesa comandos de cinemática inversa desde el nodo UI."""
        if self.state == RobotState.EMERGENCY_STOP:
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
    
    def ui_e_magnet_on_callback(self, msg: Bool):
        """Procesa comando de electroimán desde el nodo UI y publica a micro-ROS."""
        self.doElectroiman(msg.data)
    
    def ui_objects_list_callback(self, msg: String):
        """
        Procesa comando de cargar lista de objetos desde el nodo UI.
        
        Workflow:
        1. Recibe filepath del CSV con objetos
        2. Carga lista usando load_objects_list()
        3. Automáticamente solicita primer objeto al servidor de visión
        4. process_vision_response() procesa las coordenadas recibidas
        5. Envía objetivo al trajectory planner
        """
        filepath = msg.data
        self.get_logger().info(f'UI solicitó cargar lista de objetos desde: {filepath}')
        success = self.load_objects_list(filepath)
        if not success:
            self.get_logger().error('Fallo al cargar lista de objetos desde UI')
    
    def ui_request_angles_callback(self, msg: Bool):
        """Procesa solicitud de ángulos actuales desde el nodo UI."""
        if msg.data:
            self.doRequestCurrentAngles()

    def cmd_callback(self):
        """Callback periodico para publicar comandos (solo se ejecuta sin UI)."""
        if self.state != RobotState.EMERGENCY_STOP:
            self.doCmd(self.q1, self.q2, self.q3, self.qd1, self.qd2, self.qd3, self.t_total, self.n_iter)
        else:
            self.get_logger().warn('E-Stop activado: Comando bloqueado')
    
    def trigger_emergency_stop(self):
        """Activa la parada de emergencia y cancela el timer de visión."""
        self.state = RobotState.EMERGENCY_STOP
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_publisher_.publish(estop_msg)
        # TODO: Detener timer de visión durante EMERGENCY_STOP
        # self.vision_timer.cancel()
        self.get_logger().critical('EMERGENCY STOP ACTIVADO')
    
    def release_emergency_stop(self):
        """Desactiva la parada de emergencia y reinicia el timer de visión."""
        estop_msg = Bool()
        estop_msg.data = False
        self.estop_publisher_.publish(estop_msg)
        # TODO: Reiniciar timer de visión al salir de EMERGENCY_STOP
        # self.vision_timer = self.create_timer(0.1, self.check_vision_response_callback)
        self.state = RobotState.IDLE
    
    def load_trajectory_from_csv(self, filepath: str, coordinate_type: str = 'joint') -> tuple:
        """
        Carga una trayectoria desde un archivo CSV.
        
        Args:
            filepath: Ruta al archivo CSV
            coordinate_type: Tipo de coordenadas ('cartesian' o 'joint'). Por defecto 'joint'
        """
        success, message = self.trajectory_reader.read_file(filepath, has_header=False, coordinate_type=coordinate_type)
        if success:
            self.get_logger().info(f'Trayectoria cargada: {message}')
        else:
            self.get_logger().error(f'Error cargando trayectoria: {message}')
        return success, message
    
    def execute_trajectory(self, delay: float = 0.1):
        """Ejecuta la trayectoria cargada punto por punto."""
        trajectory = self.trajectory_reader.get_trajectory()
        if not trajectory:
            self.get_logger().warn('No hay trayectoria cargada')
            return
        
        self.get_logger().info(f'Ejecutando trayectoria con {len(trajectory)} puntos')
        
        for point in trajectory:
            if self.state == RobotState.EMERGENCY_STOP:
                self.get_logger().warn('E-Stop activado: Trayectoria interrumpida')
                break
            
            if point.is_cartesian():
                # Use Cartesian coordinates (x, y, z) directly
                self.doInvKin(point.x, point.y, point.z)
            elif point.is_joint():
                # Use joint coordinates with default velocities and timing parameters
                # qd defaults to 100.0 for each joint, t_total and n_iter use doCmd defaults (5.0, 200)
                qd1 = getattr(point, 'qd1', 100.0)
                qd2 = getattr(point, 'qd2', 100.0)
                qd3 = getattr(point, 'qd3', 100.0)
                self.doCmd(point.q1, point.q2, point.q3, qd1, qd2, qd3)
            time.sleep(delay)

        if self.state != RobotState.EMERGENCY_STOP:
            self.state = RobotState.IDLE    # when finish point, set to IDLE and wait for next command
            
            
    
    def destroy_node(self):
        """Override to cleanup vision client before destroying node."""
        self.get_logger().info('Shutting down vision client...')
        self.vision_client.stop()
        super().destroy_node()


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
