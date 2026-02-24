from wsgiref.validate import validator
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.publisher import Publisher
import numpy as np
from robot_control.path_planner import PathPlanner
from .Bspline import PathGenerator
from .gen_traj import TrajectoryPlanner
from .inverse_kinematics import RobotConfiguration
from .verify_point_inside_ws import WorkspaceValidator
from .inverse_kinematics import KinematicsSolver
from extra_interfaces.msg import Trama
from std_msgs.msg import Bool

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.publish_rate = self.get_parameter('publish_rate').value
        self.q, self.qd, self.T, self.N = None, None, None, None    #Placeholders for trajectory data
        
        # Subscription to receive objective points from publisher node
        self.subscription = self.create_subscription(
            Point,
            'ROS/trajectory_planning',
            self.objective_callback,
            10
        )
        
        # Publisher to send planned trajectory points to publisher node
        self.path_publisher: Publisher = self.create_publisher(
            Trama,
            'planner/path',
            10
        )
        # Publisher to send elctromagnet data
        self.electroiman_publisher_: Publisher = self.create_publisher(
            Bool,
            'planner/electroiman',
            10
        )
        
        # Publisher to signal trajectory completion to publisher node
        self.trajectory_complete_publisher: Publisher = self.create_publisher(
            Bool,
            'planner/trajectory_complete',
            10
        )
        #self.path_planner = PathPlanner()   # TODO: change to actual path planner implementation (@emanuel)


        # ------------------------------TEST TRAYECTORIA ------------------------------
        # Create instances
        # TODO: pedir el dato de angulo (feedback de microROS)
        # TODO: hacer cinematica directa 
        # darle valor a last_point
        self.last_point = None  # TODO: check initial point from microcontroller
        self.homing_position = np.array([0.1723, 0.0, 0.17185])  # TODO: hacer cinematica directa de 0 90 0
        #self.execute_trajectory_setup() # TODO: borrar, solo encesito la definicion, llamo de otro lado
        self.trajectory_state = None  # 0 = not started, 1 = started, 2 = active, 3 = completed: como compartir el z entre trajectory_planner y publisher? (para que el planner sepa a que altura planificar, y el publisher a que altura mover el robot) (@emanuel)
        self._electroiman_state = False

        #------------------------------------------------------------------------------
        self.trajectory = None
        self.trajectory_index = 0
        self.timer = None
        
        self.get_logger().info('Trajectory Planner Node started')

    def execute_trajectory_setup(self, startPoint=None, endPoint=np.array([0.15275, -0.15275, 0.05])):
        if startPoint is None:
            startPoint = self.homing_position
        config = RobotConfiguration()
        kinematics = KinematicsSolver(config)
        path_generator = PathGenerator(resolution=50, smoothing=0.5, degree=3)
        validator = WorkspaceValidator()
        planner = TrajectoryPlanner(kinematics, path_generator, validator)
    
        # Define waypoints
        if np.allclose(startPoint, self.homing_position) or np.allclose(endPoint, self.homing_position):
            waypoints = np.array([
                startPoint,
                endPoint
            ]) 
            # TODO: electroIman apagado
            self.electroiman(False)
        else:
            waypoints = np.array([
                startPoint,
                [0.15275, 0.0, 0.18],
                endPoint
            ])
            # TODO: electroIman encendido
            self.electroiman(True)
        # Plan and plot trajectory
        #q, qd, T, N = planner.plan(waypoints, plot=True)
        return planner.plan(waypoints, plot=False)

    """Este metodo es llamado para callback de microROS, recibe un booleano para encender o apagar el electroiman, publica el mensaje correspondiente y loguea la accion realizada."""
    def electroiman(self, logic : Bool = None):
            """Enciende/apaga el electroimán."""
            electroiman_msg = Bool()
            self._electroiman_state = logic
            electroiman_msg.data = self._electroiman_state
            self.electroiman_publisher_.publish(electroiman_msg)
            self.get_logger().info(f'Publicado electroimán = {electroiman_msg.data}')


    def objective_callback(self, msg):
        """Handle incoming objective point"""
        objective = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'Received objective point: {objective}')
        
        # Stop existing timer if running
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
    
        # Retrieve trajectory based on objective
        #self.trajectory = self.path_planner.plan_path(objective)    # TODO: make trajectory planning (input format: numpy array(float[3]), output format: list of numpy array(float[3])) (@emanuel)
        self.trajectory_index = 0
        q, qd, T, N = self.execute_trajectory_setup(startPoint=self.last_point if self.last_point is not None else self.homing_position, endPoint=objective) # TODO: check startPoint=objective (para mi seria redundante, pero por las dudas lo dejo asi) (@emanuel)
        
        self.q, self.qd, self.T, self.N = q, qd, T, N
        self.last_point = objective

        delta_t = T / (N-1) if N is not None and N > 1 else self.publish_rate

        if self.q is not None and len(self.q) > 0:
            self.get_logger().info(f'Trajectory retrieved with {len(self.q)} points')
            self.timer = self.create_timer(delta_t, self.publish_trajectory_point)
        else:
            self.get_logger().warn('Failed to retrieve trajectory')

    def stop_trajectory_publishing(self):
        """Stop publishing trajectory points and signal completion."""
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
            self.get_logger().info('Trajectory publishing stopped')
            
            # Signal trajectory completion to publisher node
            complete_msg = Bool()
            complete_msg.data = True
            self.trajectory_complete_publisher.publish(complete_msg)
            self.get_logger().info('Trajectory complete signal sent')

    def publish_trajectory_point(self):
        """Publish trajectory points periodically"""
        
        # Validate trajectory data exists and index is in bounds
        if self.q is None or self.qd is None:
            self.get_logger().error('Trajectory data is None')
            self.stop_trajectory_publishing()
            return
            
        if self.trajectory_index < 0 or self.trajectory_index >= len(self.q):
            self.get_logger().info(f'Trajectory complete. Published {self.trajectory_index} points.')
            self.stop_trajectory_publishing()
            return
        
        # Get current position and velocity from trajectory
        position = self.q[self.trajectory_index]
        velocity = self.qd[self.trajectory_index]
        
        # Validate data shapes (should be 3-element arrays for 3 joints)
        if len(position) != 3 or len(velocity) != 3:
            self.get_logger().error(f'Invalid trajectory data shape at index {self.trajectory_index}')
            self.stop_trajectory_publishing()
            return
        
        #--------------------Trajectory state mapping to Trama.msg--------------------
        #  - index 0 => start (1)
        #  - index in (0, len-1) => active (2)
        #  - index == len-1 => completed (3) (last point published contains completion state)
        if self.trajectory_index == 0:
            self.trajectory_state = int(1)  # Trajectory just started
        elif self.trajectory_index == len(self.q) - 1:
            # Last point: mark as completed
            self.trajectory_state = int(3)
        else:
            # Still publishing trajectory points - trajectory is active
            self.trajectory_state = int(2)
        #----------------------------------------------------------------------------
        # Create and publish message
        msg = Trama()
        msg.q = position.tolist() if hasattr(position, 'tolist') else list(position)
        msg.qd = velocity.tolist() if hasattr(velocity, 'tolist') else list(velocity)
        msg.t_total = float(self.T) if self.T is not None else 0.0
        msg.n_iter = int(self.N) if self.N is not None else 0
        msg.traj_state = int(self.trajectory_state) if self.trajectory_state is not None else 0

        self.path_publisher.publish(msg)
        
        self.get_logger().info(f'Index {self.trajectory_index}, State={msg.traj_state}')
        self.get_logger().debug(f'Published point {self.trajectory_index}/{len(self.q)}: q={position}')
        self.trajectory_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()