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
        
        #self.path_planner = PathPlanner()   # TODO: change to actual path planner implementation (@emanuel)


        # ------------------------------TEST TRAYECTORIA ------------------------------
        # Create instances
        # TODO: pedir el dato de angulo (feedback de microROS)
        # TODO: hacer cinematica directa 
        # darle valor a last_point
        self.last_point = None  # TODO: check initial point from microcontroller
        #self.execute_trajectory_setup() # TODO: borrar, solo encesito la definicion, llamo de otro lado


        #------------------------------------------------------------------------------


        self.trajectory = None
        self.trajectory_index = 0
        self.timer = None
        
        self.get_logger().info('Trajectory Planner Node started')

    def execute_trajectory_setup(self, startPoint=np.array([0.15275, 0.15275, 0.05]), endPoint=np.array([0.15275, -0.15275, 0.05])):
        config = RobotConfiguration()
        kinematics = KinematicsSolver(config)
        path_generator = PathGenerator(resolution=200, smoothing=0.5, degree=3)
        validator = WorkspaceValidator()
        planner = TrajectoryPlanner(kinematics, path_generator, validator)
    
        # Define waypoints
        waypoints = np.array([
            startPoint,
            [0.15275, 0.0, 0.18],
            endPoint
        ])
        
        # Plan and plot trajectory
        #q, qd, T, N = planner.plan(waypoints, plot=True)
        return planner.plan(waypoints, plot=False)

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
        q, qd, T, N = self.execute_trajectory_setup(startPoint=self.last_point if self.last_point is not None else objective, endPoint=objective) # TODO: check startPoint=objective (para mi seria redundante, pero por las dudas lo dejo asi) (@emanuel)
        
        self.q, self.qd, self.T, self.N = q, qd, T, N
        self.last_point = objective

        if self.q is not None and len(self.q) > 0:
            self.get_logger().info(f'Trajectory retrieved with {len(self.q)} points')
            self.timer = self.create_timer(0.1, self.publish_trajectory_point)
        else:
            self.get_logger().warn('Failed to retrieve trajectory')

    def stop_trajectory_publishing(self):
        """Stop publishing trajectory points"""
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
            self.get_logger().info('Trajectory publishing stopped')

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
        
        # Create and publish message
        msg = Trama()
        msg.q = position.tolist() if hasattr(position, 'tolist') else list(position)
        msg.qd = velocity.tolist() if hasattr(velocity, 'tolist') else list(velocity)
        msg.t_total = float(self.T) if self.T is not None else 0.0
        msg.n_iter = int(self.N) if self.N is not None else 0

        self.path_publisher.publish(msg)
        
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