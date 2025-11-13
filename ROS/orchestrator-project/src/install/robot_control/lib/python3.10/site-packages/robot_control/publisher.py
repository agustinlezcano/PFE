import rclpy
from rclpy.node import Node
from .main import *

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
        self.homing_publisher_ = self.create_publisher(Bool, '/microROS/homing', 10)
        self.joint_states_publisher_ = self.create_publisher(JointTrajectoryPoint, '/microROS/joint_states', 10)
        self.publisher_ = self.create_publisher(Point, '/microROS/int32_subscriber', 2)
        self.str_publisher_ = self.create_publisher(String, '/microROS/str_subscriber', 2)
        self.cmd_publisher_ = self.create_publisher(Point, '/microROS/cmd', 2)
        self.inv_publisher_ = self.create_publisher(Point, '/microROS/inverse', 2)
        self.subscriber = self.create_subscription(
            String,
            '/microROS/string_publisher',
            self.subscriber_callback,
            10)

        self.in_subscriber = self.create_subscription(
            Int32,
            '/microROS/in_publisher',
            self.in_subscriber_callback,
            10)
        timer_period = 0.5  # seconds [0.01]
        # self.sArray, self.sdArray, self.sddArray, self.t = initialize_and_generate_trajectory()
        self.doHoming()
        time.sleep(1)
        self.doCmd(0.5, 0.5, 0.5)
        time.sleep(1)
        self.doInvKin(0.5, 0.5, 0.5)
        
        # Two timers logic - just testing
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.timer = self.create_timer(timer_period, self.test_callback) 

        # self.inv_timer = self.create_timer(5, self.inv_timer_callback)  
        # self.timer_joint_states = self.create_timer(10, self.publish_joint_states)
        
        self.i = 0
        # For use in Point Array
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # For use in Joint States
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0



    def timer_callback(self):
        # TODO: change msg String for custom message
        # msg = Int32()
        # msg.data = self.i 
        # self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')
        # self.i += 1
        # msg = Trama()
        # msg.data = ":M2A100V500"
    

        # if (self.i<len(self.sArray)):
        #     msg = String()
        #     msg.data = str(f"{self.sArray[self.i]:08.4f}" + f"{self.sdArray[self.i]:08.4f}" + f"{self.sddArray[self.i]:08.4f}")
        #     # msg.data = 'Hello World: %d' % self.i
        #     self.publisher_.publish(msg)
        #     self.get_logger().info('Publishing: "%s"' % msg.data)
        #     self.i += 1

        msg = Point()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z

        # msg = String()
        # msg.data = ":M2A100V500"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado Timer x={msg.x}, y={msg.y}, z={msg.z}')
        # self.get_logger().info('Publishing: "%s"' % msg.data)


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
        self.get_logger().info('I heard: "%s"' % msg.data)
      

    def doHoming(self):
        do_homing = Bool()
        do_homing.data = False
        self.homing_publisher_.publish(do_homing)
        self.get_logger().info(f'Publicado homing = {do_homing.data}')

    def doCmd(self, x, y, z):
        cmd_msg = Point()
        cmd_msg.x = x
        cmd_msg.y = y
        cmd_msg.z = z
        self.cmd_publisher_.publish(cmd_msg)
        self.get_logger().info(f'Publicado CMD: x={cmd_msg.x}, y={cmd_msg.y}, z={cmd_msg.z}')

    def doInvKin(self, x, y, z):
        cmd_msg = Point()
        cmd_msg.x = x
        cmd_msg.y = y
        cmd_msg.z = z
        self.inv_publisher_.publish(cmd_msg)
        self.get_logger().info(f'Publicado INV: x={cmd_msg.x}, y={cmd_msg.y}, z={cmd_msg.z}')


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
