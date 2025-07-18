import rclpy
from rclpy.node import Node
from .main import *

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.01  # seconds
        self.sArray, self.sdArray, self.sddArray, self.t = initialize_and_generate_trajectory()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # TODO: change msg String for custom message 
        if (i<len(self.sArray)):
            msg = String()
            msg.data = str(f"{self.sArray[self.i]:08.4f}" + f"{self.sdArray[self.i]:08.4f}" + f"{self.sddArray[self.i]:08.4f}")
            # msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1


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