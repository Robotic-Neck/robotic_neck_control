import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
import math

class PlatformManager(Node):
    """
    This class is used to manage the motors velocity controllers.
    """
    def __init__(self):
        super().__init__('platform_manager')
        

def main(args=None):
    rclpy.init(args=args)

    manager =  PlatformManager()

    rclpy.spin(manager)

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()