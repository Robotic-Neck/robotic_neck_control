import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16, Empty
import math

class MotorManager(Node):
    """
    This class is used to manage the motors velocity controllers.
    """
    def __init__(self):
        super().__init__('motor_manager')
        
        # Motor parameters
        # self.motor_ppr = 7
        # self.motor_gear_ratio = 50
        # self.motor_counts_per_revolution = self.motor_ppr * self.motor_gear_ratio
        self.motor_counts_per_revolution = 1400 # Experimentally obtained
        self.limit_block = False


        # Create the publishers with the actual state of angular velocity
        self.left_motor_actual_vel_pub = self.create_publisher(Float32, '/left_motor/actual_state', 10)
        self.right_motor_actual_vel_pub = self.create_publisher(Float32, '/right_motor/actual_state', 10)

        # Create the subscriber to the control effort
        self.left_motor_control_effort_sub = self.create_subscription(Float32, '/left_motor/control_effort',
                                                                      self.left_motor_control_effort_callback, 10)
        self.right_motor_control_effort_sub = self.create_subscription(Float32, '/right_motor/control_effort',
                                                                       self.right_motor_control_effort_callback, 10)

        # Crete the publishers with the control effort to the raspberry pi
        self.set_left_motor_vel_pub = self.create_publisher(Int16, '/rpip/motor_left_sub', 10)
        self.set_right_motor_vel_pub = self.create_publisher(Int16, '/rpip/motor_right_sub', 10)


        # Create the subscribers with the counts per second from the encoder
        self.left_motor_encoder_sub = self.create_subscription(Int16, '/rpip/encoder_left_pub', 
                                                               self.left_motor_encoder_callback, 10)
        self.right_motor_encoder_sub = self.create_subscription(Int16, '/rpip/encoder_right_pub',
                                                                self.right_motor_encoder_callback, 10)
        self.right_ls_sub = self.create_subscription(Empty,'/rpip/ls_right_pub', self.right_stop_callback, 10)
        self.left_ls_sub = self.create_subscription(Empty,'/rpip/ls_left_pub', self.left_stop_callback, 10)

    def left_motor_control_effort_callback(self, msg):
        """
        This function is used to publish the control effort to the left motor.
        The ideal is that te raspberry Pi subscribe to this topic directly.
        """
        int_msg = Int16()
        int_msg.data = round(msg.data)
        
        """
        This function stops sending speeds when the speed is negative and the lock generated by the
        contact of the lineal actuator with the limit switch is activated. On the other hand, if the speed is positive
         and the limit switch is activated ,it deactives the lock 
        """

        if self.limit_block:
            if int_msg.data >0:
                self.limit_block = False
            else:
                int_msg.data = 0

        if int_msg.data > 255:
            int_msg.data = 255
        elif int_msg.data < -255:
            int_msg.data = -255
        
        self.set_left_motor_vel_pub.publish(int_msg)

    def right_motor_control_effort_callback(self, msg):
        """
        This function is used to publish the control effort to the right motor.
        """
        int_msg = Int16()
        int_msg.data = round(msg.data)

        if self.limit_block:
            if int_msg.data >0:
                self.limit_block = False
            else:
                int_msg.data = 0

        if int_msg.data > 255:
            int_msg.data = 255
        elif int_msg.data < -255:
            int_msg.data = -255
        
        self.set_right_motor_vel_pub.publish(int_msg)
    
    def left_motor_encoder_callback(self, msg):
        """
        This function is used to publish the actual state from the left motor velocity.
        Recieves the counts per second from the encoder and publishes the angular velocity.
        """
        revolution_per_secod = (msg.data / self.motor_counts_per_revolution)/0.1
        angular_velocity = revolution_per_secod * 2 * math.pi
        float_msg = Float32()
        float_msg.data = angular_velocity
        self.left_motor_actual_vel_pub.publish(float_msg)

    def right_motor_encoder_callback(self, msg):
        """
        This function is used to publish the actual state from the right motor velocity.
        Recieves the counts per second from the encoder and publishes the angular velocity.
        """
        revolution_per_secod = (msg.data / self.motor_counts_per_revolution)/0.1
        angular_velocity = revolution_per_secod * 2 * math.pi
        float_msg = Float32()
        float_msg.data = angular_velocity
        self.right_motor_actual_vel_pub.publish(float_msg)

    def left_stop_callback(self, msg):
        self.get_logger().info('Left motor stop')
        self.limit_block = True
        

    def right_stop_callback(self, msg):
        self.get_logger().info('Right motor stop')
        self.limit_block = True
        


def main(args=None):
    rclpy.init(args=args)

    manager =  MotorManager()

    rclpy.spin(manager)

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()