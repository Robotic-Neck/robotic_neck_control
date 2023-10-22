import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32, Int16
import math

class PlatformManager(Node):
    """
    This class is used to manage the motors velocity controllers.
    Lineal travel: 2mm/rev
    Velocity: 12.56 rad/s = 2 rev/s
    """
    def __init__(self):
        super().__init__('platform_manager')
        self.motor_velocity = Float32(data=12.56) # rad/s
        
        # Create the publishers with angular velocity setpoints for each motor
        self.left_motor_vel_setpoint_pub = self.create_publisher(Float32, '/left_motor/set_point', 10)
        self.right_motor_vel_setpoint_pub = self.create_publisher(Float32, '/right_motor/set_point', 10)
        
        # Create the subscribers to the lineal travel
        self.left_motor_lineal_travel_sub = self.create_subscription(Float32, '/platform/left_lineal_actuator_travel',
                                                                      self.left_motor_lineal_travel_callback, 10)
        self.right_motor_lineal_travel_sub = self.create_subscription(Float32, '/platform/right_lineal_actuator_travel',
                                                                       self.right_motor_lineal_travel_callback, 10)


    def left_motor_lineal_travel_callback(self, msg):
        if msg.data > 0:
            self.motor_velocity.data = 12.56
        elif msg.data < 0:
            self.motor_velocity.data = -12.56
        motor_rev = abs(msg.data/2) # rev
        time = abs((motor_rev * 2 * math.pi) / self.motor_velocity.data) # s
        self.left_motor_run_duration = Duration(seconds=time)
        self.left_motor_start_time = self.get_clock().now()
        self.left_motor_timer = self.create_timer(0.1, self.publish_left_motor_velocity)

    def right_motor_lineal_travel_callback(self, msg):
        if msg.data > 0:
            self.motor_velocity.data = 12.56
        elif msg.data < 0:
            self.motor_velocity.data = -12.56
        motor_rev = abs(msg.data/2) # rev
        time = abs((motor_rev * 2 * math.pi) / self.motor_velocity.data)
        self.right_motor_run_duration = Duration(seconds=time)
        self.right_motor_start_time = self.get_clock().now()
        self.right_motor_timer = self.create_timer(0.1, self.publish_right_motor_velocity)

    def publish_left_motor_velocity(self):
        current_time = self.get_clock().now()
        if (current_time - self.left_motor_start_time) < self.left_motor_run_duration:
            # Todavía estamos dentro del período de tiempo, publica la velocidad
            self.left_motor_vel_setpoint_pub.publish(self.motor_velocity)
        else:
            # El tiempo ha terminado, así que detenemos el temporizador y, por lo tanto, la publicación
            self.left_motor_timer.cancel()
            self.left_motor_vel_setpoint_pub.publish(Float32(data=0.0))

    def publish_right_motor_velocity(self):
        current_time = self.get_clock().now()
        if (current_time - self.right_motor_start_time) < self.right_motor_run_duration:
            # Todavía estamos dentro del período de tiempo, publica la velocidad
            self.right_motor_vel_setpoint_pub.publish(self.motor_velocity)
        else:
            # El tiempo ha terminado, así que detenemos el temporizador y, por lo tanto, la publicación
            self.right_motor_timer.cancel()
            self.right_motor_vel_setpoint_pub.publish(Float32(data=0.0))



def main(args=None):
    rclpy.init(args=args)

    manager =  PlatformManager()

    rclpy.spin(manager)

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()