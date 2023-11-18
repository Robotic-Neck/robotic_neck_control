import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import math


class PlatformManager(Node):
    """
    This class is used to manage the motors velocity controllers.
    Lineal travel: 2mm/rev
    Velocity: 12.56 rad/s = 2 rev/s
    """
    def __init__(self):
        super().__init__('platform_manager')
        self.r_motor_velocity = Float32(data=12.56) # rad/s
        self.l_motor_velocity = Float32(data=12.56) # rad/s

        self.prev_l_actuator_pos = 0.0
        self.prev_r_actuator_pos = 0.0

        # Create the publishers with angular velocity setpoints for each motor
        self.l_motor_vel_setpoint_pub = self.create_publisher(Float32, '/left_motor/set_point', 10)
        self.r_motor_vel_setpoint_pub = self.create_publisher(Float32, '/right_motor/set_point', 10)

        # Create the subscribers to the inverse kinematics nodes
        self.joint_states_sub = self.create_subscription(JointState, 'target/joint_states',
                                                         self.joint_states_callback, 1)

    def joint_states_callback(self, msg):
        actual_l_actuator_pos = msg.position[9]
        actual_r_actuator_pos = msg.position[4]

        l_dist = 0.0
        r_dist = 0.0

        # Lineal actuator goes down
        if actual_l_actuator_pos < self.prev_l_actuator_pos:
            self.l_motor_velocity.data = -12.56
            l_dist = abs(actual_l_actuator_pos - self.prev_l_actuator_pos)
        # Lineal actuator goes up
        elif actual_l_actuator_pos > self.prev_l_actuator_pos:
            self.l_motor_velocity.data = 12.56
            l_dist = abs(actual_l_actuator_pos - self.prev_l_actuator_pos)

        # Lineal actuator goes down
        if actual_r_actuator_pos < self.prev_r_actuator_pos:
            self.r_motor_velocity.data = -12.56
            r_dist = abs(actual_r_actuator_pos - self.prev_r_actuator_pos)
        elif actual_r_actuator_pos > self.prev_r_actuator_pos:
            self.r_motor_velocity.data = 12.56
            r_dist = abs(actual_r_actuator_pos - self.prev_r_actuator_pos)

        self.prev_r_actuator_pos = actual_r_actuator_pos
        self.prev_l_actuator_pos = actual_l_actuator_pos

        self.get_logger().info(f'Left actuator distance: {l_dist}')
        self.get_logger().info(f'Right actuator distance: {r_dist}')


        if l_dist > 0.0:
            l_dist = l_dist * 1000 # m to mm
            l_rev = l_dist / 2 # mm to rev
            l_time = abs((l_rev * 2 * math.pi) / self.l_motor_velocity.data)
            self.l_motor_run_duration = Duration(seconds=l_time)
            self.l_motor_start_time = self.get_clock().now()
            self.l_motor_timer = self.create_timer(0.1, self.publish_left_motor_velocity)


        if r_dist > 0.0:
            r_dist = r_dist * 1000
            r_rev = r_dist / 2
            r_time = abs((r_rev * 2 * math.pi) / self.r_motor_velocity.data)
            self.r_motor_run_duration = Duration(seconds=r_time)
            self.r_motor_start_time = self.get_clock().now()
            self.r_motor_timer = self.create_timer(0.1, self.publish_right_motor_velocity)

    def publish_left_motor_velocity(self):
        current_time = self.get_clock().now()
        if (current_time - self.l_motor_start_time) < self.l_motor_run_duration:
            # Todavía estamos dentro del período de tiempo, publica la velocidad
            self.l_motor_vel_setpoint_pub.publish(self.l_motor_velocity)
        else:
            # El tiempo ha terminado, así que detenemos el temporizador y, por lo tanto, la publicación
            self.l_motor_timer.cancel()
            self.l_motor_vel_setpoint_pub.publish(Float32(data=0.0))

            
    def publish_right_motor_velocity(self):
        current_time = self.get_clock().now()
        if (current_time - self.r_motor_start_time) < self.r_motor_run_duration:
            # Todavía estamos dentro del período de tiempo, publica la velocidad
            self.r_motor_vel_setpoint_pub.publish(self.r_motor_velocity)
        else:
            # El tiempo ha terminado, así que detenemos el temporizador y, por lo tanto, la publicación
            self.r_motor_timer.cancel()
            self.r_motor_vel_setpoint_pub.publish(Float32(data=0.0))

def main(args=None):
    rclpy.init(args=args)

    manager =  PlatformManager()

    rclpy.spin(manager)

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()