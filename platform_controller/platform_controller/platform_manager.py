import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer


class PlatformManager(Node):
    """
    This class is used to manage the motors velocity controllers.
    Lineal travel: 2mm/rev
    Velocity: 12.56 rad/s = 2 rev/s
    """
    def __init__(self):
        super().__init__('platform_manager')

        self.linear_actuator_setpoint = 0.0

        # Create the publishers with angular velocity setpoints for each motor
        self.l_motor_vel_setpoint_pub = self.create_publisher(Float32, '/left_motor/set_point', 10)
        self.r_motor_vel_setpoint_pub = self.create_publisher(Float32, '/right_motor/set_point', 10)

        # Create the publishers with the actual state lineal actuators
        self.l_actuator_actual_state_pub = self.create_publisher(Float32, '/left_actuator/actual_state', 10)
        self.r_actuator_actual_state_pub = self.create_publisher(Float32, '/right_actuator/actual_state', 10)

        # Create the subscribers with the velocity control effort for lineal actuators
        self.l_actuator_control_effort_sub = self.create_subscription(Float32, '/left_actuator/control_effort',
                                                                       self.l_actuator_control_effort_callback, 10)
        self.r_actuator_control_effort_sub = self.create_subscription(Float32, '/right_actuator/control_effort',
                                                                       self.r_actuator_control_effort_callback, 10)

        # Create the subscribers to the inverse kinematics nodes
        self.target_joint_states_sub = Subscriber(self, JointState, 'target/joint_states')
        self.state_joint_states_sub = Subscriber(self, JointState, 'state/joint_states')
        
        # Create the synchronizer with a 0.1 seconds tolerance
        self.syncronizer = ApproximateTimeSynchronizer([self.target_joint_states_sub, self.state_joint_states_sub], 10, 0.1)
        self.syncronizer.registerCallback(self.joint_states_callback)

        self.get_logger().info('Platform manager node started')

    def joint_states_callback(self, target_joint_states, actual_joint_states):
        """Inverse kinematics callback function, it calculates the error distance to move the lineal actuators"""
        target_l_actuator_pos = target_joint_states.position[9]
        target_r_actuator_pos = target_joint_states.position[4]

        actual_l_actuator_pos = actual_joint_states.position[9]
        actual_r_actuator_pos = actual_joint_states.position[4]

        l_dist = 0.0
        r_dist = 0.0

        l_dist = abs(actual_l_actuator_pos - target_l_actuator_pos)
        r_dist = abs(actual_r_actuator_pos - target_r_actuator_pos)

        if actual_l_actuator_pos < target_l_actuator_pos:
            l_dist = -l_dist
        if actual_r_actuator_pos < target_r_actuator_pos:
            r_dist = -r_dist
            
        # self.get_logger().info(f'Left actuator distance: {l_dist}')
        # self.get_logger().info(f'Right actuator distance: {r_dist}')

        self.l_actuator_actual_state_pub.publish(Float32(data=l_dist))
        self.r_actuator_actual_state_pub.publish(Float32(data=r_dist))


    def l_actuator_control_effort_callback(self, msg):
        # Limit the control effort to 20.0
        if msg.data > 20:
            msg.data = 20.0
        self.l_motor_vel_setpoint_pub.publish(Float32(data=msg.data))
            
    def r_actuator_control_effort_callback(self, msg):
        # Limit the control effort to 20.0
        if msg.data > 20:
            msg.data = 20.0
        self.r_motor_vel_setpoint_pub.publish(Float32(data=msg.data))

def main(args=None):
    rclpy.init(args=args)

    manager =  PlatformManager()

    rclpy.spin(manager)

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()