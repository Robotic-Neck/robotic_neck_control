from launch import LaunchDescription
from launch_ros.actions import Node
from launch_utils.utils import launch_robot_state_publisher_node as launch_rsp_node
from launch_utils.utils import launch_rviz_node

def generate_launch_description():

    # Set the parameters for the motors velocity PID controller
    motor_out_min = -250
    motor_out_max = 250
    motors_kp = 1.6
    motors_ki = 16.4
    motors_kd = 0.31

    # Set the parameters for the lineal actuators PID controller
    lineal_actuator_out_min = -20
    lineal_actuator_out_max = 20
    lineal_actuators_kp = 1000.0
    lineal_actuators_ki = 0.0
    lineal_actuators_kd = 0.0

    return LaunchDescription([
        Node(package='robotic_neck_viz', executable='neck_state_joint_publisher', name='neck_state_joint_publisher'),
        launch_rsp_node(package_name="robotic_neck_viz", xacro_file="robotic_neck_state.urdf.xacro", namespace="state"),
        Node(package='robotic_neck_viz', executable='neck_target_joint_publisher', name='neck_target_joint_publisher'),
        launch_rsp_node( package_name="robotic_neck_viz", xacro_file="robotic_neck_target.urdf.xacro", namespace="target"),
        launch_rviz_node(package_name='robotic_neck_viz', config_file='robotic_neck.rviz'),
        Node(package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure'
        ),
        Node(
            package='motors_velocity_controller',
            executable='motors_velocity_controller',
            name='motors_velocity_controller_right',
            output='screen',
            parameters=[
                {'kp' : motors_kp},
                {'ki' : motors_ki},
                {'kd' : motors_kd},
                {'use_sample_time' : False},
                {'sample_time' : 2},
                {'derivative_on_measurement' : False},
                {'remove_ki_bump' : False},
                {'reset_windup' : False},
                {'pid_enabled' : True},
                {'cut_off_freq' : 0},
                {'out_min' : motor_out_min},
                {'out_max' : motor_out_max},
                {'control_value_topic' : 'right_motor/control_effort'},
                {'actual_state_topic' : 'right_motor/actual_state'},
                {'set_point_topic' : 'right_motor/set_point'},
                {'loop_freq' : 10}
            ]
        ),
        Node(
            package='motors_velocity_controller',
            executable='motors_velocity_controller',
            name='motors_velocity_controller_left',
            output='screen',
            parameters=[
                {'kp' : motors_kp},
                {'ki' : motors_ki},
                {'kd' : motors_kd},
                {'use_sample_time' : False},
                {'sample_time' : 2},
                {'derivative_on_measurement' : False},
                {'remove_ki_bump' : False},
                {'reset_windup' : False},
                {'pid_enabled' : True},
                {'cut_off_freq' : 0},
                {'out_min' : motor_out_min},
                {'out_max' : motor_out_max},
                {'control_value_topic' : 'left_motor/control_effort'},
                {'actual_state_topic' : 'left_motor/actual_state'},
                {'set_point_topic' : 'left_motor/set_point'},
                {'loop_freq' : 10}
            ]
        ),
        Node(
            package='motors_velocity_controller',
            executable='motors_velocity_controller',
            name='left_linear_actuator_controller',
            output='screen',
            parameters=[
                {'kp' : lineal_actuators_kp},
                {'ki' : lineal_actuators_ki},
                {'kd' : lineal_actuators_kd},
                {'use_sample_time' : False},
                {'sample_time' : 2},
                {'derivative_on_measurement' : False},
                {'remove_ki_bump' : False},
                {'reset_windup' : False},
                {'pid_enabled' : True},
                {'cut_off_freq' : 0},
                {'out_min' : lineal_actuator_out_min},
                {'out_max' : lineal_actuator_out_max},
                {'control_value_topic' : 'left_actuator/control_effort'},
                {'actual_state_topic' : 'left_actuator/actual_state'},
                {'set_point_topic' : 'left_actuator/set_point'},
                {'loop_freq' : 10}
            ]
        ),
        Node(
            package='motors_velocity_controller',
            executable='motors_velocity_controller',
            name='right_linear_actuator_controller',
            output='screen',
            parameters=[
                {'kp' : lineal_actuators_kp},
                {'ki' : lineal_actuators_ki},
                {'kd' : lineal_actuators_kd},
                {'use_sample_time' : False},
                {'sample_time' : 2},
                {'derivative_on_measurement' : False},
                {'remove_ki_bump' : False},
                {'reset_windup' : False},
                {'pid_enabled' : True},
                {'cut_off_freq' : 0},
                {'out_min' : lineal_actuator_out_min},
                {'out_max' : lineal_actuator_out_max},
                {'control_value_topic' : 'right_actuator/control_effort'},
                {'actual_state_topic' : 'right_actuator/actual_state'},
                {'set_point_topic' : 'right_actuator/set_point'},
                {'loop_freq' : 10}
            ]
        ),
        Node(
            package='platform_controller',
            executable='motor_manager',
            output='screen'
        ),
        Node(
            package='platform_controller',
            executable='platform_manager',
            output='screen'
        )
    ])
