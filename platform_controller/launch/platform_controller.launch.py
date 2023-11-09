from launch import LaunchDescription
from launch_ros.actions import Node
from launch_utils.utils import launch_robot_state_publisher_node as launch_rsp_node
from launch_utils.utils import launch_rviz_node

def generate_launch_description():

    # Set the parameters for the motors velocity  PID controller
    out_min = -250
    out_max = 250
    kp = 1.6
    ki = 16.4
    kd = 0.31

    return LaunchDescription([
        launch_rviz_node(package_name='robotic_neck_viz', config_file='robotic_neck.rviz'),
        launch_rsp_node( package_name="robotic_neck_viz", xacro_file="robotic_neck.urdf.xacro"),
        Node(package='robotic_neck_viz',
            executable='neck_joint_publisher',
            name='neck_joint_publisher'
        ),
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
                {'kp' : kp},
                {'ki' : ki},
                {'kd' : kd},
                {'use_sample_time' : False},
                {'sample_time' : 2},
                {'derivative_on_measurement' : False},
                {'remove_ki_bump' : False},
                {'reset_windup' : False},
                {'pid_enabled' : True},
                {'cut_off_freq' : 0},
                {'out_min' : out_min},
                {'out_max' : out_max},
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
                {'kp' : kp},
                {'ki' : ki},
                {'kd' : kd},
                {'use_sample_time' : False},
                {'sample_time' : 2},
                {'derivative_on_measurement' : False},
                {'remove_ki_bump' : False},
                {'reset_windup' : False},
                {'pid_enabled' : True},
                {'cut_off_freq' : 0},
                {'out_min' : out_min},
                {'out_max' : out_max},
                {'control_value_topic' : 'left_motor/control_effort'},
                {'actual_state_topic' : 'left_motor/actual_state'},
                {'set_point_topic' : 'left_motor/set_point'},
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
