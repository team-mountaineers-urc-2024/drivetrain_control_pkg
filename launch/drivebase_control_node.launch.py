import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    launch_description = launch.LaunchDescription()
    
    launch_description.add_action(
        DeclareLaunchArgument(name = 'debugging', default_value = 'False', description = 'sets debug value to true or false')
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'can_topic_name', default_value ='outgoing_can_commands', description = 'name of outgoing can topic')   
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'front_left_motor_arbitration_id', default_value = '0x144', description = 'arbitration id for left front motor')#ID 1   
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'back_left_motor_arbitration_id', default_value = '0x143', description = 'arbitration id for left back motor')#ID 2  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'front_right_motor_arbitration_id', default_value = '0x142', description = 'arbitration id for front right motor')#ID 3  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'back_right_motor_arbitration_id', default_value = '0x141', description = 'arbitration id for back right motor')#ID 4  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'gear_reduction_ratio', default_value = str(1/35), description = 'gear reduction ratio for drive motors')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'track_width', default_value = '1.0', description = 'track width of drive motors')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'wheel_radius', default_value = '1.0', description = 'radius of wheels')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'cmd_vel_topic', default_value = 'cmd_vel', description = 'command velocity topic')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'max_linear_speed_mps', default_value = '10.0', description = 'max linear speed in meters per seconds')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'joy_topic', default_value = 'joy', description = 'topic for joystick inputs')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'enable_button', default_value = '2', description = 'enable Xbox X button')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'enable_turbo_button', default_value = '5', description = 'enable Xbox Right Trigger')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'isHeimdall', default_value = 'False', description = 'Is the robot being run heimdall?')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'doHeartbeat', default_value = 'True', description = 'Is the robot running by itself?')  
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'internal_zeroing', default_value = 'True', description = 'Is the robot running by itself?')  
    )

    launch_description.add_action(
            launch_ros.actions.Node(
            package='drivebase_control_pkg',
            executable='drivebase_control_node',
            name='drivebase_control_node',
            parameters=[
                {'debugging' : LaunchConfiguration('debugging')},
                {'can_topic_name' : LaunchConfiguration('can_topic_name')},
                {'front_left_motor_arbitration_id' : LaunchConfiguration('front_left_motor_arbitration_id')},
                {'back_left_motor_arbitration_id' : LaunchConfiguration('back_left_motor_arbitration_id')},
                {'front_right_motor_arbitration_id' : LaunchConfiguration('front_right_motor_arbitration_id')},
                {'back_right_motor_arbitration_id' : LaunchConfiguration('back_right_motor_arbitration_id')},
                {'gear_reduction_ratio' : LaunchConfiguration('gear_reduction_ratio')},
                {'track_width' : LaunchConfiguration('track_width')},
                {'wheel_radius' : LaunchConfiguration('wheel_radius')},
                {'cmd_vel_topic' : LaunchConfiguration('cmd_vel_topic')},
                {'max_linear_speed_mps' : LaunchConfiguration('max_linear_speed_mps')},
                {'joy_topic' : LaunchConfiguration('joy_topic')},
                {'enable_button' : LaunchConfiguration('enable_button')},
                {'enable_turbo_button' : LaunchConfiguration('enable_turbo_button')},
                {'isHeimdall' : LaunchConfiguration('isHeimdall')},
                {'doHeartbeat' : LaunchConfiguration('doHeartbeat')},
                {'internal_zeroing' : False}
            ]
        )
    )

    return launch_description