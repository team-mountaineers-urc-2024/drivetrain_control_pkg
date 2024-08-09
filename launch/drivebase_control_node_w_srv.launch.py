import launch
import launch_ros.actions

def generate_launch_description():

    launch_description = launch.LaunchDescription()

    launch_description.add_action(
            launch_ros.actions.Node(
            package='drivebase_control_pkg',
            executable='drivebase_control_node_w_srv',
            name='drivebase_control_node_w_srv')
        )
    
    return launch_description