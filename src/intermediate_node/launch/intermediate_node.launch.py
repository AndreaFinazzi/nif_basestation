from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    intermediate = launch_ros.actions.Node(
        package = 'intermediate_node',
        executable = 'intermediate_node',
        output = 'screen'
    )

    return LaunchDescription([intermediate])
