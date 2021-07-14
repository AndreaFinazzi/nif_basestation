from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    intermediate = launch_ros.actions.Node(
        package = 'intermediate_node',
        executable = 'intermediate_node',
        output = 'screen', 
        remappings=[
            ("/joystick/steering_cmd", "/joystick/steering_cmd_not_used"),
            ("/joystick/accelerator_cmd", "/joystick/accelerator_cmd_max")
        ]
    )

    return LaunchDescription([intermediate])
