import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    
    user_input = launch_ros.actions.Node(
            package='userinput', 
            node_executable='talker',
            output='screen')
    return launch.LaunchDescription([user_input])
   