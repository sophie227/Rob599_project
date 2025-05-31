# launch file for project 

# turtle_launch.py

# Sophie Turner

# We need to import the launch system modules.  There's a generic launch
# system, in launch, and some ROS-specific stuff, in launch_ros.
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='project',
            executable='chaser',
            name='turtle_chaser',
            ),

        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            
        ),


    ])