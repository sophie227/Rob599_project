# Rob599 Project 
The project is to spawn 5 turtles. Turtle 1 will follow the turtle 5 by default. A service can change which turtle will be followed. 

To run the code:
source install/setup.bash 
ros2 launch project turtle_launch.py

To change the turtle being followed: 
ros2 service call /set_follow_target project_msg/srv/TurtleFollower "{name: 'turtle2'}"

