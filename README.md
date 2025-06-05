# Rob599 Project 
This code runs on ROS2 Jazzy and uses Turtlesim simulation. 
The project is to spawn 5 turtles. Turtle 1 will follow the turtle 5 by default. A service can change which turtle will be followed. chaser.py is the code used in the launch file. 

Install Turtlesim: 
sudo apt update
sudo apt install ros-jazzy-turtlesim

To run Turtlesim: 
ros2 run turtlesim turtlesim_node

To run the code:

source install/setup.bash 

ros2 launch project turtle_launch.py

To change the turtle being followed: 

ros2 service call /set_follow_target project_msg/srv/TurtleFollower "{name: 'turtle2'}"

