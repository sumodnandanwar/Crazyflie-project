# Crazyflie-project

A Project course in Robotics and Autonomous systems. It involves autonomous drone control using various aspects in robotics such as "Localozation", "Path Planning" and "Object Detection"

To run the project:

gitclone the DD2419-PRAS dir 
catkin build

roslaunch localization localization_main.launch
roslaunch dd2419_launch base.launch ch:=89

rosrun localization Nav0_1resworked.py
rosrun localization Detect_trig.py (optional)
