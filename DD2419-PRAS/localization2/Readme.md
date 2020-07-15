## First try 

roscore
roslaunch localization localization_main.launch
rviz rviz

rostopic echo /final_goal # Not necessarily needed
rosrun localization localization_main.py


## If nothing works then this is the sequence

roscore
roslaunch dd2419_simulation simulation.launch
roslaunch dd2419_simulation aruco.launch gui:=false

rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map cf1/odom
rosrun tf2_ros static_transform_publisher 0.01 0 0.02 -1.54 0 -1.54 cf1/base_link cf1/camera_link
rosrun part2 displaymapmarkers ~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/tutorial_1.world.json

rosrun map_server map_server /home/akanshu/dd2419_ws/src/localization/2Dmap.yaml

rosrun localization Readfinalgoal.py

rviz
rostopic echo /final_goal
rosrun localization localization_main.py

