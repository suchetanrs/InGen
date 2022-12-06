To run line follower, follow these steps
1. Ensure TURTLEBOT3 is installed on your system. This code is tested with ROS Melodic and catkin_make.
2. In my case, my workspace name is ```Ingen_ws```, replace it with your workspace when you run the following commands
3. ```source ~/Ingen_ws/devel/setup.bash```
4. ```roslaunch line_follower_pkg yellow_line_world.launch```
5. ```cd ~/Ingen_ws/src/line_follower_pkg/src/```
6. ```chmod +x follower_script.py```
7. ```rosrun line_follower_pkg follower_script.py```
