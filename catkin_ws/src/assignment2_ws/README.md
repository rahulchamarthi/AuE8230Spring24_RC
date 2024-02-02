
# Assignment 2a

This is my readme file for Assignment 2a for AuE 8230 offered at Clemson University in the Spring of 2024.  

# Task 1: Configure GIT 
The success of this task can be seen by the fact that you are reading this file. 

I pushed this readme file, along with all the additional files, as part of a push from my local machine. 

# Task 2: Creating a ROS Workspace, and a Turtlesim Operator
Our goal here was to create a ROS Workspace, and then to create two python scripts that would be responsible for manipulating the Turtlesim in specific path patterns. 

**circle.py:** a python file that contains code for moving the turtlesim in a circle with a user specified linear, and angular velocity. 

**square_openloop.py:** a python file that contains code for moving the turtlesim in a 2x2 square, at 0.2 linear velocity and 0.2 rad/s angular velocity. 

Both python scripts can be run by executing the roslaunch command on the included launch file. The command should be as follows:
```bash
  roslaunch assignment2_ws assignment2_ws.launch
```

The result will look something like the following; 2 windows will open with one executing the square_oneloop script, and the other executing the circle script. 

