#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('turtlebot_controller', anonymous=False)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed = 1
    radius = 1
    omega = speed/radius
    distance = 2*math.pi*radius

    #Checking if the movement is forward or backwards
    vel_msg.linear.x= speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = omega
    print(omega)

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while(current_distance < distance):
        print(current_distance)
        #Publish the velocity
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass