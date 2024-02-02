#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
import math


class TurtleBot: 
	def __init__(self): 
		rospy.init_node('turtlebot_controller', anonymous=True) 
		
		self.velocity_publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10) 
		
		self.pose_subscriber = rospy.Subscriber('turtle1/pose', Pose, self.update_pose)
		
		self.pose = Pose() 
		
		self.rate = rospy.Rate(1)
		
	def update_pose(self, data): 
		self.pose = data
		self.pose.x = round(self.pose.x, 4) 
		self.pose.y = round(self.pose.y, 4) 
	
	def forward(self): 
		#knowing that linear velocity is in m/s, we can execute the following equation: 
		#0.2m/s * 10 seconds = 2 meters; which is the length of one side 
		
		#create the velocity message
		vel_msg = Twist()   
		vel_msg.linear.x = 0.2
		
		#move forward for 10 seconds 
		start_time = rospy.Time.now().to_sec() 
		while (rospy.Time.now().to_sec() - start_time < 10) and not rospy.is_shutdown(): 
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep() 
			
		#stop the turtle 
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg) 
		
	def rotate(self): 
		#rotate 90 degrees using the same calculation logic above, but with angular velocity
		vel_msg = Twist() 
		
		#convert the angular speed and relative angle to proper units 
		angular_speed = 0.2 #already in radians/sec 
		relative_angle = 90 * 2 * math.pi/360 
		
		#linear components not necessary so 0 them out 
		vel_msg.linear.x = 0 
		vel_msg.linear.y = 0 
		vel_msg.linear.z = 0 
		
		#also 0 out x and y angular changes
		vel_msg.angular.x = 0 
		vel_msg.angular.y = 0 
		
		#set angular speed 
		vel_msg.angular.z = abs(angular_speed) 
		
		start_time = rospy.Time.now().to_sec() 
		current_angle = 0 
		
		while(current_angle < relative_angle): 
			self.velocity_publisher.publish(vel_msg)
			current_time = rospy.Time.now().to_sec() 
			current_angle = angular_speed * (current_time - start_time) 
		
		#stop the turtle 
		vel_msg.angular.z = 0 
		self.velocity_publisher.publish(vel_msg) 
		
	def square_movement(self): 
		#draw line for base of square
		self.forward() 
		
		#rotate the turtle 90 degrees 
		self.rotate() 
		
		#draw right side of square 
		self.forward() 
		
		#rotate the turtle 90 degrees 
		self.rotate() 
		
		#draw top of the square 
		self.forward() 
		
		#rotate the turtle 90 degrees
		self.rotate() 
		
		#draw the left side of the square
		self.forward() 
		
		#rotate the turtle back to starting position 
		self.rotate() 
		
if __name__ == '__main__':
	try: 
		x = TurtleBot() 
		x.square_movement()  
	except rospy.ROSInterruptException: 
		pass 
		
		
