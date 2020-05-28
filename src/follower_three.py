#! /usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Int32
from math import pow, atan2, sqrt
import sys

global set_x_position 
global set_y_position
global set_z_position
global act_x_position 
global act_y_position 
global act_z_position 


def callback_actual_position(potential):
	global x_potential
	global y_potential
	global z_potential

	x_potential = potential.linear.x
	y_potential = potential.linear.y
	z_potential = potential.linear.z
	
def max_velocity(velocity):
	if velocity > 4:
		return 4
	elif velocity < -4:
		return -4
	else:
		return velocity

def main(follower_name):
	global x_potential
	global y_potential
	global z_potential
	 
	x_potential = 0
	y_potential = 0
	z_potential = 0
	
	rospy.init_node('follower_publisher_' + follower_name)	
	
	sub_set_position = rospy.Subscriber('potentail_field_' + follower_name, Twist, callback_actual_position)
	
	pub = rospy.Publisher('/'+ follower_name + '/cmd_vel', Twist, queue_size=10)
	
	
	cmd_p = 0.2
	if rospy.has_param('cmd_p'):
		cmd_p = rospy.get_param('cmd_p')
	else:
		rospy.set_param('cmd_p', cmd_p)
	
	
	
	#rospy.spin()
	r = rospy.Rate(100)
	
	while not rospy.is_shutdown():
		sub_set_position = rospy.Subscriber('potentail_field_' + follower_name, Twist, callback_actual_position)
		
		# Tworzenie waidomosci Twist
		msg_twist_cmd_vel = Twist()
		msg_twist_cmd_vel.linear.x = 0
		msg_twist_cmd_vel.linear.y = 0
		msg_twist_cmd_vel.linear.z = 0
		msg_twist_cmd_vel.angular.x = 0
		msg_twist_cmd_vel.angular.y = 0
		msg_twist_cmd_vel.angular.z = 0
		
		cmd_p = rospy.get_param('cmd_p')
		
		x_set = x_potential
		y_set = y_potential
		z_set = z_potential
		
		x_cmd = 0
		y_cmd = 0
		
		## Roznica miedzy wartoscia zadana a aktualna
		#dif = Pose()
		#dif.position.x = x_distance_diff
		#dif.position.y = y_distance_diff
		#dif.position.z = z_distance_diff
		#pub_dif.publish(dif) # Wyslanie danych na topic
		
		# Regulator pitch
		if x_set > 0.2:
			x_cmd = x_set * cmd_p
		if x_set < -0.2:
			x_cmd = x_set * cmd_p
		
		
		# Regulator roll
		if y_set > 0.2:
			y_cmd = y_set * cmd_p
		if y_set < -0.2:
			y_cmd = y_set * cmd_p

		
		if z_set > 0.1:
			msg_twist_cmd_vel.linear.z = 0.2 * z_set
		if z_set < -0.1:
			msg_twist_cmd_vel.linear.z = 0.2 * z_set

		msg_twist_cmd_vel.linear.x = max_velocity(x_cmd)
		msg_twist_cmd_vel.linear.y = max_velocity(y_cmd)
		
		pub.publish(msg_twist_cmd_vel) # Wyslanie danych na topic
	
		x_act = "x_set: %s"%x_set
		x_zad = "y_set: %s"%y_set
		x_cmd = "x_cmd: %s"%msg_twist_cmd_vel.linear.x
		y_cmd = "y_cmd: %s"%msg_twist_cmd_vel.linear.y
		print("-------------------------")
		print("     ")
		rospy.loginfo(x_act)
		rospy.loginfo(x_zad)
		rospy.loginfo(x_cmd)
		rospy.loginfo(y_cmd)
		
		r.sleep()
	
		
		
		
    
if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except rospy.ROSInterruptException:
		pass
