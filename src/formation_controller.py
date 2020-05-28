#! /usr/bin/python
import rospy
from mgr.msg import Formation
import sys


def get_formation_type():
	formation_type = 0
	if rospy.has_param('formation_type'):
		formation_type = rospy.get_param('formation_type')
	else:
		rospy.set_param('formation_type', formation_type)
	return formation_type	

def simple_formation(follower_nr, formation_publisher):
	
	formation_parameter = Formation()

	if follower_nr == 1:
		formation_parameter.Lx = 1
		formation_parameter.Ly = 0
	elif follower_nr == 2:
		formation_parameter.Lx = -1
		formation_parameter.Ly = 0	

	formation_publisher.publish(formation_parameter)


def v_formation(follower_nr, formation_publisher):
	
	formation_parameter = Formation()
	
	if follower_nr == 1:
		formation_parameter.Lx = 1
		formation_parameter.Ly = 1
	elif follower_nr == 2:
		formation_parameter.Lx = -1
		formation_parameter.Ly = 1

	formation_publisher.publish(formation_parameter)


def v2_formation(follower_nr, formation_publisher):
	
	formation_parameter = Formation()
	
	if follower_nr == 1:
		formation_parameter.Lx = 1
		formation_parameter.Ly = -1
	elif follower_nr == 2:
		formation_parameter.Lx = -1
		formation_parameter.Ly = -1
		
	formation_publisher.publish(formation_parameter)

	
def set_formation_parameter(follower_nr, formation_publisher):
		
		formation_nr = get_formation_type()
		
		if formation_nr == 0:
			simple_formation(follower_nr, formation_publisher)
		elif formation_nr == 1:
			v_formation(follower_nr, formation_publisher)
		elif formation_nr == 2:
			v2_formation(follower_nr, formation_publisher)
		elif formation_nr == 3:
			simple_formation(follower_nr, formation_publisher)
		else:
			simple_formation(follower_nr, formation_publisher)

	
def main(follower_1_name, follower_2_name):
	
	
	rospy.init_node('formation_controller')	
		
	formation_follower_1 = rospy.Publisher('formation_control_' + follower_1_name, Formation, queue_size=10)
	formation_follower_2 = rospy.Publisher('formation_control_' + follower_2_name, Formation, queue_size=10) 
	
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		
		
		set_formation_parameter(1, formation_follower_1)
		set_formation_parameter(2, formation_follower_2)
		
		rate.sleep()
		
		
		
    
if __name__ == '__main__':
	try:
		main(sys.argv[1], sys.argv[2])
	except rospy.ROSInterruptException:
		pass
