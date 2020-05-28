#! /usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Int32
import sys


def main(model_name):
	rospy.init_node('odom_pub')
	print(model_name)

	odom_pub=rospy.Publisher ("/position_" + model_name, Odometry, queue_size=10)

	rospy.wait_for_service ('/gazebo/get_model_state')
	get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

	odom=Odometry()
	header = Header()
	header.frame_id='/odom'

	model = GetModelStateRequest()
	model.model_name=model_name

	r = rospy.Rate(100)

	while not rospy.is_shutdown():
		result = get_model_srv(model)

		odom.pose.pose = result.pose
		odom.twist.twist = result.twist

		header.stamp = rospy.Time.now()
		odom.header = header

		odom_pub.publish (odom)
		print(model_name)

		r.sleep()

	
    
if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except rospy.ROSInterruptException:
		pass
