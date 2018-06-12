import rospy
import math

from std_msgs.msg import Float64
from math import *


#Define a RRBot joint positions publisher for joint controllers.
def humanoid_joint_positions_publisher():

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('robot_state_publisher', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/humanoid/hiprroll_position_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/humanoid/hiplroll_position_controller/command', Float64, queue_size=10)

	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():

		#Have each joint follow a sine movement of sin(i/100).
		sine_movement = sin(i/100.)

		#Publish the same sine movement to each joint.
		pub1.publish(sine_movement)
		pub2.publish(sine_movement)

		i = i+1 #increment i

		rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: humanoid_joint_positions_publisher()
	except rospy.ROSInterruptException: pass
