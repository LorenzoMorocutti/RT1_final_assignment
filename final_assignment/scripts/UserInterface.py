#! /usr/bin/env python

from std_srvs.srv import *
import math, sys
import roslib, rospy, smach, smach_ros


#define state Idle
class Idle(smach.State):
	def __init__(self):
		smach.State.__init__(self,
							 outcomes=['outcome1', 'outcome2', 'outcome3'])

	def execute(self, userdata):
	#here we execute the code of idle, that is the choice of the operative modality of the robot

		sm_modality = int(input('Choose the modality: '))
		if sm_modality == 1:
			return 'outcome1'
		elif sm_modality == 2:
			return 'outcome2'
		elif sm_modality == 3:
			return 'outcome3'

#define state autonomous
class Autonomous(smach.State):
	def __init__(self):
		smach.State.__init__(self,
							 outcomes=['outcome4'])

	def execute(self, userdata):
	#here we set the modality for the first type of movement (autonomous driving)

		print("Select the destination you want the robot to reach. \n")

		rospy.set_param("/modality", 1)
				
		while(True):			
			if rospy.get_param("/modality") == 4:
				print("I've exited AUTONOMOUS modality")
				break

		return 'outcome4'

#define state teleoperation
class Teleoperation(smach.State):
	def __init__(self):
		smach.State.__init__(self,
							 outcomes=['outcome4'])

	def execute(self, userdata):
	#here we set the modality for the second type of movement (teleoperation keyboard)

		rospy.set_param("/modality", 2)

		while(True):
			if rospy.get_param("/modality") == 4:
				print("I've exited TELEOPERATION modality \n")
				break

		return 'outcome4'	


class Assistive(smach.State):
	def __init__(self):
		smach.State.__init__(self,
							 outcomes=['outcome4'])

	def execute(self, userdata):
	#here we set the modality for the last type of movement (assisitive modality)
	
		rospy.set_param("/modality", 3)

		while(True):
			if rospy.get_param("/modality") == 4:
				print("I've exited ASSISTIVE modality \n")
				break

		return 'outcome4'	


def main():

	print("Welcome to the starting menù! \n")
	print("I will now request you to choose between several modalities to drive the robot. \n")
	print("Please, press the <1> key, if you want the robot to autonomously reach the target. \n")
	print("Please, press the <2> key, if you want to directly drive the robot, with the keyboard, by yoursel. \n")
	print("Please, press the <3> key, if you want to drive the robot by yourself but with my help to avoid unwanted collisions against the wall. \n")	

	rospy.init_node('state_machine_modalities')

	#create state machine
	sm = smach.StateMachine(outcomes=['exit'])

	#open the container
	with sm:
		#add the states
		smach.StateMachine.add('idle', Idle(),
								transitions={'outcome1':'autonomous',
											 'outcome2':'teleoperation',
											 'outcome3':'assistive'}
								)

		smach.StateMachine.add('autonomous', Autonomous(),
								transitions={'outcome4':'idle'}
								)

		smach.StateMachine.add('teleoperation', Teleoperation(),
								transitions={'outcome4':'idle'}
								)
							
		smach.StateMachine.add('assistive', Assistive(),
								transitions={'outcome4':'idle'}
								)

	#execute smach plan
	outcome = sm.execute()


if __name__ == "__main__":
	main()
 