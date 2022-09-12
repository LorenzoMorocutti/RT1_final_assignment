#! /usr/bin/env python

# import ros stuff
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
import math


def client_init():
#function needed to initialize the client to perform the actions of MoveBase

	global autonomous_client
	global goal

	autonomous_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	autonomous_client.wait_for_server()

	goal = MoveBaseGoal()  
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.orientation.w = 1.0


def clbk_odom(msg):
    global position_
    position_ = msg.pose.pose.position

def update_goal():
#very simple fuction that request the coordinates we want the robot to reach

    global desired_x, desired_y
    
    x = float(input("Insert X coordinate: "))
    y = float(input("Insert Y coordinate: "))
    
    desired_x = x
    desired_y = y

def set_goal():
#function to assign our desired coordinates to the target_pose field and
#to "send" the goal to start moving the robot

    goal.target_pose.pose.position.x = desired_x
    goal.target_pose.pose.position.y = desired_y
    print("\nDriving to tha goal... \n")
    autonomous_client.send_goal(goal)

def cancel_goal():
#function to cancel the goal, performed again through an action

    print("The goal has been canceled. \n")
    autonomous_client.cancel_goal()

def timeout_event():
#simple function that print that there has been a problem to reach the destination
#after a certain amount of time chosen by the programmer

    if active_modality == 1:
        print("\nTime has expired, the goal is too far or unreachable. \n")
        cancel_goal()


if __name__=='__main__':

    #initialization of the node and subscriber to the odom topic
    rospy.init_node('autonomous_node')
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    client_init()

    rate = rospy.Rate(5)

    #variable used to request another goal after the first cycle of the while loop
    do_after_first = False

    while not rospy.is_shutdown():

        #check in which modality we are operating
        active_modality = rospy.get_param("/modality")

        #if construct to request another goal after we have set the first one
        if do_after_first == True:
            another_goal = str(input("Do you want to set another goal? ('y' for yes/'n' for no): "))
                
            if another_goal == 'n':
                #if we don't want to set another goal, we return to idle modality
                rospy.set_param("/modality", 4)
                break

        if active_modality == 1:

            update_goal()

            set_goal()

            #variable used to check if the user want to cancel the current goal
            cancel = True

            #starting time of our timer
            start = time.time()

            #if the robot takes more than 30 seconds to reach the goal, it means that the the goal is unreachable or too far
            while time.time()-start < 30:
                robot_state = autonomous_client.get_state()  #state 0 = accepted goal, state 1 = driving, state 3 = goal reached

                if robot_state == 0 or robot_state == 1:

                    if cancel == True:
                        choice = str(input("Do you want to cancel goal? (Mind that, after the robot has arrived, the choice will be useless)(y/n): "))
                        if choice == 'y':
                            cancel_goal()
                            break
                        else:
                            cancel = False
                    
                elif robot_state == 3:
                    print("Robot has arrived. \n")
                    break

            #if after 30 seconds the robot is still in state 1, cancel the goal
            if robot_state == 1:
                timeout_event()

            do_after_first = True

        rate.sleep()

