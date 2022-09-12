#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import sys
from select import select

import termios, time
import tty

TwistMsg = Twist

pub = None


instructions = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

f to quit
"""


moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()
        
        twist = twist_msg
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):

    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
 
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    take_action(regions)

def take_action(regions):
#This function has been studied during the course with the professor, I simply took the general scheme and adapted it
#to the code of teleoperation_twist.py. Here I change just the value of the velocities in the case he robot is crushing

    global x 
    global y 
    global z 
    global th 

    if regions['front'] > 0.5 and regions['fleft'] > 0.5 and regions['fright'] > 0.5:
        state_description = 'case 1 - nothing'
        x = x
        th = th
    elif regions['front'] < 0.5 and regions['fleft'] > 0.5 and regions['fright'] > 0.5:
        state_description = 'case 2 - front'
        x = 0
        th = turn
    elif regions['front'] > 0.5 and regions['fleft'] > 0.5 and regions['fright'] < 0.5:
        state_description = 'case 3 - fright'
        x = 0
        th = 0.3*turn 
    elif regions['front'] > 0.5 and regions['fleft'] < 0.5 and regions['fright'] > 0.5:
        state_description = 'case 4 - fleft'
        x = 0
        th = -0.3*turn
    elif regions['front'] < 0.5 and regions['fleft'] > 0.5 and regions['fright'] < 0.5:
        state_description = 'case 5 - front and fright'
        x = 0
        th = 0.3*turn
    elif regions['front'] < 0.5 and regions['fleft'] < 0.5 and regions['fright'] > 0.5:
        state_description = 'case 6 - front and fleft'
        x = 0
        th = -0.3*turn
    elif regions['front'] < 0.5 and regions['fleft'] < 0.5 and regions['fright'] < 0.5:
        state_description = 'case 7 - front and fleft and fright'
        x = 0
        th = 0.3*turn
    elif regions['front'] > 0.5 and regions['fleft'] < 0.5 and regions['fright'] < 0.5:
        state_description = 'case 8 - fleft and fright'
        x = x
        th = th
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


if __name__=="__main__":

    print(instructions)
    
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist') #initialization of the node

    rate = rospy.Rate(5)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.5)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        while not rospy.is_shutdown():
            active_modality = rospy.get_param('/modality')
            
            if active_modality == 2 or active_modality == 3:

                if active_modality == 2:
                    print("You are in teleoperation mode \n")

                if active_modality == 3:
                    print("You are in assistive mode \n")                    
                
                while(True):
                    key = getKey(settings, key_timeout)
                    if key in moveBindings.keys():
                        x = moveBindings[key][0]
                        y = moveBindings[key][1]
                        z = moveBindings[key][2]
                        th = moveBindings[key][3]
                    elif key in speedBindings.keys():
                        speed = speed * speedBindings[key][0]
                        turn = turn * speedBindings[key][1]

                        print(vels(speed,turn))
                        if (status == 14):
                            print(instructions)
                            status = (status + 1) % 15
                    else:
                        # Skip updating cmd_vel if key timeout and robot already
                        # stopped.
                        if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                            continue
                        x = 0
                        y = 0
                        z = 0
                        th = 0

                        #if the key is 'f', then quit the modality without terminating the node
                        if (key == 'f'):
                            rospy.set_param("/modality", 4)
                            print("exiting modality... \n")
                            break

                    if active_modality == 3:
                        #sleep for 0.5 seconds so the robot has time to avoid the walls without receiving too many inputs
                        time.sleep(0.5)
                        sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
                        

                    pub_thread.update(x, y, z, th, speed, turn)


    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)