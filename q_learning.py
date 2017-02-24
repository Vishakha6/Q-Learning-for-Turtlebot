#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import random as rand
from sys import exit
global move_to_pose
move_to_pose={'x':-1,'y':-1}

# Q learning 
q_size=6
learning_rate=0.8
iterations=200
initial_states=np.array([0,2,3,4,5,1])

reward_matrix=np.matrix('-1 0 -1 -1 -1 -1; -1 -1 0 -1 -1 -1; -1 -1 -1 0 -1 -1; -1 -1 -1 -1 0 -1; -1 -1 -1 -1 -1 10; 0 -1 -1 -1 -1 10')

q_values=np.zeros((q_size, q_size))
current_state=0

def check(ini_state):
    global current_state
    current_state=ini_state
    get_action()
    while current_state==5:
        
        get_action()
        
        if current_state==5:
            break
        
        for index in range(0,q_size):
            get_action()

def get_action():
    
    possible_action=get_random_action(q_size,0)
    global current_state
    if rewards_matrix[current_state, possible_action]>=0:
        q_values[current_state,possible_action]=rewards_matrix[current_state,possible_action]+learning_rate*maximum_q(action,False)
        current_state=possible_action

def get_random_action(upper_bound, lower_bound):
    
    while True:
        global current_state
        action=lower_bound+rand.randint(0,5)
        if rewards_matrix[current_state, action]>-1:
             valid_choice=True
        else:
            valid_choice=False

        if valid_choice==True:
            break
            
    return action

def maximum_q(state, boolean):
    final=0
    done=False
    while True:
        new_action=False
        for index in range(0,q_size):
            if ((index<final) or (index>final)):
                 if q_values[state,index]>q_values[state,final]:
                     final=index
                     new_action=True
            if new_action==False:
                done=True
        if done==True: 
            break
    if boolean==True:
        return final
    else:
        return q_values[state,final]

# Move_base Algorithm for path panning

class GoToPose():
    def __init__(self):

        self.goal_sent = False
	rospy.on_shutdown(self.shutdown)
	
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        self.move_base.send_goal(goal)

	success = self.move_base.wait_for_result(rospy.Duration(40)) 

        state = self.move_base.get_state()
        result = False;

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def get_pose(state,pose):
	
	position = pose
	if state==0:
		position['x']=-2.5
		position['y']=1
	elif state==1:
		position['x']=3
		position['y']=-3
	elif state==2:
		position['x']=2.8
		position['y']=3.4
	elif state==3:
		position['x']=-4.8
		position['y']=8.96
	elif state==4:
		position['x']=-9.3
		position['y']=2.2
	elif state==5:
		position['x']=-8
		position['y']=-6
	return position

# Main code combining move_base with state values from q learning algorithm
if __name__ == '__main__':
    try:
	rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        for index1 in range(0,iterations-1):
              for index2 in range(0,q_size-1):        
	            temp=initial_states[index2]
	            check(temp)	
	
        for index in range(0,q_size):
            current_state=initial_states[index]
            new_state=0
			move_to_pose=get_pose(current_state, move_to_pose)	

            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
            rospy.loginfo("Go to (%s, %s) pose", move_to_pose['x'], move_to_pose['y'])
            success = navigator.goto(move_to_pose, quaternion) 

            if success:
            	rospy.loginfo("Hooray, reached the desired pose")
				rospy.loginfo("Current State is %s", current_state)
            else:
            	rospy.loginfo("The base failed to reach the desired pose")

            while True:
                new_state=maximum_q(current_state, True)
                current_state=new_state
				move_to_pose=get_pose(current_state, move_to_pose)	
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
                rospy.loginfo("Go to (%s, %s) pose", move_to_pose['x'], move_to_pose['y'])
                success = navigator.goto(move_to_pose, quaternion) 

        	if success:
            		rospy.loginfo("Hooray, reached the desired pose")
			rospy.loginfo("Current State is %s", current_state)
        	else:
            		rospy.loginfo("The base failed to reach the desired pose")
		
                print current_state
                if current_state==5:
                    exit()

        rospy.sleep(1)

    except rospy.ROSInterruptException:
	rospy.loginfo("Ctrl-C caught. Quitting")

