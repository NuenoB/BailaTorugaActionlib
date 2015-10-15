#! /usr/bin/env python

import roslib;roslib.load_manifest('beginner_tutorials')
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
from actionlib.msg import *
#import roslib.rostime.Duration

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import beginner_tutorials.msg
def gotu(client,xf,yf,neworder):
    goal = beginner_tutorials.msg.BaileTorugaGoal(x=xf, y=yf, a=180, order=neworder)
    rospy.loginfo(str(xf)+" , " +str(yf))
		    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    while(client.get_state()!=GoalStatus.SUCCEEDED):
        #rospy.loginfo(str(client.get_state())+str(GoalStatus.SUCCEEDED)+str(client.get_state()!=GoalStatus.SUCCEEDED))
	client.wait_for_result(rospy.Duration(3))
        rospy.loginfo("waiteando")
def dance(client):
    while 1==1:
    	gotu(client,0,1,'')
        gotu(client,1,0,'')

def prueba_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('server_T', beginner_tutorials.msg.BaileTorugaAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    dance(client)
    

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('prueba_client_py')
        result = prueba_client()
        print "Result:"+str(result.x)+str(result.y)+str(result.a)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
