#! /usr/bin/env python

import roslib;roslib.load_manifest('beginner_tutorials')
import rospy
import actionlib_msgs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time
import math


# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import beginner_tutorials.msg

x_speed = 0.2  # velocidad 
Angle=0 # Angulo actual
maxDist = 0.2 #distancia max
Dis = 0 # distancia al punto inicial
posx = 0
posy = 0
def reciver_odom(data):
    global Angle
    global Dis
    global posx
    global posy
    rospy.loginfo
    pose=data.pose.pose
    quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
    (roll , pitch , yaw) = euler_from_quaternion(quaternion)
    tAngle = math.degrees(yaw)
    if (tAngle<0):
       tAngle = tAngle+360
    Angle = tAngle
    Dis = math.sqrt(pow(data.pose.pose.position.x,2)+pow(data.pose.pose.position.y,2))
    posx=data.pose.pose.position.x
    posy=data.pose.pose.position.y

class TorugaAction(object):
  # create messages that are used to publish feedback/result
  _feedback =  beginner_tutorials.msg.BaileTorugaFeedback()
  _result   =  beginner_tutorials.msg.BaileTorugaResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, beginner_tutorials.msg.BaileTorugaAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

  def stopAStep(self,pub):
    twist = Twist()
    rospy.loginfo("Stop "+str(twist))
    pub.publish(twist)
    time.sleep(0.5)
      
  def detGiro(self, grados):
    global Angle
    algo=grados- Angle
    if ((algo<180 and algo>0) or algo<-180):
      return 1
    else:
      return -1

  def adelante(self, pub, rate, obj):
    global Dis
    #Dis = obj
    twist = Twist()
    twist.linear.x = x_speed;
    twist.angular.z = 0;
    counter = 0
    #obj=Dis+obj
    while ((not rospy.is_shutdown()) and (abs(obj-Dis)>0.1)):
        #rospy.loginfo("obj: " + str(obj) + " Dis: " + str(Dis) +" delta: " + str(abs(obj-Dis)))
        #rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()
    self.stopAStep(pub)

  def atras(self, pub, rate, obj):
    global Dis
    #Dis = obj
    twist = Twist()
    twist.linear.x =-x_speed;
    twist.angular.z = 0;
    counter = 0
    #obj=Dis+obj
    while ((not rospy.is_shutdown()) and (abs(obj-Dis)>0.1)):
        #rospy.loginfo("obj: " + str(obj) + " Dis: " + str(Dis) +" delta: " + str(abs(obj-Dis)))
        #rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()
    self.stopAStep(pub)

  def girar(self, pub, rate, obj):
    twist = Twist()
    if (obj<0):
      obj=obj+360
    global Angle
    twist.linear.x=0.0
    twist.linear.y=0.0
    twist.angular.z=0.6*self.detGiro(obj)
    while ((not rospy.is_shutdown()) and (abs(obj-Angle)>5.0)):
      rospy.loginfo("obj: " + str(obj) + " angulo: " + str(Angle) +" delta: " + str(abs(obj-Angle)))
      #rospy.loginfo(twist)
      pub.publish(twist)
      rate.sleep()
    self.stopAStep(pub)


  def execute_cb(self, goal):
    global Angle
    global Dis
    global pos
    # helper variables
    r = rospy.Rate(10)
    success = True
    rospy.loginfo( "Result:"+str(goal.x)+str(goal.y)+str(goal.a))
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
    rospy.loginfo("obj ?= " + str(goal.y) + " , " + str(posy) + " , " + str(goal.x) + " , " + str(posx) )
    angl = math.degrees(math.atan2(goal.y ,goal.x ))
    rospy.loginfo(angl)
    if (goal.order=='atras'):
      self.atras(pub,r,math.sqrt(pow(goal.x + posx,2)+pow(goal.y+ posy,2)))
    else:
      self.girar(pub, r,angl)
      self.adelante(pub,r,math.sqrt(pow(goal.x + posx,2)+pow(goal.y+ posy,2)))
    #self.girar(pub, r,goal.a)
    #self._feedback.APosX = Dis[0]
    #self._feedback.APosY = Dis[1]
    #self._feedback.APosAngle = Angle
    
    # publish info to the console for the user
    #rospy.loginfo("delta = " self._feedback.APosX )
    #if self._as.is_preempt_requested():
	#rospy.loginfo("recivi strign " + goal.doSomething)
        
    # start executing the action
    #for i in xrange(1, goal.order):
      # check that preempt has not been requested by the client
      #if self._as.is_preempt_requested():
        #rospy.loginfo('%s: Preempted' % self._action_name)
        #self._as.set_preempted()
        #success = False
        #break
      #self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
      # publish the feedback
      #self._as.publish_feedback(self._feedback)
      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes

      
    if success:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      
      self._as.set_succeeded(goal)


if __name__ == '__main__':
  rospy.init_node('server_T')
  rospy.Subscriber('odom',Odometry, reciver_odom)
  TorugaAction(rospy.get_name())
  rospy.spin()


