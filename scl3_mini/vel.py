#!/usr/bin/env python
import rospy
import roslib 
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64
import message_filters
from std_msgs.msg import Int64
import sys 

from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing

import numpy as np

pubVA = rospy.Publisher('/velA',Float64,queue_size=5)
pubVB = rospy.Publisher('/velB',Float64,queue_size=5)
pubVC = rospy.Publisher('/velC',Float64,queue_size=5)

pwmA = 0
pwmB = 0
pwmC = 0
encoder1 = 0
encoder2 = 0
encoder3 = 0

def callback1(msg):
  global velA
  global velB
  global velC
  velx = msg.linear.x*100
  vely = msg.linear.y*100
  ang = msg.angular.z
  #rospy.loginfo("velx=[%f]m/s"%(velx))
  #rospy.loginfo("vely=[%f]m/s"%(vely))
  #rospy.loginfo("ang=[%f]m/s"%(ang))
  A = np.mat('-0.25,0.25,0.0;0.433,0.433,-0.333;0.0647,0.0647,0.0647')
  #rospy.loginfo(A)
  b = np.mat('1.0,2.0,3.0').T
  b[0,0] = velx
  b[1,0] = vely
  b[2,0] = ang
  #rospy.loginfo(b)
  r = np.linalg.solve(A,b)
  #rospy.loginfo(r)
  velA = r[0,0]
  velB = r[1,0]
  velC = r[2,0] 
  pubVA.publish(velA)
  pubVB.publish(velB)
  pubVC.publish(velC)
def callback2(msg):
  global encoder1
  encoder1 = msg.data
  #rospy.loginfo("encoder1=[%f]m/s"%(encoder1))

def callback3(msg):
  global encoder2
  encoder2 = msg.data
  #rospy.loginfo("encoder2=[%f]m/s"%(encoder2))
  
def callback4(msg):
  global encoder3
  encoder3=msg.data

if __name__ =='__main__':
  rospy.init_node('cmd_vel_To_pwm')
  rate = rospy.Rate(100)
  while not rospy.is_shutdown():
    rospy.Subscriber("/cmd_vel",Twist,callback1)
    rospy.Subscriber("/encoder1_value",Float64,callback2)
    rospy.Subscriber("/encoder2_value",Float64,callback3)
    rospy.Subscriber("/encoder3_value",Float64,callback4)
    rospy.spin()
    rate.sleep()
