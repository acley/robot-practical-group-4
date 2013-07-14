#! /usr/bin/env python

import roslib; roslib.load_manifest('nao_world_msgs')
import rospy
import actionlib
import sys
import motion
import time
import math
from naoqi import ALProxy
from nao_world_msgs.msg import *

class WalkActionServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('/nao_world_msgs/WalkActionServer', WalkAction, self.execute, False)
    self.server.start()
    rospy.loginfo("Walk Action Server started.")

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    self.walk(goal)
    self.server.set_succeeded()
    
  def walk(self, goal):
    # Init proxies.
    try:
        motionProxy = ALProxy("ALMotion", "192.168.105.15", 9559)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e

    # Set NAO in Stiffness On
    self.StiffnessOn(motionProxy)

    # Send NAO to Pose Init
    #motionProxy.walkInit()

    #####################
    ## Enable arms control by Walk algorithm
    #####################
    motionProxy.setWalkArmsEnabled(True, True)
    
    names = list()
    times = list()
    keys = list()
    
    names.append("LShoulderPitch")
    times.append([0.50000])
    keys.append([0.12728])

    names.append("LShoulderRoll")
    times.append([0.50000])
    keys.append([-0.06447])

    names.append("LElbowYaw")
    times.append([ 0.50000])
    keys.append([-0.01231])

    names.append("LElbowRoll")
    times.append([ 0.50000])
    keys.append([-0.05211])

    names.append("LWristYaw")
    times.append([ 0.50000])
    keys.append([-0.03072])

    names.append("LHand")
    times.append([ 0.50000])
    keys.append([0.00020])

    names.append("RShoulderPitch")
    times.append([ 0.50000])
    keys.append([0.09208])

    names.append("RShoulderRoll")
    times.append([ 0.50000])
    keys.append([0.06899])

    names.append("RElbowYaw")
    times.append([ 0.50000])
    keys.append([-0.04913])

    names.append("RElbowRoll")
    times.append([ 0.50000])
    keys.append([0.05680])

    names.append("RWristYaw")
    times.append([ 0.50000])
    keys.append([-0.01078])

    names.append("RHand")
    times.append([ 0.50000])
    keys.append([0.00017])
    
    #motionProxy.angleInterpolation(names, keys, times, True);


    #####################
    ## FOOT CONTACT PROTECTION
    #####################
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

    gait_config = [["MaxStepX", 0.04], ["MaxStepY", 0.14], ["MaxStepTheta", 0.35], ["MaxStepFrequency", 0.5], ["StepHeight", 0.019999999552965164], ["TorsoWx", 0.0], ["TorsoWy", -0.5]]

    motionProxy.walkTo(goal.distance, 0, 0, gait_config)
    
  def StiffnessOn(self, proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)
    proxy.setFallManagerEnabled(0)
    
if __name__ == '__main__':
  rospy.init_node('WalkActionServer')
  server = WalkActionServer()
  rospy.spin()
