#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  6 20:36:29 2021

@author: Glenn Maes

Listener script: ROS node intended to give an endpose to the invariants_ros script 
by using the endpose of the HTC Vive tracker and the transformation between the
Vive lighthouses and the robot base, obtained after calibration is done.
"""

import rospy
import tf
import tf_conversions as tf_c
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
import time
import numpy as np

def getFrameName(listener,frame_string):
    """
    Ask the user to select the requested frame in /tf
    
    Parameters:
        listener: tf listener
        frame_string: description of requested frame
    
    Returns:
        String: name of requested frame inside /tf
    
    """
    
    index = 0
    listFrames = []

    for frame in listener.getFrameStrings():
        print(str(index) + ': ' +  frame)
        index += 1
        listFrames.append(frame)
    
    if index==0:
       raise Exception("List of frames is empty. Check whether /tf topic is available.") 
    
    index = input('Select the ' + frame_string + ' using the numbers above: ')
    print "You selected: ",listFrames[index]
    print ""
    
    return listFrames[index]



if __name__ == '__main__':
    
    print("Start ROS node for endpose tracker")
    rospy.init_node('endpose_tracker', anonymous=True)
    tracker_endpose = rospy.Publisher("target_pose_pub", Pose, queue_size=1)
    listener = tf.TransformListener()
    
    print("Waiting for something to appear on /tf topic...")
    rospy.wait_for_message('/tf', TFMessage)
    time.sleep(0.1)
    print " "
    
    robot_base_frame = getFrameName(listener, "robot base frame")
    camera_frame = getFrameName(listener, "camera frame")
    tracker_frame = getFrameName(listener, "tracker frame")
    
    while not rospy.is_shutdown():
        now = rospy.Time.now()       
        #T_b_c = listener.lookupTransform(robot_base_frame, camera_frame, rospy.Time(0))
        #T_c_t = listener.lookupTransform(camera_frame, tracker_frame, rospy.Time(0))
        
        # Denk dat de 2 hierboven niet nodig zijn, dat je dat in 1 keer kan transformeren
        T_b_t = listener.lookupTransform(robot_base_frame, tracker_frame, rospy.Time(0))
        
        # Neem voorlopig enkel positie data over
        mat_T_b_t = tf_c.toMatrix(tf_c.fromTf(T_b_t))
        #temp = np.eye(4)
        #temp[0:2,3] = mat_T_b_t[0:2,3]
        
        temp = tf_c.Frame(tf_c.Rotation.EulerZYX(0.0, np.pi/4, np.pi), tf_c.Vector(mat_T_b_t[0,3], mat_T_b_t[1,3], mat_T_b_t[2,3]+0.30))

        #endpose_msg = tf_c.toMsg(tf_c.fromTf(T_b_t))
        endpose_msg = tf_c.toMsg(tf_c.fromMatrix(temp))
        tracker_endpose.publish(endpose_msg)
    
    
    
    
    
    
