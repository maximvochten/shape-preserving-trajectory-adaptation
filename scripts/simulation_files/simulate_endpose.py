#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 27 09:15:27 2021

@author: Glenn Maes

This ROS node is used to simulate an endpose, which is used as constraint in the invariant trajectory calculation.
"""

import rospy
import numpy as np
import tf_conversions as tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
#import time

if __name__ == '__main__':
    
    # Initialize ROS node 
    rospy.init_node("simulated_end_pose", anonymous=True)
    pub = rospy.Publisher("target_pose_pub", Pose, queue_size=1)
    vis_pub = rospy.Publisher("visual_target_pub", PoseStamped, queue_size=1)
    vel_pub = rospy.Publisher("target_vel_pub", Twist, queue_size=1)
    
    rate = rospy.Rate(60.0)
    # Set endpose
    #  TODO: more elaborate calculation than just setting a pose
    #endpose = tf.Frame(tf.Rotation.EulerZYX(0.0, 0.0, np.pi/6), tf.Vector(0.3, -0.2, 0.0))
    
    #endpose = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0), tf.Vector(-0.6, 0.2, 0.3))
    # Franka Panda
    #endpose = tf.Frame(tf.Rotation.EulerZYX(90*np.pi/180, 0.0*np.pi/180, 90*np.pi/180), tf.Vector(0.7, -0.1, 0.4))
    endpose = tf.Frame(tf.Rotation.EulerZYX(60*np.pi/180, 0.0, np.pi), tf.Vector(0.5,0.0,0.3))

    theta = 0
    radius = 0.1
    # KUKA LWR
    #endpose = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0), tf.Vector(-0.6, -0.2, 0.3))
    
    print "Publishing simulated /target_pose_pub data"
    
    while not rospy.is_shutdown():
        #if (abs(np.sin(theta)) <= 1e-2):
        #        rospy.loginfo('now')

        theta += 0.01
        #endpose = tf.Frame(tf.Rotation.EulerZYX(0.0, np.pi/4, np.pi), tf.Vector(0.2+radius*np.sin(theta), -0.3, 0.2)) #0.2+radius*np.sin(theta)
        linear_vel = Vector3()
        #linear_vel.x = radius*np.cos(theta)
        linear_vel.x = radius*0.6*np.cos(theta)
        #linear_vel.y = 0.0
        #linear_vel.z = 0.0
         
        #endpose = tf.Frame(tf.Rotation.EulerZYX(theta, np.pi/6, np.pi), tf.Vector(0.2, -0.3, 0.2))
        
        visual_target = PoseStamped()
        twist = Twist()
        
        twist.linear = linear_vel

        # Put endpose data on topic
        visual_target.header.stamp = rospy.Time.now()
        visual_target.header.frame_id = "panda_link0"
        endpose_msg = tf.toMsg(tf.fromMatrix(endpose))
        visual_target.pose = endpose_msg
        pub.publish(endpose_msg)
        vis_pub.publish(visual_target)
        #vel_pub.publish(twist)
        
        rate.sleep()
