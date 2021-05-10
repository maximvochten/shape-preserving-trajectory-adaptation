#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 27 11:29:47 2021

@author: Glenn Maes

This ROS node is used to simulate a startpose, which is used as constraint in the invariant trajectory calculation.
"""

import rospy
import numpy as np
import tf_conversions as tf
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    
    # Initialize ROS node 
    rospy.init_node("simulated_start_pose", anonymous=True)
    pub = rospy.Publisher("start_traj_pub", Pose, queue_size=1)
    
    # Set endpose
    startpose = tf.Frame(tf.Rotation.EulerZYX(0.0, 0.0, np.pi), tf.Vector(0.2, -0.3, 0.55)) #rot_x = np.pi/6
    
    print "Publishing simulated /start_traj_pub data"
    
    while not rospy.is_shutdown():
        # Put endpose data on topic
        startpose_msg = tf.toMsg(tf.fromMatrix(startpose))
        pub.publish(startpose_msg)
