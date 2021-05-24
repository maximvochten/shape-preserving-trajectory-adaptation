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
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    
    # Initialize ROS node 
    rospy.init_node("simulated_object_pose", anonymous=True)
    pub = rospy.Publisher("object_pose_pub", Pose, queue_size=1)
    vis_pub = rospy.Publisher("visual_object_pose", PoseStamped, queue_size=1)
    # Set objectpose
    objectpose = tf.Frame(tf.Rotation.EulerZYX(0.0, 0.0, 0.0), tf.Vector(0.5, 0.0, 0.25))
    
    print "Publishing simulated /object_pose_pub data"
    
    while not rospy.is_shutdown():
        # Setup for visualization of objectpose in rviz
        visual_objectpose = PoseStamped()
        visual_objectpose.header.stamp = rospy.Time.now()
        visual_objectpose.header.frame_id = "world"
        
        # Put objectpose data on topic
        objectpose_msg = tf.toMsg(tf.fromMatrix(objectpose))
        visual_objectpose.pose = objectpose_msg

        pub.publish(objectpose_msg)
        vis_pub.publish(visual_objectpose)
