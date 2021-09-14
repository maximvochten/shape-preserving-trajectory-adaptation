#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  6 21:35:17 2021

@author: Glenn Maes

Publish some simulated data on /tf topic to test tracker pose determination

This script is not needed in real application
"""

import rospy
import numpy as np
import tf
import tf_conversions as tf_c
import time

if __name__ == '__main__':
    
    rospy.init_node('simulated_tf_data', anonymous=True)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(60.0)
    
    T_c_t = tf_c.Frame(tf_c.Rotation.EulerZYX(0.0, 0.0, 0.0), tf_c.Vector(1.0, 1.5, 0.5))
    print("Tracker pose with respect to camera")
    print T_c_t
    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        
        br.sendTransform(tf_c.toTf(T_c_t)[0], tf_c.toTf(T_c_t)[1], now, "tracker", "camera")
        
        rate.sleep()