#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Simulate trajectories +  accompanying velocity profile

@author: mvochten
"""

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from etasl_invariants_integration.msg import TwistArray
import numpy as np

class TrajectoryGenerator:
    
    def __init__(self):
        
        self.pose_traj = np.loadtxt('../data/traj1.txt')
        self.twist_traj = np.loadtxt('../data/twist1.txt')
    
        self.pose_traj_pub = rospy.Publisher('pose_traj_pub', PoseArray, queue_size=20)
        self.twist_traj_pub = rospy.Publisher('twist_traj_pub', TwistArray, queue_size=20)
        
        #self.last_time = rospy.Time.now().to_sec()
           
    def publisher(self):
        
        rospy.init_node('publisher', anonymous=True)
        
        rate = rospy.Rate(10) # 10hz 
        
        while not rospy.is_shutdown():
                        
            p_list = PoseArray()
            t_list = TwistArray()
                       
            for line in self.pose_traj:
                p = Pose()
                p.position.x = line[0]
                p.position.y = line[1]
		p.position.z = line[1]      
                p_list.poses.append(p)
                
            for line in self.twist_traj:
                t = Twist()
                t.linear.x = line[0]
                t.linear.y = line[1]
		t.linear.z = line[1]    
                t_list.twists.append(t)
            
            self.pose_traj_pub.publish(p_list)
            self.twist_traj_pub.publish(t_list)
            
            #rospy.loginfo(p_list)
            #rospy.loginfo(t_list)
            #rospy.loginfo(len(t_list.twists))
            
            rate.sleep()
       
if __name__ == '__main__':
    try:

        trajgen = TrajectoryGenerator()
        
        trajgen.publisher()
        
        
    except rospy.ROSInterruptException:
        pass
    
