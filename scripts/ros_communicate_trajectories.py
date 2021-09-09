#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 31 11:09:51 2021

@author: Glenn Maes

ROS part of the decoupling
"""

import rospy
import rospkg
import tf_conversions as tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from etasl_invariants_integration.msg import Trajectory
import calculate_invariant_trajectory as calc_traj
import numpy as np

class InvariantsROS:
    
    def __init__(self, demo_file_location, parameterization, invariants_file_location = None):
        '''Define topics and calculate trajectories'''
        
        # Initialize ROS node, subscribers, and publishers
        rospy.init_node('invariants_ros', anonymous=True)
        self.trajectory_pub = rospy.Publisher('trajectory_pub', Trajectory, queue_size=10)
        rospy.Subscriber("current_pose_pub", Pose, self.callback_currentpose)
        rospy.Subscriber("current_twist_pub", Twist, self.callback_currenttwist)
        rospy.Subscriber("progress_partial", Float64, self.callback_localprogress)
        rospy.Subscriber("start_traj_pub", Pose, self.callback_startpose)
        rospy.Subscriber("target_pose_pub", Pose, self.callback_targetpose)
        
        # Define results folder
        rospack = rospkg.RosPack()
        self.results_folder = rospack.get_path('etasl_invariants_integration') + '/data/results/'
        
        # Trajectory generator
        self.traj_data = calc_traj.CalculateTrajectory(demo_file_location, parameterization, invariants_file_location)
#        self.save_trajectory(index='demo')

    def publish_trajectory(self):
        '''Put calculated trajectory on a topic'''
        
        trajectory_array = Trajectory() # Initialize message
        # Load data from trajectory generator data
        poses = self.traj_data.current_pose_trajectory 
        twists = self.traj_data.current_twist_trajectory
        
        for pose in poses:
            pose_msg = tf.toMsg(tf.fromMatrix(pose)) # Convert to pose message
            trajectory_array.poses.append(pose_msg)
        for twist in twists:
            t = Twist()
            t.angular.x = twist[0]
            t.angular.y = twist[1]
            t.angular.z = twist[2]
            t.linear.x = twist[3]
            t.linear.y = twist[4]
            t.linear.z = twist[5]
            trajectory_array.twists.append(t)
        self.trajectory_pub.publish(trajectory_array)
        rospy.loginfo('published' + ' a trajectory')
        
    def callback_currentpose(self,pose):
        '''Save ROS topic message to Class property'''
        self.currentPose = tf.toMatrix(tf.fromMsg(pose))
    
    def callback_currenttwist(self,t):
        '''Save ROS topic message to Class property'''
        twist = np.array([t.angular.x, t.angular.y, t.angular.z, t.linear.x, t.linear.y, t.linear.z])
        self.currentTwist = twist
    
    def callback_localprogress(self,progress):
        '''Save ROS topic message to Class property'''
        self.localprogress = progress.data
        
    def callback_startpose(self,startpose):
        '''Save ROS topic message to Class property'''
        self.startPose = tf.toMatrix(tf.fromMsg(startpose))
        
    def callback_targetpose(self,targetpose):
        '''Save ROS topic message to Class property'''
        self.targetPose = tf.toMatrix(tf.fromMsg(targetpose))
        
        
#    def save_trajectory(self, index='_demo'):
#        '''Save the calculated trajectory to a file'''
#        
#        # Load data from trajectory generator data
#        poses = self.traj_data.current_pose_trajectory
#        twists = self.traj_data.current_twist_trajectory
#        invariants = self.traj_data.current_invariants
#        
#        # Reshape matrix data to a 1x12 array, and append to text file
#        pose_list = []
#        for pose in poses:
#            pose_list.append(np.append(rospy.get_time(),pose[0:3,:].flatten('F')))
#        np.savetxt(self.results_folder + 'traj')
        
if __name__ == '__main__':
    try:
        # Set location of file containing the demonstrated trajectory
        demo_traj_file = "pouring_motion.csv"
        rospack = rospkg.RosPack()
        file_location = rospack.get_path('etasl_invariants_integration') + '/data/demonstrated_trajectories/' + demo_traj_file
        
        # Load trajectory & calculate invariants
        bool_use_earlier_invariants = 1
        if bool_use_earlier_invariants:
            invariants_file_location = 'dummy_not_actually_used'
            inv = InvariantsROS(demo_file_location=file_location, parameterization='geometric', invariants_file_location=invariants_file_location)
        else:
            inv = InvariantsROS(demo_file_location=file_location, parameterization='geometric')
       
        # Calculate first window
        inv.traj_data.first_window(inv.startPose, inv.targetPose)
        inv.publish_trajectory()
        
        # Loop calculation
        for i in range(0,10):
            inv.traj_data.trajectory_generation(inv.currentPose, inv.targetPose, inv.localprogress)
            inv.publish_trajectory()
    except rospy.ROSInterruptException:
        pass
