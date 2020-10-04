#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 11 11:27:56 2020

@author: Maxim Vochten

This ROS node is intended for bridging the invariant trajectory generation functionality into ROS.

The idea is to initialize this component using a recorded trajectory from
which the invariants are then calculated. Given the current and desired trajectory
states (+ other constraints), this node repeatedly calculates and outputs new trajectories.

input topics: current pose/twist of robot + desired target (and other constraints)
output topics: pose and twist trajectory
"""

#!/usr/bin/env python
# license removed for brevity
import rospy
import rospkg
import tf_conversions as tf
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
#from etasl_invariants_integration.msg import TwistArray
from etasl_invariants_integration.msg import Trajectory
import invariant_descriptor_class as invars
import numpy as np

class InvariantsROS:

    def __init__(self,demo_file_location,parameterization):
        '''Define topics and calculate invariants of demonstrated trajectory'''
        
        # Initialize ROS subscribers and publishers
        rospy.init_node('invariants_ros', anonymous=True)
        self.trajectory_pub = rospy.Publisher('trajectory_pub', Trajectory,queue_size=10)
        self.trajectory_pub2 = rospy.Publisher('trajectory_pub2', PoseArray,  queue_size=10)
        rospy.Subscriber("current_pose_pub",Pose,self.callback_currentpose)
        rospy.Subscriber("current_twist_pub",Twist,self.callback_currenttwist)
        rospy.Subscriber('progress_partial',Float32,self.callback_localprogress)
        rospack = rospkg.RosPack()

        # Calculate invariants + return corresponding trajectory, sample period is found from timestamps
        descriptor = invars.MotionTrajectory(demo_file_location, invariantType=parameterization, suppressPlotting=True)
        poses = descriptor.getPosesFromInvariantSignature(descriptor.getInvariantSignature())
        self.results_folder = rospack.get_path('etasl_invariants_integration') + '/data/results/' 
        self.current_pose_trajectory = poses
        self.current_twist_trajectory = [np.zeros([1,6])]
        self.current_invariants = descriptor.getInvariantsDemo()['U']
        self.shape_descriptor = descriptor
        self.save_trajectory(index='_demo')

        # Initialize progress along trajectory (between 0 and 1)
        self.N = len(poses)
        self.localprogress = 0
        self.globalprogress = 0

    def simulate_new_target_poses(self,nb_samples):
        '''Simulate new desired target poses, in reality this would come from an external measurement system. At the moment the target pose is moving linearly in space'''
        targetpose_orig = self.current_pose_trajectory[-1]
  
#        targetpose_end = targetpose_orig.copy()
#        targetpose_end[0,3] -= 0.0
#        targetpose_end[1,3] += 0.0
#        targetpose_end[2,3] += 1.0
        
        # Transformation matrix in world frame
        DeltaT_translation = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0),tf.Vector(-0.7,0.35,-0.1))
        DeltaT_rotation = tf.Frame(tf.Rotation.EulerZYX(-0.7,+0.7,-0.7),tf.Vector(0.0,0.0,0.0))
        targetpose_end1 = tf.toMatrix( DeltaT_translation * tf.fromMatrix(targetpose_orig) * DeltaT_rotation )
        
        target_poses_list1 = self.shape_descriptor.generateLinearMotionTrajectory(startPose=targetpose_orig,endPose=targetpose_end1,duration=1.0,cyclePeriod=1./nb_samples)
        
#        DeltaT_translation = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0),tf.Vector(0.5,-0.2,0.2))
#        DeltaT_rotation = tf.Frame(tf.Rotation.EulerZYX(0.7/2,-0.7/2,-1.7/2),tf.Vector(0.0,0.0,0.0))
#        targetpose_end2 = tf.toMatrix( DeltaT_translation * tf.fromMatrix(targetpose_end1) * DeltaT_rotation )
#        
#        target_poses_list2 = self.shape_descriptor.generateLinearMotionTrajectory(startPose=targetpose_end1,endPose=targetpose_end2,duration=0.5,cyclePeriod=1./nb_samples)

        #target_poses_list = target_poses_list1 + target_poses_list2

        target_poses_list = target_poses_list1 #+ target_poses_list2
 
        #target_poses_list = self.shape_descriptor.generateLinearMotionTrajectory(startPose=targetpose_end2,endPose=targetpose_orig,duration=1.0,cyclePeriod=1./nb_samples)
            
        
#        # Publish
#        targetRange_msg = PoseArray()
#        targetRange_msg.header.stamp = rospy.Time.now()
#        targetRange_msg.header.frame_id = '/world'
#        
#        # Cast target range in ROS message format. Get rid of timestamps by [1], get rid of [0,0,0,1] row by [:-1]
#        for pose in target_poses_list:
#            targetRange_msg.poses.append(tf.toMsg(tf.fromMatrix(pose[1][:-1])))
#        self.targetRange_pub.publish(targetRange_msg)
        
        return target_poses_list

    def transform_pose_trajectory(self):
      
        #startpose_orig = self.current_pose_trajectory[0]
        #pose_robot_home = np.array([ [0.330273, 0.000795057,   -0.943885, -0.603343],[0.943885,-0.000797594,    0.330273, -0.1629],[-0.000490251,   -0.999999, -0.00101387, 0.733288],[0,0,0,1] ])
        
        #pose_robot_home = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0) , tf.Vector(0.2,-1.3,-1.0+0.07))

        DeltaT_translation = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0),tf.Vector(0.19,-1.5-0.5,-0.93))
        DeltaT_rotation = tf.Frame(tf.Rotation.EulerZYX(2.11,-0.13,-0.14),tf.Vector(0.0,0.0,0.0))
        
        #deltapose_world = np.matmul(pose_robot_home,np.linalg.inv(startpose_orig))
        
        for idx,pose in enumerate(self.current_pose_trajectory):
            self.current_pose_trajectory[idx] = tf.toMatrix( DeltaT_translation * tf.fromMatrix(pose) * DeltaT_rotation )

        
        #deltapose_orig = np.linalg.inv(startpose_orig) * targetpose_orig # is in object frame
        
                #        [[    0.330273, 0.000795057,   -0.943885;
        #     0.943885,-0.000797594,    0.330273;
        # -0.000490251,   -0.999999, -0.00101387]
        #[   -0.603343,    -0.16298,    0.733288]]
  
        
        
        

    def publish_trajectory(self):
        '''Put the calculated trajectory (pose/twist) on a topic''' 
        trajectory_array = Trajectory() # to be put on topic
        poses = self.current_pose_trajectory
        twists = self.current_twist_trajectory
        for pose in poses:
            pose_msg = tf.toMsg(tf.fromMatrix(pose))
            trajectory_array.poses.append(pose_msg)
        for twist in twists:
            t = Twist()
            t.angular.x = twist[0]
            t.angular.y = twist[1]
            t.angular.z = twist[2]
            t.linear.x  = twist[3]
            t.linear.y  = twist[4]
            t.linear.z  = twist[5]
            trajectory_array.twists.append(t)
        self.trajectory_pub.publish(trajectory_array)
        
        trajectory_array2 = PoseArray()
        trajectory_array2.header.stamp = rospy.Time.now()
        trajectory_array2.header.frame_id = '/world'
        trajectory_array2.poses = trajectory_array.poses
        self.trajectory_pub2.publish(trajectory_array2)
        
        
        rospy.loginfo('published' + ' traj')

    def save_trajectory(self,index='_demo'):
        '''Save the calculated trajectory (pose/twist/invariants) to a file'''
        poses = self.current_pose_trajectory
        twists = self.current_twist_trajectory
        invariants = self.current_invariants
        
        pose_list = []
        for pose in poses:
            pose_list.append(np.append(rospy.get_time(),pose[0:3,:].flatten('F')))
        np.savetxt(self.results_folder + 'traj'+str(index)+'.txt',pose_list)
        rospy.loginfo('saved' + ' traj'+str(index)+'.txt')
        
        twist_list = []
        for twist in twists:
            twist_list.append(np.append(rospy.get_time(),twist))
        np.savetxt(self.results_folder + 'twist'+str(index)+'.txt',twist_list)
        rospy.loginfo('saved' + ' twist'+str(index)+'.txt')

        np.savetxt(self.results_folder + 'invariants'+str(index)+'.txt',invariants.transpose())
        rospy.loginfo('saved' + ' invariants'+str(index)+'.txt')

    def first_window(self):
        '''Generate the first trajectory (+ initializes structure optimization problem for faster trajectories later)'''
        new_trajectory,new_twists,invariants = self.shape_descriptor.generateFirstWindowTrajectory(startPose=self.current_pose_trajectory[0], endPose=self.current_pose_trajectory[-1])
        self.current_pose_trajectory = new_trajectory
        self.current_twist_trajectory = new_twists
        self.currentPose = new_trajectory[0]
        self.currentTwist = new_twists[0]
        self.current_invariants = invariants
        self.publish_trajectory()
        self.save_trajectory(index=0)

    def standalone_test(self,targetposes):
        '''Use this for testing this component in isolation from etasl'''
        progress_sequence = [0.2,0.4,0.6,0.8] #start from this point in the trajectory
        #targetposes = self.simulate_new_target_poses(self.N)
        old_progress = 0 # goes from 0 to 1

        # For each progress trigger, calculate a new trajectory
        for idx,progress_val in enumerate(progress_sequence):
            
            # Set window index (important to set model invariants correctly)
            new_progress = progress_val
            startwindow_index = int(new_progress*self.N)
            targetPose = targetposes[startwindow_index][1]
            rospy.loginfo('globalprogress: '+str(startwindow_index))
            
            # Set current pose and twist
            currentPose_index = int((new_progress-old_progress)*self.N) # subtract past trajectory
            currentPose = self.current_pose_trajectory[currentPose_index]
            currentTwist = self.current_twist_trajectory[currentPose_index]
            rospy.loginfo('localprogress: '+str(currentPose_index))

            # Calculation of new trajectory
            new_trajectory, new_twists, invariants = self.shape_descriptor.generateNextWindowTrajectory(startwindow_index, currentPose_index, startPose=currentPose, startTwist=currentTwist, endPose=targetPose)
            rospy.loginfo('calculated traj')
            
            # Store results
            self.current_pose_trajectory = new_trajectory
            self.current_twist_trajectory = new_twists
            #self.currentPose = new_trajectory[0]
            #self.currentTwist = new_twists[0]
            self.current_invariants = invariants
            
            # Communicate results
            self.publish_trajectory()
            self.save_trajectory(index=idx+1)

            old_progress = new_progress

    def loop_trajectory_generation_etasl(self,targetposes):
        '''Generate trajectories towards new endpoints and publish them'''
        #rospy.sleep(0.1)
        
        counter = 1 # keep track of how many trajectories were calculated already
        globalprogress = 0
        
        # while not at the end of the global trajectory (or total amount of trajectories < 30)
        while not globalprogress >= 0.85  and not counter == 30:

            # Set target pose
            localprogress = self.localprogress
            globalprogress += (1-globalprogress)*localprogress # add progress along remainder of path
            startwindow_index = int(globalprogress*self.N)
            
            currentPose_index = int(localprogress*float(len(self.current_pose_trajectory))) # subtract past trajectory
            targetPose = targetposes[startwindow_index][1]
            #rospy.loginfo('startpose: ' + str(self.currentPose))
            #rospy.loginfo('starttwist: ' + str(self.currentTwist))
            rospy.loginfo('global progress: ' + str(globalprogress))
            rospy.loginfo('local progress: ' + str(localprogress))

            # Calculate new trajectory
            new_trajectory, new_twists, invariants = self.shape_descriptor.generateNextWindowTrajectory(startwindow_index, currentPose_index, startPose=self.currentPose, startTwist=self.currentTwist, endPose=targetPose)
            rospy.loginfo(' ')
            #rospy.loginfo('new twists: ' + str(new_twists[0]))
            
            # Store results
            self.current_pose_trajectory = new_trajectory
            self.current_twist_trajectory = new_twists
            self.current_invariants = invariants
            

            self.publish_trajectory()
            self.save_trajectory(index=counter)

            counter += 1

    def callback_currentpose(self,pose):
        '''ROS topic message to class property'''
        self.currentPose = tf.toMatrix(tf.fromMsg(pose))

    def callback_currenttwist(self,t):
        '''ROS topic message to class property'''
        twist = np.array([t.angular.x,t.angular.y,t.angular.z,t.linear.x,t.linear.y,t.linear.z])
        self.currentTwist = twist

    def callback_localprogress(self,progress):
        '''ROS topic message to class property'''
        self.localprogress = progress.data


if __name__ == '__main__':
    try:
        # Location file of demonstrated trajectory
        demo_traj_file = "sinus.txt" #recorded_motion.csv
        rospack = rospkg.RosPack()
        file_location = rospack.get_path('etasl_invariants_integration') + '/data/demonstrated_trajectories/' + demo_traj_file

        # Initialization
        inv = InvariantsROS(demo_file_location=file_location,parameterization='geometric')
        inv.transform_pose_trajectory()
        
        # Test this component on its own
        test_standalone = False
        
        # This function takes a long time (~0.6 seconds, so keep out of loop)
        targetposes = inv.simulate_new_target_poses(inv.N)
        
        # Do first window (#TODO is this really necessary?)
        inv.first_window()

        # Handle the other windows
        if test_standalone:
            inv.standalone_test(targetposes)
        else:
            inv.loop_trajectory_generation_etasl(targetposes)

    except rospy.ROSInterruptException:
        pass
