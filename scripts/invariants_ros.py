#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 11 11:27:56 2020

@author: Maxim Vochten

This ROS node is intended for bridging the invariant trajectory generation functionality into ROS.

The idea is to initialize this component using a recorded trajectory from
which the invariants are then calculated. Given the current and desired trajectory
states (+ other constraints), this node repeatedly calculates and outputs new trajectories.

input topics: current pose/twist of robot + current progress along trajectory
output topics: trajectory (including pose/twist)
"""


import rospy
import rospkg
import tf_conversions as tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
#from etasl_invariants_integration.msg import TwistArray
from etasl_invariants_integration.msg import Trajectory # this is a custom-made message type
from etasl_invariants_integration.msg import Bsplines # this is a custom-made message type
from etasl_invariants_integration.msg import VelocityProfile # this is a custom-made message type
import invariant_descriptor_class as invars
import numpy as np
from scipy.interpolate import splrep
import time
import math

class InvariantsROS:

    def __init__(self,demo_file_location,parameterization):
        '''Define topics and calculate invariants of demonstrated trajectory'''
        
        # Initialize ROS node, subscribers and publishers
        rospy.init_node('invariants_ros', anonymous=True)
        self.trajectory_pub = rospy.Publisher('trajectory_pub', Trajectory,queue_size=10)
        self.trajectory_pub2 = rospy.Publisher('trajectory_pub2', PoseArray,  queue_size=10)
        self.bspline_pub = rospy.Publisher('bspline_pub',Bsplines,queue_size=10)
        self.bspline_velocity_pub = rospy.Publisher('bspline_vel_pub', Float64, queue_size=10)
        self.eot_pub = rospy.Publisher('eot_pub', Float64, queue_size=10)
        self.velprof_pub = rospy.Publisher('velprof_pub', VelocityProfile, queue_size=10)
        rospy.Subscriber("current_pose_pub",Pose,self.callback_currentpose)
        rospy.Subscriber("current_twist_pub",Twist,self.callback_currenttwist)
        rospy.Subscriber('progress_partial',Float64,self.callback_localprogress)
        #TEST
        rospy.Subscriber("start_traj_pub",Pose,self.callback_startpose)
        rospy.Subscriber("target_pose_pub",Pose,self.callback_targetpose)
        
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
        
        # Bspline velocity
        self.velocity_bspline = 0.1
        self.s_final = 1.0
        #print(descriptor.velocityprofile_trans)
        #print(descriptor.velocityprofile_trans/descriptor.path_variable[-1])
        self.demonstrated_velocityprofile_trans = descriptor.velocityprofile_trans
        self.demonstrated_length = descriptor.path_variable[-1]
        self.setVelocityProfile()
        
    def simulate_new_target_poses(self,nb_samples):
        '''Simulate new desired target poses, in reality this would come from an external measurement system. At the moment the target pose is moving linearly in space'''
        targetpose_orig = self.current_pose_trajectory[-1]
  
#        targetpose_end = targetpose_orig.copy()
#        targetpose_end[0,3] -= 0.0
#        targetpose_end[1,3] += 0.0
#        targetpose_end[2,3] += 1.0
        
        # Transformation matrix in world frame
        DeltaT_translation = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0),tf.Vector(-0.6,0.4,-0.1))
        #DeltaT_translation = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0),tf.Vector(0.0,0.0,0.0))
        #DeltaT_rotation = tf.Frame(tf.Rotation.EulerZYX(-0.7,+0.7,-0.7),tf.Vector(0.0,0.0,0.0))
        DeltaT_rotation = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0),tf.Vector(0.0,0.0,0.0))
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
        '''This function is used to globally transform the trajectory so that eventually the new target poses are inside the workspace of the UR10 robot'''
        #TODO 
        
        #startpose_orig = self.current_pose_trajectory[0]
        #pose_robot_home = np.array([ [0.330273, 0.000795057,   -0.943885, -0.603343],[0.943885,-0.000797594,    0.330273, -0.1629],[-0.000490251,   -0.999999, -0.00101387, 0.733288],[0,0,0,1] ])
        #pose_robot_home = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0) , tf.Vector(0.2,-1.3,-1.0+0.07))

        # Define an offset translation and rotation as two homogeneous transformation matrices
        DeltaT_translation = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0),tf.Vector(0.1,-1.82,-1.2))
        DeltaT_rotation = tf.Frame(tf.Rotation.EulerZYX(2.0,-0.13,-0.14),tf.Vector(0.0,0.0,0.0))
        # DeltaT_rotation = tf.Frame(tf.Rotation.EulerZYX(0.0,0.0,0.0),tf.Vector(0.0,0.0,0.0))

        #deltapose_world = np.matmul(pose_robot_home,np.linalg.inv(startpose_orig))
        
        # Apply the offset translation and rotation to the whole trajectory
        for idx,pose in enumerate(self.current_pose_trajectory):
            self.current_pose_trajectory[idx] = tf.toMatrix( DeltaT_translation * tf.fromMatrix(pose) * DeltaT_rotation )

        
        #deltapose_orig = np.linalg.inv(startpose_orig) * targetpose_orig # is in object frame
        
                #        [[    0.330273, 0.000795057,   -0.943885;
        #     0.943885,-0.000797594,    0.330273;
        # -0.000490251,   -0.999999, -0.00101387]
        #[   -0.603343,    -0.16298,    0.733288]]
  
        
        
        

    def publish_trajectory(self):
        '''Put the calculated trajectory (pose/twist) on a topic'''
        
        trajectory_array = Trajectory() # initialize message
        poses = self.current_pose_trajectory # Load data from the class properties
        twists = self.current_twist_trajectory # Load data from the class properties
        
        for pose in poses:
            pose_msg = tf.toMsg(tf.fromMatrix(pose)) # convert to pose message
            trajectory_array.poses.append(pose_msg) # append message to array
        
        for twist in twists:
            t = Twist()
            t.angular.x = twist[0]
            t.angular.y = twist[1]
            t.angular.z = twist[2]
            t.linear.x  = twist[3]
            t.linear.y  = twist[4]
            t.linear.z  = twist[5]
            trajectory_array.twists.append(t) # append message to array
        
        self.trajectory_pub.publish(trajectory_array)
        
        # This one is purely for visualization, not so efficient...
        trajectory_array2 = PoseArray()
        trajectory_array2.header.stamp = rospy.Time.now()
        trajectory_array2.header.frame_id = '/world'
        trajectory_array2.poses = trajectory_array.poses
        self.trajectory_pub2.publish(trajectory_array2)
        
        rospy.loginfo('published' + ' a trajectory')

    def save_trajectory(self,index='_demo'):
        '''Save the calculated trajectory (pose/twist/invariants) to a file'''
        
        # Load data from the class properties
        poses = self.current_pose_trajectory
        twists = self.current_twist_trajectory
        invariants = self.current_invariants
        
        # Reshape matrix data to a 1x12 array, append to text file
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
        '''Generate the first trajectory (+ it initializes structure optimization problem for faster trajectories later)'''
        #new_trajectory,new_twists,invariants = self.shape_descriptor.generateFirstWindowTrajectory(startPose=self.current_pose_trajectory[0], endPose=self.current_pose_trajectory[-1])
        
        #TEST
        #roll_end, pitch_end, yaw_end = self.getRPY(self.targetPose[0:3,0:3])
        #new_trajectory,new_twists,invariants = self.shape_descriptor.generateFirstWindowTrajectory(startPose=self.startPose, endPose=self.current_pose_trajectory[-1])
        #self.rotation_symmetry_start()
        #new_trajectory,new_twists,invariants,solver_time = self.shape_descriptor.generateFirstWindowTrajectory(startPose=self.startPose, endPose=self.test)
        new_trajectory,new_twists,invariants,solver_time = self.shape_descriptor.generateFirstWindowTrajectory(startPose=self.startPose, endPose=self.targetPose)
        #print(len(new_trajectory))
        #new_trajectory,new_twists,invariants,solver_time = self.shape_descriptor.generateFirstWindowTrajectory(startPose=self.startPose, endPos=self.targetPose[0:3,3], endRPY = [roll_end, pitch_end, yaw_end])
        #print(invariants[0,0])
        
        # Save results to Class properties
        self.current_pose_trajectory = new_trajectory
        self.current_twist_trajectory = new_twists
        self.currentPose = new_trajectory[0]
        self.currentTwist = new_twists[0]
        self.current_invariants = invariants
        
        # Publish and save results in a text file
        #TEMP
        self.bspline_parameters()
        self.bspline_quaternion()
        self.publish_trajectory()
        self.save_trajectory(index=0)
        self.publish_bspline()
        self.publish_bspline_velocity()
        self.publish_velocityprofile()

    def standalone_test(self,targetposes):
        '''Use this for testing the invariant trajectory generator in complete isolation from eTaSL'''
        
        # Progress is a variable between 0 and 1, signifying the arc length along the trajectory
        current_progress = 0 # goes from 0 to 1
        progress_trigger_vals = [0.2,0.4,0.6,0.8] # select points along trajectory where a new trajectory needs to be generated
        
        # For each progress in progress_trigger_vals, calculate a new trajectory
        for idx,progress_val in enumerate(progress_trigger_vals):
            
            # Determine index corresponding to start window (important to set the model invariants correctly)
            new_progress = progress_val
            startwindow_index = int(new_progress*self.N)
            #targetPose = targetposes[startwindow_index][1]
            targetPose = self.targetPose
            rospy.loginfo('globalprogress: '+str(startwindow_index)) # global progress is progress along the complete trajectory
            
            # Load the pose and twist corresponding to the new progress value
            currentPose_index = int((new_progress-current_progress)*self.N) # subtract past trajectory
            currentPose = self.current_pose_trajectory[currentPose_index]
            currentTwist = self.current_twist_trajectory[currentPose_index]
            rospy.loginfo('localprogress: '+str(currentPose_index)) # local progress is progress along the "current" generated trajectory (which is logically shorter than the complete one)

            # Calculation of new trajectory
            new_trajectory, new_twists, invariants, solver_time = self.shape_descriptor.generateNextWindowTrajectory(startwindow_index, currentPose_index, startPose=currentPose, startTwist=currentTwist, endPose=self.targetPose)
            rospy.loginfo('I have calculated a new trajectory')
            
            # Store results
            self.current_pose_trajectory = new_trajectory
            self.current_twist_trajectory = new_twists
            #self.currentPose = new_trajectory[0]
            #self.currentTwist = new_twists[0]
            self.current_invariants = invariants
            
            # Publish and save the results in a text file
            self.publish_trajectory()
            self.save_trajectory(index=idx+1)

            current_progress = new_progress

    #def loop_trajectory_generation_etasl(self,targetposes):
    #TEST
    def loop_trajectory_generation_etasl(self):
        '''Generate trajectories towards new target poses and publish them'''
        
        counter = 1 # keep track of how many trajectories were calculated already
        globalprogress = 0
        
        # while not at the end of the global trajectory (or total amount of trajectories < 30)
        while not globalprogress >= 0.95  and not counter == 100: #counter == 30
        #while not globalprogress >= self.s_final and not counter == 5:
            # Set target pose
            starttime = time.time()

            localprogress = self.localprogress
            globalprogress += (1-globalprogress)*localprogress # add progress along remainder of path
            startwindow_index = int(globalprogress*self.N)
            
            currentPose_index = int(localprogress*float(len(self.current_pose_trajectory))) # subtract past trajectory
            #targetPose = targetposes[startwindow_index][1]
            #TEST
            targetPose = self.targetPose
            #targetPose = self.test
            #rospy.loginfo('startpose: ' + str(self.currentPose))
            #rospy.loginfo('starttwist: ' + str(self.currentTwist))
            rospy.loginfo('global progress: ' + str(globalprogress))
            rospy.loginfo('local progress: ' + str(localprogress))
            
            if globalprogress <= self.s_final:
                # Calculate new trajectory
                #self.rotation_symmetry()
                new_trajectory, new_twists, invariants, solver_time = self.shape_descriptor.generateNextWindowTrajectory(startwindow_index, currentPose_index, startPose=self.currentPose, startTwist=self.currentTwist, endPose=targetPose)
                #print(len(new_trajectory))
                rospy.loginfo(' ')
                #rospy.loginfo('new twists: ' + str(new_twists[0]))
                #l = self.velocity_bspline * solver_time
                L = invariants[3,0]
                #self.s_final = (L-l)/L
                #print(self.s_final)
                #print(s_final)
                # Store results
                self.current_pose_trajectory = new_trajectory
                self.current_twist_trajectory = new_twists
                self.current_invariants = invariants
                
                # TEST: define bspline parameters
                self.bspline_parameters()
                self.bspline_quaternion()

                self.publish_trajectory()
                self.save_trajectory(index=counter)
                self.publish_bspline()

                counter += 1
                time.sleep(1.0)
                endtime = time.time()
                l = self.velocity_bspline * (endtime-starttime)
                #self.s_final = (L-l)/L
                self.s_final = (1-l)
                #print(self.s_final)
                #print(1-l)
                #print(endtime-starttime)

        self.signal_eot()

    def callback_currentpose(self,pose):
        '''Save ROS topic message to class property'''
        self.currentPose = tf.toMatrix(tf.fromMsg(pose))

    def callback_currenttwist(self,t):
        '''Save ROS topic message to class property'''
        twist = np.array([t.angular.x,t.angular.y,t.angular.z,t.linear.x,t.linear.y,t.linear.z])
        self.currentTwist = twist

    def callback_localprogress(self,progress):
        '''Save ROS topic message to class property'''
        self.localprogress = progress.data
    
    #TEST
    def callback_startpose(self,startpose):
        '''Save ROS topic message to class property'''
        self.startPose = tf.toMatrix(tf.fromMsg(startpose))
        
    def callback_targetpose(self,targetpose):
        '''Save ROS topic message to class property'''
        self.targetPose = tf.toMatrix(tf.fromMsg(targetpose))

    def callback_targetpose_real_robot(self,targetpose):
        '''Save ROS topic message to class property -- In real application, the targetpose is determined by the pose of the HTC Vive tracker'''
        #self.targetPose = 

    def rotation_symmetry_start(self):
        tracker_pose = self.targetPose
        #print(tracker_pose)
        #print(tracker_pose[0,0])
        #print(tracker_pose[2,2])
        current_pose = self.startPose
        tracker_rot = tracker_pose[0:3,0:3]
        current_rot = current_pose[0:3,0:3]
        #print(tracker_rot)
        r_tr, p_tr, y_tr = self.getRPY(tracker_rot)
        r_cur, p_cur, y_cur = self.getRPY(current_rot)
        target_rot = self.RPYtoR(r_tr, p_tr, y_cur)
        target = np.eye(4)
        target[0:3,0:3] = target_rot
        target[0:3,3] = tracker_pose[0:3,3]
        self.test = target

    def rotation_symmetry(self):
        tracker_pose = self.targetPose
        current_pose = self.startPose
        tracker_rot = tracker_pose[0:3,0:3]
        current_rot = current_pose[0:3,0:3]
        r_tr, p_tr, y_tr = self.getRPY(tracker_rot)
        r_cur, p_cur, y_cur = self.getRPY(current_rot)
        target_rot = self.RPYtoR(r_tr, p_tr, y_cur)
        target = np.eye(4)
        target[0:3,0:3] = target_rot
        target[0:3,3] = tracker_pose[0:3,3]
        self.test = target


    def getRPY(self,R):
        r = math.atan2(R[2,1], R[2,2])
        y = math.atan2(R[1,0], R[0,0])
        p = math.atan2(-R[2,0], math.cos(y)*R[0,0] + math.sin(y)*R[1,0])
        return r,p,y

    def RPYtoR(self,r,p,y):
        R11 = math.cos(y)*math.cos(p)
        R12 = math.cos(y)*math.sin(p)*math.sin(r) - math.sin(y)*math.cos(r)
        R13 = math.cos(y)*math.sin(p)*math.cos(r) + math.sin(y)*math.sin(r)
        R21 = math.sin(y)*math.cos(p)
        R22 = math.sin(y)*math.sin(p)*math.sin(r) + math.cos(y)*math.cos(r)
        R23 = math.sin(y)*math.sin(p)*math.cos(r) - math.cos(y)*math.sin(r)
        R31 = -math.sin(p)
        R32 = math.cos(p)*math.sin(r)
        R33 = math.cos(p)*math.cos(r)
        R = [[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]]
        return R

    def bspline_parameters(self):
        '''Determine the parameters of B-spline representation of trajectory'''
        x = []
        y = []
        z = []
        poses = self.current_pose_trajectory
        for pose in poses:
            x.append(pose[0,3])
            y.append(pose[1,3])
            z.append(pose[2,3])
        #print(len(x))
        #print(len(y))
        #print(len(z))
        s = np.linspace(0,1,len(x))
        #print(s)
        #print(len(s))
        #knot_array = np.array([0,0,0,0,0.25,0.5,0.75,1,1,1,1])
        knot_array = np.array([0.25,0.5,0.75])
        #knot_array = np.array([0.5])
        self.tck_x = splrep(s,x, k=3, task=-1, t=knot_array)
        self.tck_y = splrep(s,y, k=3, task=-1, t=knot_array)
        self.tck_z = splrep(s,z, k=3, task=-1, t=knot_array)
        #print(len(self.tck_x[0]))
        #print(self.tck[1])
        
    def bspline_quaternion(self):
        '''Temporary separate function to obtain B-spline representation of orientation'''
        qx = []
        qy = []
        qz = []
        qw = []
        poses = self.current_pose_trajectory
        print(len(poses))
        for idx, pose in enumerate(poses):
            trace = pose[0,0] + pose[1,1] + pose[2,2]
            #print(trace)
            #print(idx)
            if trace > 0.0:
                s = np.sqrt(trace + 1.0)
                kw = s*0.5
                s = 0.5/s
                kx = s*(pose[2,1] - pose[1,2])
                ky = s*(pose[0,2] - pose[2,0])
                kz = s*(pose[1,0] - pose[0,1])
            else:
                if pose[0,0] >= pose[1,1] and pose[0,0] >= pose[2,2]:
                    # i = 0, j = 1, k = 2
                    s = np.sqrt(pose[0,0] - pose[1,1] - pose[2,2] + 1.0)
                    kx = s*0.5
                    s = 0.5/s
                    kw = s*(pose[2,1] - pose[1,2])
                    ky = s*(pose[1,0] + pose[0,1])
                    kz = s*(pose[2,0] + pose[0,2])
                elif pose[0,0] < pose[1,1] and pose[1,1] >= pose[2,2]:
                    # i = 1, j = 2, k = 0
                    s = np.sqrt(pose[1,1] - pose[2,2] - pose[0,0] + 1.0)
                    ky = s*0.5
                    s = 0.5/s
                    kw = s*(pose[0,2] - pose[2,0])
                    kz = s*(pose[2,1] + pose[1,2])
                    kx = s*(pose[0,1] + pose[1,0])
                else:
                    # i = 2, j = 0, k = 1
                    s = np.sqrt(pose[2,2] - pose[0,0] - pose[1,1] + 1.0)
                    kz = s*0.5
                    s = 0.5/s
                    kw = s*(pose[1,0] - pose[0,1])
                    kx = s*(pose[0,2] + pose[2,0])
                    ky = s*(pose[1,2] + pose[2,1])
            
            if (idx > 0) and (np.sqrt((kx - qx[idx-1])**2 + (ky - qy[idx-1])**2 + (kz - qz[idx-1])**2 + (kw - qw[idx-1])**2) > 0.5):
                qx.append(-kx)
                qy.append(-ky)
                qz.append(-kz)
                qw.append(-kw)
            else:
                qx.append(kx)
                qy.append(ky)
                qz.append(kz)
                qw.append(kw)

            #qs = np.sqrt(pose[0,0] + pose[1,1] + pose[2,2] + 1.0)/2
            #print(qs)
            #kx = pose[2,1] - pose[1,2]
            #ky = pose[0,2] - pose[2,0]
            #kz = pose[1,0] - pose[0,1]
            #
            #if pose[0,0] >= pose[1,1] and pose[0,0] >= pose[2,2]:
            #    kx1 = pose[0,0] - pose[1,1] - pose[2,2] + 1
            #    ky1 = pose[1,0] + pose[0,1]
            #    kz1 = pose[2,0] + pose[0,2]
            #    condition = (kx >= 0)
            #elif pose[1,1] >= pose[2,2]:
            #    kx1 = pose[1,0] + pose[0,1]
            #    ky1 = pose[1,1] - pose[0,0] - pose[2,2] + 1
            #    kz1 = pose[2,1] + pose[1,2]
            #    condition = (ky >= 0)
            #else:
            #    kx1 = pose[2,0] + pose[0,2]
            #    ky1 = pose[2,1] + pose[1,2]
            #    kz1 = pose[2,2] - pose[0,0] - pose[1,1] + 1
            #    condition = (kz >= 0)
            #
            #if condition:
            #    kx = kx + kx1
            #    ky = ky + ky1
            #    kz = kz + kz1
            #else:
            #    kx = kx - kx1
            #    ky = ky - ky1
            #    kz = kz - kz1
            #    
            #nm = np.linalg.norm([kx, ky, kz])
            ##print(nm)
            #if nm == 0:
            #    qx.append(0)
            #    qy.append(0)
            #    qz.append(0)
            #    qw.append(1)
            #else:
            #    s = np.sqrt(1 - qs**2)/nm
            #    qvx = s*kx
            #    qvy = s*ky
            #    qvz = s*kz
            #    if idx > 0 and np.sqrt((qvx - qx[idx-1])**2 + (qvy - qy[idx-1])**2 + (qvz - qz[idx-1])**2 + (qs - qw[idx-1])**2) > 0.5:
            #        qx.append(-qvx)
            #        qy.append(-qvy)
            #        qz.append(-qvz)
            #        qw.append(-qs)
            #    else:
            #        qx.append(qvx)
            #        qy.append(qvy)
            #        qz.append(qvz)
            #        qw.append(qs)
        #print(len(qx)) 
        s = np.linspace(0, 1, len(qx))
        knot_array = np.array([0.25,0.5,0.75])
        self.tck_qx = splrep(s, qx, k=3, task=-1, t=knot_array) 
        self.tck_qy = splrep(s, qy, k=3, task=-1, t=knot_array)
        self.tck_qz = splrep(s, qz, k=3, task=-1, t=knot_array)
        self.tck_qw = splrep(s, qw, k=3, task=-1, t=knot_array)
                    
        
    
    def publish_bspline(self):
        bspline_array = Bsplines() # initialize message
        poses = self.current_pose_trajectory # Load data from class property
        knots = self.tck_x[0] # Load data from class property
        coefs_x = self.tck_x[1] # Load data from class property
        coefs_y = self.tck_y[1]
        coefs_z = self.tck_z[1]
        coefs_qx = self.tck_qx[1]
        coefs_qy = self.tck_qy[1]
        coefs_qz = self.tck_qz[1]
        coefs_qw = self.tck_qw[1]
        
        coefs_x = coefs_x[0:-4]
        coefs_y = coefs_y[0:-4]
        coefs_z = coefs_z[0:-4]
        coefs_qx = coefs_qx[0:-4]
        coefs_qy = coefs_qy[0:-4]
        coefs_qz = coefs_qz[0:-4]
        coefs_qw = coefs_qw[0:-4]
       
        #print(coefs_x)
        #print(coefs_y)
        #print(coefs_z)
        #print(coefs_qx)
        #print(coefs_qy)
        #print(coefs_qz)
        #print(coefs_qw)
        for pose in poses:
            pose_msg = tf.toMsg(tf.fromMatrix(pose))
            bspline_array.poses.append(pose_msg)
        for knot in knots:
            bspline_array.knots.append(knot)
        #for coef_x in coefs_x:
        #    bspline_array.control_points.x.append(coef_x)
        #for coef_y in coefs_y:
        #    bspline_array.control_points.y.append(coef_y)
        #for coef_z in coefs_z:
        #    bspline_array.control_points.z.append(coef_z)
        
        for i in range(0,len(coefs_x)):
            point = Point()
            point.x = coefs_x[i]
            point.y = coefs_y[i]
            point.z = coefs_z[i]
            bspline_array.control_points.append(point) #([coefs_x[i], coefs_y[i], coefs_z[i]])
            
            quat = Quaternion()
            quat.x = coefs_qx[i]
            quat.y = coefs_qy[i]
            quat.z = coefs_qz[i]
            quat.w = coefs_qw[i]
            bspline_array.control_quaternion.append(quat)
        
        self.bspline_pub.publish(bspline_array)
        rospy.loginfo('published bspline data')
        
    def publish_bspline_velocity(self):
        self.bspline_velocity_pub.publish(self.velocity_bspline)
        #counter = 0
        #while not counter == 20:
        #    self.bspline_velocity_pub.publish(self.velocity_bspline)
        #    counter += 1
    def signal_eot(self):
        val = 1.0
        self.eot_pub.publish(val)
        
    def setVelocityProfile(self):
        geometric_velocityprofile = self.demonstrated_velocityprofile_trans/self.demonstrated_length
        r = np.linspace(0, 1, len(geometric_velocityprofile))
        knot_array = np.array([0.25,0.5,0.75])
        self.tck_sdot = splrep(r, geometric_velocityprofile, k=3, task=-1, t=knot_array)
        
    def publish_velocityprofile(self):
        velprof = VelocityProfile()
        coefs_velprof = self.tck_sdot[1]
        coefs_velprof = coefs_velprof[0:-4]
        for control_point in coefs_velprof:
            velprof.control_velocityprofile.append(control_point)
        
        self.velprof_pub.publish(velprof)

if __name__ == '__main__':
    try:
        # Set location of file containing demonstrated trajectory
        #demo_traj_file = "sinus.txt" #recorded_motion.csv
        demo_traj_file = "test_pouring.csv"
        #demo_traj_file = "pouring_motion.csv"
        #demo_traj_file = "curved_motion.csv"
        rospack = rospkg.RosPack()
        file_location = rospack.get_path('etasl_invariants_integration') + '/data/demonstrated_trajectories/' + demo_traj_file

        # Load trajectory and calculate invariants
        inv = InvariantsROS(demo_file_location=file_location,parameterization='geometric')
        
        # Transform demonstrated trajectory to workspace UR10 robot
        inv.transform_pose_trajectory()

        # Set to True if you want to test this component on its own
        test_standalone = False
        
        # Simulate new target poses (in a real application this would come in from an external source)
        targetposes = inv.simulate_new_target_poses(inv.N)

        # Generate first new trajectory (#TODO is it really necessary to do this separately?)
        inv.first_window()

        # Loop in which new trajectories are continuously generated
        if test_standalone:
            inv.standalone_test(targetposes)
        else:
            #inv.loop_trajectory_generation_etasl(targetposes)
            #TEST
            inv.loop_trajectory_generation_etasl()

    except rospy.ROSInterruptException:
        pass
