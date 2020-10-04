#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu May  2 11:07:26 2019
@author: Zeno Gillis, Toon Daemen

This ROS-node creates and maintains a marker message for visualisation of the trajecories in RVIZ
(see http://wiki.ros.org/rviz/DisplayTypes/Marker)
The marker includes:
    - a red cube indicating the current pose of the qbSoftHand
    - a yellow cube indicating the start pose of the most recently generated trajectory
    - a green cube  indicating the target pose of the most recently generated trajectory
    - a white line indicating the range within which the target pose can (linearly) vary
    - a purple set of points indicating the most recently generated trajectory
    - a black set of points indicating the previously generated trajectories
    - a blue line that is being 'drawn' by the qbSoftHand during its motion

makePoseCube(), makeArrows() and showTrajectory() are obsolete but temporarily kept in the code because maybe they
still can be of use in the near future.
"""


import rospy
#import tf

from std_msgs.msg import ColorRGBA
#from iiwa_msgs.msg import JointPositionVelocity, CartesianPose
from geometry_msgs.msg import Pose, Point, PoseStamped
from etasl_invariants_integration.msg import Trajectory
from visualization_msgs.msg import Marker

#from scipy.spatial.transform import Rotation as R


#def makePoseCube(nb, pose, frame = '/qbhand1_palm_link', color = ColorRGBA(0,1,1,1)):
#    cube = Marker()
#    cube.color = color
#    cube.ns = 'cube' + str(nb)
#    cube.action  = Marker.ADD
#    cube.header.frame_id = frame
#    cube.type = Marker.CUBE
#    cube.id = 30 + nb
#    
#    cube.scale.x = 0.01
#    cube.scale.y = 0.01
#    cube.scale.z = 0.01
#    
#    cube.pose.position.x = pose[0,3] 
#    cube.pose.position.y = pose[1,3] 
#    cube.pose.position.z = pose[2,3] 
#    
#    rotM = R.from_dcm(pose[0:3,0:3])
#    quat = rotM.as_quat()
#    
#    cube.pose.orientation.x = quat[0]
#    cube.pose.orientation.y = quat[1]
#    cube.pose.orientation.z = quat[2]
#    cube.pose.orientation.w = quat[3]
#    
#    return cube
#
#def makeArrows(nb, pose):
#    #TODO not working properly, arrows orientations are crooked
#    xArrow = Marker()
#    xArrow.color = ColorRGBA(1,0,0,1)
#    xArrow.ns = 'arrow' + str(nb)
#    xArrow.action  = Marker.ADD
#    xArrow.header.frame_id = '/world'
#    xArrow.type = Marker.ARROW
#    xArrow.id = 100 + nb
#    
#    xArrow.scale.x = 0.05
#    xArrow.scale.y = 0.005
#    xArrow.scale.z = 0.01
#    
#    xArrow.pose.position.x = pose[0,3] 
#    xArrow.pose.position.y = pose[1,3] 
#    xArrow.pose.position.z = pose[2,3] 
#    
#    rotM = R.from_rotvec(pose[0,0:3])
#    quat = rotM.as_quat()
#    
#    xArrow.pose.orientation.x = quat[0]
#    xArrow.pose.orientation.y = quat[1]
#    xArrow.pose.orientation.z = quat[2]
#    xArrow.pose.orientation.w = quat[3]
##    
#    yArrow = Marker()
#    yArrow.color = ColorRGBA(0,1,0,1)
#    yArrow.ns = 'arrow' + str(nb)
#    yArrow.action  = Marker.ADD
#    yArrow.header.frame_id = '/world'
#    yArrow.type = Marker.ARROW
#    yArrow.id = 600 + nb
#    
#    yArrow.scale.x = 0.05
#    yArrow.scale.y = 0.005
#    yArrow.scale.z = 0.01
#    
#    yArrow.pose.position = xArrow.pose.position
#    
#    rotM = R.from_rotvec(pose[1,0:3])
#    quat = rotM.as_quat()
#    
#    yArrow.pose.orientation.x = quat[0]
#    yArrow.pose.orientation.y = quat[1]
#    yArrow.pose.orientation.z = quat[2]
#    yArrow.pose.orientation.w = quat[3]
#    
#    zArrow = Marker()
#    zArrow.color = ColorRGBA(0,0,1,1)
#    zArrow.ns = 'arrow' + str(nb)
#    zArrow.action  = Marker.ADD
#    zArrow.header.frame_id = '/world'
#    zArrow.type = Marker.ARROW
#    zArrow.id = 1100 + nb
#    
#    zArrow.scale.x = 0.05
#    zArrow.scale.y = 0.005
#    zArrow.scale.z = 0.005
#    
#    zArrow.pose.position = xArrow.pose.position
#    
#    rotM = R.from_rotvec(pose[2,0:3])
#    quat = rotM.as_quat()
#    
#    zArrow.pose.orientation.x = quat[0]
#    zArrow.pose.orientation.y = quat[1]
#    zArrow.pose.orientation.z = quat[2]
#    zArrow.pose.orientation.w = quat[3]
#    
#    return xArrow, yArrow, zArrow
#    
#
#def showTrajectory(poses, publisher, rgbaColor, frame = '/qbhand1_palm_link', name = 'trajectory'):
#    try:
#        trajectoryLineStrip = Marker()
#        trajectoryLineStrip.color = rgbaColor
#        trajectoryLineStrip.ns = name
#        trajectoryLineStrip.action  = Marker.ADD
#        trajectoryLineStrip.header.frame_id = frame
#        trajectoryLineStrip.type = Marker.LINE_STRIP
#        trajectoryLineStrip.id = 20
#        
#        trajectoryLineStrip.scale.x = 0.005
#            
#        cube_list = []
##        arrows_list = []
#        index = 0
#        for pose in poses:
#            newPoint = Point()
#            newPoint.x = pose[0,3] 
#            newPoint.y = pose[1,3]
#            newPoint.z = pose[2,3]
#            trajectoryLineStrip.points.append(newPoint)
#            
#            if (index == len(poses)/2) or (index == 3*len(poses)/4) or (index == len(poses)/4):
#                cube_list.append(makePoseCube(index, pose, frame = frame))
##                arrows_list.append(makeArrows(index, pose))
#            if index == 0:
#                cube_list.append(makePoseCube(index, pose, frame = frame, color = ColorRGBA(0.0,1.0,0.0,1.0)))
#            if index == len(poses)-1 :
#                cube_list.append(makePoseCube(index, pose, frame = frame, color = ColorRGBA(1.0,0.0,0.0,1.0)))
#                
#            index += 1
#        
#        trajectoryLineStrip.header.stamp = rospy.Time.now()
#        for i in range(30):
#            publisher.publish(trajectoryLineStrip)
#            for cube in cube_list:
#                publisher.publish(cube)
##            for arrows in arrows_list:
##                for arrow in arrows:
##                    publisher.publish(arrow)
#            rospy.Rate(100).sleep()
#            
#    except rospy.ROSException, e:
#        print "ROSException: %s"%e
#        

def plot_obstacle(visualPublisher):
    obstacleMarker = Marker()
    obstacleMarker.color = ColorRGBA(0,1,0,1)
    obstacleMarker.ns = 'obstacle'
    obstacleMarker.action  = Marker.ADD
    obstacleMarker.header.frame_id = '/world'
    obstacleMarker.header.stamp = rospy.Time.now()
    obstacleMarker.type = 3
    obstacleMarker.id = 1
    
    obstacleMarker.pose.position.x = 0.036;
    obstacleMarker.pose.position.y = -0.90;
    obstacleMarker.pose.position.z = 0.31;
    
    obstacleMarker.scale.x = 0.1
    obstacleMarker.scale.y = 0.1
    
    obstacleMarker.scale.z = 0.5 #height
    
    visualPublisher.publish(obstacleMarker)

def deleteAllMarkers(visualPublisher):
    """
    Clear everything this code has drawn on the RVIZ view
    """
    marker = Marker()
    marker.color.a = 0
    marker.action = Marker.DELETEALL 
    visualPublisher.publish(marker)  

def sendMarkers(visualPublisher):
    
        rate = rospy.Rate(100)
        
        ## Define some variables:
#        jntPosVelocity = JointPositionVelocity()
        cartesianPose  = PoseStamped()
#        startPose      = PoseStamped()
#        targetPose     = PoseStamped()
        trajectory     = Trajectory()
#        targetRange    = Trajectory()
        trajectoryHistoryList = []
        reachedPointsList = []

        ##callbackfunctions
#        def jntPosVelCallback(jntPosVelData):
#            jntPosVelocity.header = jntPosVelData.header
#            jntPosVelocity.position = jntPosVelData.position
#            jntPosVelocity.velocity = jntPosVelData.velocity
#            
        def cartesianPoseCallback(cartesianPose_msg):
            #cartesianPose.header = cartesianPose_msg.header
            cartesianPose.header.stamp = rospy.Time.now()
            cartesianPose.pose = cartesianPose_msg            
#        
#        def startPoseCallback(startPose_msg):
#            startPose.pose  = startPose_msg.pose
#            
#        def targetPoseCallback(targetPose_msg):
#            targetPose.pose = targetPose_msg.pose
        
        def trajectoryCallback(trajectory_msg):
            trajectory.header = trajectory_msg.header
            trajectory.poses  = trajectory_msg.poses
            trajectoryHistoryList.append(trajectory_msg)
            print('new trajectory received')
            
#        def targetRangeCallback(targetrange_msg):
#            targetRange.header = targetrange_msg.header
#            targetRange.poses  = targetrange_msg.poses  
        
        # Define subscribers
        rospy.Subscriber('robot_state_pose_pub', Pose, cartesianPoseCallback, queue_size=10)
        #rospy.Subscriber('trajectory/startPose',      PoseStamped, startPoseCallback,     queue_size=10)
        #rospy.Subscriber('trajectory/targetPose',     PoseStamped, targetPoseCallback,    queue_size=10)
        rospy.Subscriber('trajectory_pub', Trajectory, trajectoryCallback, queue_size=10)
        #rospy.Subscriber('trajectory/targetRange',    Trajectory,  targetRangeCallback,   queue_size=10)
        targetpose_pub = rospy.Publisher('targetpose_pub',PoseStamped,queue_size=10)
        
        
        # Blue line "drawn" by the qbSoftHand during its motion
        reachedPoints = Marker()
        reachedPoints.color = ColorRGBA(0,0,0,1)
        reachedPoints.ns = 'followedPath'
        reachedPoints.action  = Marker.ADD
        reachedPoints.header.frame_id = '/world'
        reachedPoints.type = Marker.LINE_STRIP
        reachedPoints.id = 0
        reachedPoints.scale.x = 0.005
        reachedPoints.scale.y = 0.005
        
        # Purple points to display the most recently generated trajectory 
        currentTrajectoryMarker = Marker()
        currentTrajectoryMarker.color = ColorRGBA(1,0,0,1)
        currentTrajectoryMarker.ns = 'trajectory'
        currentTrajectoryMarker.action  = Marker.ADD
        currentTrajectoryMarker.header.frame_id = '/world'
        currentTrajectoryMarker.type = Marker.LINE_STRIP
        currentTrajectoryMarker.id = 1
        currentTrajectoryMarker.scale.x = 0.005
        currentTrajectoryMarker.scale.y = 0.005
        
        # White line indicating the range within which the target pose can (linearly) vary.
#        targetRangeMarker = Marker()
#        targetRangeMarker.color = ColorRGBA(1,1,1,1)
#        targetRangeMarker.ns = 'targetRange'
#        targetRangeMarker.action = Marker.ADD
#        targetRangeMarker.header.frame_id = '/world'
#        targetRangeMarker.type = Marker.LINE_STRIP
#        targetRangeMarker.id = 2
#        
#        targetRangeMarker.scale.x = 0.005
#        targetRangeMarker.scale.y = 0.005
        
        # Red pose cube indicating the current pose of the qbSoftHand, located at the hand's palm link
#        currMarker = Marker()
#        currMarker.color = ColorRGBA(1,0,0,1)
#        currMarker.ns = 'currentPose'
#        currMarker.action  = Marker.ADD
#        currMarker.header.frame_id = '/world'
#        currMarker.type = Marker.CUBE
#        currMarker.id = 3
#        
#        currMarker.scale.x = 0.02
#        currMarker.scale.y = 0.02
#        currMarker.scale.z = 0.02
        
        # Yellow pose cube indicating the start pose of the most recently generated trajectory
#        startMarker = Marker()
#        startMarker.color = ColorRGBA(1,1,0,1)
#        startMarker.ns = 'startPose'
#        startMarker.action = Marker.ADD
#        startMarker.header.frame_id = '/world'
#        startMarker.type = Marker.CUBE
#        startMarker.id = 4
#        
#        startMarker.scale.x = 0.02
#        startMarker.scale.y = 0.02
#        startMarker.scale.z = 0.02
        
        # Green pose cube indicating the target pose of the most recently generated trajectory
#        targetMarker = Marker()
#        targetMarker.color = ColorRGBA(0,1,0,1)
#        targetMarker.ns = 'targetPose'
#        targetMarker.action  = Marker.ADD
#        targetMarker.header.frame_id = '/world'
#        targetMarker.type = Marker.CUBE
#        targetMarker.id = 5
#        
#        targetMarker.scale.x = 0.02
#        targetMarker.scale.y = 0.02
#        targetMarker.scale.z = 0.02
        
        # Black points that display the trajectory history, accumulation of all trajectory points  contained
        # in all trajectories being generated during the motion of the robot
        trajectoryHistoryMarker = Marker()
        trajectoryHistoryMarker.color = ColorRGBA(1,0,0,1)
        trajectoryHistoryMarker.ns = 'trajectoryHistory'
        trajectoryHistoryMarker.action = Marker.ADD
        trajectoryHistoryMarker.header.frame_id = '/world'
        trajectoryHistoryMarker.type = Marker.SPHERE_LIST
        trajectoryHistoryMarker.id = 6
        trajectoryHistoryMarker.scale.x = 0.005
        trajectoryHistoryMarker.scale.y = 0.005
        
        rospy.loginfo('Path visualization is up and running')
        #rospy.wait_for_message('trajectory_pub',Trajectory)
        
        while not rospy.is_shutdown():
            # Add the current position of the qbSoftHand to the 'blue line' indicating the followed path
            
            try: 
                reachedPoint = Point()
                reachedPoint.x = cartesianPose.pose.position.x
                reachedPoint.y = cartesianPose.pose.position.y
                reachedPoint.z = cartesianPose.pose.position.z
            
                # Avoid including (0,0,0) from empty topic at launch in the followed trajectory
                if not (reachedPoint.x==0 and reachedPoint.y==0 and reachedPoint.z==0):
                    reachedPointsList.append(reachedPoint)
                reachedPoints.points = reachedPointsList
                
    #            # Publish the current pose marker
    #            currMarker.header.stamp = rospy.Time.now()
    #            currMarker.pose = cartesianPose.pose
    #            visualPublisher.publish(currMarker)
    #            
    #            # Publish the target marker indicating the target pose of the generated trajectory
    #            targetMarker.pose = targetPose.pose
    #            visualPublisher.publish(targetMarker)
    #            
    #            # Publish the start marker indicating the start pose of the generated trajectory
    #            startMarker.pose = startPose.pose
    #            visualPublisher.publish(startMarker)
    
                # Publish the trajectory waypoints
                currentTrajectoryMarker.header.stamp = rospy.Time.now()
                currentTrajectoryMarker.points = []
                for pose in trajectory.poses:
                    point = pose.position
                    currentTrajectoryMarker.points.append(point)
                visualPublisher.publish(currentTrajectoryMarker)
                
                try:
                    targetpose = PoseStamped()
                    targetpose.header.stamp = rospy.Time.now()
                    targetpose.header.frame_id = '/world'
                    targetpose.pose = trajectory.poses[-1]
                    targetpose_pub.publish(targetpose)
                except:
                    continue
                
#                br = tf.TransformBroadcaster()
#                targetpose = trajectory.poses[-1]
#                br.sendTransform(targetpose.position,targetpose.orientation,rospy.Time.now(),'/targetpose','/world')

                
               # plot_obstacle(visualPublisher)    
                
                
    #            # Publish the target range
    #            targetRangeMarker.header.stamp = rospy.Time.now()
    #            targetRangeMarker.points = []
    #            for pose in targetRange.poses:
    #                point = pose.position
    #                targetRangeMarker.points.append(point)
    #            visualPublisher.publish(targetRangeMarker)
                
                # Publish the trajectory history 
                trajectoryHistoryMarker.header.stamp = rospy.Time.now()
                trajectoryHistoryMarker.points = []
                
                #print(len(trajectoryHistoryList))
                
                for trajectory_entry in trajectoryHistoryList:
                    #print(len(trajectory_entry.poses))
                    for pose in trajectory_entry.poses:
                        point = pose.position
                        trajectoryHistoryMarker.points.append(point)
                visualPublisher.publish(trajectoryHistoryMarker)
                
                # Publish the actual path followed by the hand
                visualPublisher.publish(reachedPoints)
                
                rate.sleep()

            except rospy.ROSTimeMovedBackwardsException:
                print "detected jump back in time, clearing markers"
                
                
                reachedPointsList = []
                reachedPoints.points = []
                cartesianPose.pose = Pose()
                
                trajectoryHistoryList = []
                trajectoryHistoryMarker.points = []
                trajectory = Trajectory()
                
                deleteAllMarkers(visualPublisher)
                
                pass
            
            except rospy.ROSException, e:
                print "ROSException: %s"%e

if __name__ == '__main__':
    
    rospy.init_node('rvizMarker_simulation', anonymous=False)
    visualPublisher = rospy.Publisher("visualization_marker", Marker , queue_size=10)
    
    sendMarkers(visualPublisher)
 
    
    
    
