#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Maxim Vochten

ROS component that enables trajectory following in eTaSL
    input: trajectory (+velocity)
    output: robot joint velocities

Limitations etasl driver for Python: input/output channels can only be scalars at the moment, I would like frame/twist
"""


"""Load required modules"""
import rospy
import rospkg

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseArray
#from etasl_invariants_integration.msg import TwistArray
from etasl_invariants_integration.msg import Trajectory # custom message
from sensor_msgs.msg import JointState
import tf_conversions as tf

try:
    from etasl_py.etasl import etasl_simulator
except ImportError: # sometimes when reimporting it gives an error... I skip this error in this way
    pass 
import numpy as np

import PyKDL as KDL

'''Load optional modules'''
from urdf_parser_py.urdf import URDF # installed through ros melodic urdfdom-py
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model # download hrl-kdl from https://github.com/gt-ros-pkg/hrl-kdl/
# Robot name: panda, iiwa, gen3 (Kinova), lwr, ur10
robot_name = 'ur10'
# Retrieve kinematic chain of the robot from URDF file
rospack = rospkg.RosPack()
robot_urdf = URDF.from_xml_file(rospack.get_path('etasl_invariants_integration')+'/robot_description/urdf/'+robot_name+'/use_case_setup_'+robot_name+'.urdf')
robot_urdf_kdl_tree = kdl_tree_from_urdf_model(robot_urdf)
#[lowerlimits,upperlimits,names] = urdf_get_joint_limits(ur10_urdf)
robot_ee = robot_urdf_kdl_tree.getChain('world','TCP_frame')
Nq = robot_ee.getNrOfJoints()

# Define forward position kinematics solver + inverse position kinematics solver
fkpos_ee = KDL.ChainFkSolverPos_recursive(robot_ee)
ikvel_ee = KDL.ChainIkSolverVel_pinv(robot_ee)
ikpos_ee = KDL.ChainIkSolverPos_NR(robot_ee,fkpos_ee,ikvel_ee)

# Initialize empty frame and joint arrays
Finit = KDL.Frame()
qinit = KDL.JntArray(Nq)
qinit2 = KDL.JntArray(Nq)

# Test forward kinematics
current_robotpos = np.array([0,-np.pi*0.6,np.pi*0.6,-2.0,-np.pi*0.5,0.1])
qinit = KDL.JntArray(Nq)
for i in range(Nq):
    qinit[i] = current_robotpos[i]  
fkpos_ee.JntToCart(qinit,Finit)

# Test inverse kinematics
Finit = KDL.Frame(KDL.Rotation.EulerZYX(-1.4465,0.3998,-2.0707), KDL.Vector(0.4,-0.4,0.9))
ikpos_ee.CartToJnt(qinit,Finit,qinit2)
print qinit2


def closest_node(point, trajectory):
    ''' Returns the index of the point in the trajectory to which a given point lies the closest'''
    trajectory = np.asarray(trajectory) # list to array
    dist_2 = np.sum((trajectory - point)**2, axis=1) # Euclidean distances
    return np.argmin(dist_2)

def array_to_dict( arr, labels):
    """
    (copied from Erwin's etasl driver)
    
    def array_to_dict( arr, labels):
    
    Args:
        arr: array of values
        labels: corresponding labels
        
    Requirements:
        len(arr) == len(labels)
        
    Returns:
        d: python dictionary of the labels containing the corresponding values.
           i.e. such that d[labels[i]] == arr[i]
    """
    return dict(zip(labels,arr.flatten()))

def skew(omega):
    ''' Construct the skew-symmetric matrix '''
    return np.array([ [0,-omega[2],omega[1]],
                      [omega[2],0,-omega[0]],
                      [-omega[1],omega[0],0]])

def calculate_pose_derivative (pose, twist):
    '''Calculate derivative of pose matrix from pose twist: pdot = v; Rdot = [omega] R'''
    R_mat = np.reshape(pose[0:9],[3,3],order='F')
    R_deriv_mat = skew(twist[0:3]).dot(R_mat)
    return np.append( R_deriv_mat.flatten('F') , twist[3:6] )

class EtaslSimulator:

    def __init__(self):       
        
        rospy.init_node('etasl_simulator', anonymous=True)
        
        # Initialize ROS subscribers        
        #rospy.Subscriber("/pose_traj_pub",PoseArray,self.callback_pose_traj)
        #rospy.Subscriber("/twist_traj_pub",TwistArray,self.callback_twist_traj) 
        rospy.Subscriber("/trajectory_pub",Trajectory,self.callback_traj)

        # Initialize ROS publishers        
        self.start_traj_pub = rospy.Publisher('start_traj_pub',Pose,queue_size=50)
        self.current_pose_pub = rospy.Publisher('current_pose_pub',Pose,queue_size=50)
        self.current_twist_pub = rospy.Publisher('current_twist_pub',Twist,queue_size=50)
        self.current_progress_pub = rospy.Publisher('progress_partial',Float64,queue_size=50)
        self.jointState_pub = rospy.Publisher('/joint_states', JointState, queue_size=50)
        
        rospack = rospkg.RosPack()
        
        self.target_pose = Pose()
        self.target_twist = Twist()
        self.pose_traj = np.zeros([1,12])
        self.twist_traj = np.zeros([1,6])
        self.stored_joints = []
        self.results_folder = rospack.get_path('etasl_invariants_integration') + '/data/results/'
        
        # Define etasl simulator
        sim = etasl_simulator(regularization_factor = 0.000001)
    
        # Define robot in etasl
        #TODO everything related to etasl specifications can be stored in separate lua files and loaded here
        simple_6DOF_robot_specification="""
            require("context")
            require("geometric")
            
            x = Variable{context=ctx,name="x",vartype='robot'}
            y = Variable{context=ctx,name="y",vartype='robot'}
            z = Variable{context=ctx,name="z",vartype='robot'}
            yaw = Variable{context=ctx,name="yaw",vartype='robot'}
            pitch = Variable{context=ctx,name="pitch",vartype='robot'}
            roll = Variable{context=ctx,name="roll",vartype='robot'}
            
            ee = translate_x(x) * translate_y(y) * translate_z(z) * rotate_z(yaw) * rotate_y(pitch) * rotate_x(roll)
            
            -- ctx:setOutputExpression("ee",ee)
        """
        
        robot="""
            require("context")
            require("geometric")
            -- Robot:
            robot_name = "ur10"
            local u=UrdfExpr();
            local fn = rospack_find("etasl_invariants_integration").."/robot_description/urdf/"..robot_name.."/use_case_setup_"..robot_name..".urdf"
            u:readFromFile(fn)
            u:addTransform("ee","TCP_frame","base_link")
            local r = u:getExpressions(ctx)
            ee = r.ee
            robot_jname={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}
            robot_jval = {}
            for i=1,#robot_jname do
               robot_jval[i]   = ctx:getScalarExpr(robot_jname[i])
            end
        """
                
        sim.readTaskSpecificationString(robot)
        
        #sim.displayContext()
        
#        # Define trajectory tracking task
#        point_trajectory_tracking_specification = """  
#        -- Input trajectory coordinates
#        tgt_x = ctx:createInputChannelScalar("tgt_x",0)
#        tgt_y = ctx:createInputChannelScalar("tgt_y",0)
#        tgt_z = ctx:createInputChannelScalar("tgt_z",0)
#        Constraint{
#            context=ctx,
#            name='x tracking',
#            expr = tgt_x - coord_x(origin(ee)),
#            priority = 2,
#            K = 10
#        }
#        Constraint{
#            context=ctx,
#            name='y tracking',
#            expr = tgt_y - coord_y(origin(ee)),
#            priority = 2,
#            K = 10
#        }
#        Constraint{
#            context=ctx,
#            name='z tracking',
#            expr = tgt_z - coord_z(origin(ee)),
#            priority = 2,
#            K = 10
#        }
#        ctx:setOutputExpression("error_x",coord_x(origin(ee))-tgt_x)
#        ctx:setOutputExpression("error_y",coord_y(origin(ee))-tgt_y)
#        ctx:setOutputExpression("error_z",coord_z(origin(ee))-tgt_z)
#        """
        
        # Define trajectory tracking task
        pose_trajectory_tracking_specification = """  
        -- Input trajectory coordinates
        
        R_11 = ctx:createInputChannelScalar("R_11",0)
        R_12 = ctx:createInputChannelScalar("R_12",0)
        R_13 = ctx:createInputChannelScalar("R_13",0)
        R_21 = ctx:createInputChannelScalar("R_21",0)
        R_22 = ctx:createInputChannelScalar("R_22",0)
        R_23 = ctx:createInputChannelScalar("R_23",0)
        R_31 = ctx:createInputChannelScalar("R_31",0)
        R_32 = ctx:createInputChannelScalar("R_32",0)
        R_33 = ctx:createInputChannelScalar("R_33",0)   
        p_x = ctx:createInputChannelScalar("p_x",0)
        p_y = ctx:createInputChannelScalar("p_y",0)
        p_z = ctx:createInputChannelScalar("p_z",0)
             
        R1 = vector(R_11,R_12,R_13)
        R2 = vector(R_21,R_22,R_23)
        R3 = vector(R_31,R_32,R_33)
        R = construct_rotation_from_vectors(R1,R2,R3)
        p = vector(p_x,p_y,p_z)
        Td = frame(R,p)
        
        Constraint{
            context=ctx,
            name='pose tracking',
            expr = inv(Td) * ee,
            priority = 2,
            K = 10
        }
        ctx:setOutputExpression("positionerror_x", coord_x(origin(inv(Td)*ee)))
        ctx:setOutputExpression("positionerror_y", coord_y(origin(inv(Td)*ee)))
        ctx:setOutputExpression("positionerror_z", coord_z(origin(inv(Td)*ee)))
        """
        sim.readTaskSpecificationString(pose_trajectory_tracking_specification)        

        ## ETASL DOCUMENTATION
        #    [[
        #        function distance_between( frame1, shape1, radius1, margin1, frame2, shape2, radius2, margin2)
        #            create an expression for the distance between two shapes
        #                - frame1   : expression for the pose of shape1
        #                - shape1   : object representing a shape1
        #                - radius1  : the shape can be spherically expanded by a given radius1
        #                - margin1  : margin used to make the GJK algorithm more robust
        #                - frame2   : expression for the pose of shape2
        #                - shape2   : object representing a shape2
        #                - radius2  : the shape can be spherically expanded by a given radius2
        #                - margin2  : margin used to make the GJK algorithm more robust
        #    ]]

        obstacle_avoidance_specification="""
        require("libexpressiongraph_collision")
        
        obstacle_pose = translate_x(0.036)*translate_y(-0.9)*translate_z(0.31)
        
        local d = distance_between( obstacle_pose, CylinderZ(0.1,0.1,2.0), 0.00, 0.001, ee, Box(0.06,0.06,0.1), 0.00, 0.001 )
        
        Constraint {
            context = ctx,
            name    = "collision avoidance",
            expr    = d,
            target_lower = 0,
            weight = 1,
            K       = 2, -- interpretation: how fast are we allowed to approach
            priority = 1 
        }
        """
        
        #sim.readTaskSpecificationString(obstacle_avoidance_specification)
        
        #sim.displayContext()
        
        self.sim = sim

    def standalone_test(self):
        '''Use this for testing this component in isolation from the trajectory generator using pre-computed trajectories'''
        
        # Define when trajectories should switch
        progress_sequence = np.array([0 , 0.2+0.1 , 0.4+0.1 , 0.6+0.1 , 0.8+0.1 , 1]) #global progress triggers, copied from invariants
        local_progress_triggers = (progress_sequence[1:]-progress_sequence[:-1])/(1 - progress_sequence[:-1]) #local progress triggers
        
        # Initialization etasl controller
        controller_freq = 200 # frequency in Hz
        rate = rospy.Rate(controller_freq)    
        
        input_labels = ['R_11','R_12','R_13','R_21','R_22','R_23','R_31','R_32','R_33','p_x','p_y','p_z'] # input variables 
        
        #robot_labels = ['x','y','z','yaw','pitch','roll'] # robot variables
        robot_labels = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
        
        #current_robotpos = np.array([0.3441, 1.657, 1.601, 0.4956, -0.4489, 2.7519]) #np.array([0.14,1.5,1.72]) #        
        #current_robotpos = np.array([-1.2913, -1.68006, 1.3826, -1.16354, -0.460297, -1.24451]) #np.array([0.14,1.5,1.72]) #        
        current_robotpos = np.array([-1.2366, -1.79548, 1.40716, -1.18334, -0.4573147,  -1.12111])
        
        self.sim.initialize(current_robotpos,robot_labels)
 
        # Gather results in this list
        joint_list = []    
        starttime = rospy.get_time()
        
        # Progress over the different trajectories
        count = 0 # this variable holds the index of the current trajectory
        while count <= len(progress_sequence)-2 and rospy.get_time()-starttime < 10.0: # while there are still trajectories left
            
            # Load in global trajectory of pose and twist
            pose_traj = np.loadtxt(self.results_folder + 'traj'+str(count) + '.txt')[:,1:]
            twist_traj = np.loadtxt(self.results_folder  + 'twist'+str(count) + '.txt')[:,1:]
            N = float(len(pose_traj))

            # Find closest point to trajectory + estimate local progress along current trajectory
            Finit=KDL.Frame()
            qinit = KDL.JntArray(Nq)
            for i in range(Nq):
                qinit[i] = current_robotpos[i]            
            fkpos_ee.JntToCart(qinit,Finit)
            #print Finit
            
            joint_list.append(np.append(rospy.get_time(),np.append(0,tf.toMatrix(Finit)[0:3,:].flatten('F'))))
            
            closest_index = closest_node(tf.toMatrix(Finit)[0:3,3],pose_traj[:,9:12])
            local_progress_var = closest_index/(N-1)
            
            # Progress over current trajectory until you hit trigger for switching to next trajectory
            while local_progress_var < local_progress_triggers[count] and rospy.get_time()-starttime < 10.0:
                
                # Get setpoint for pose and twist
                pose_setpoint = pose_traj[closest_index]
                twist_setpoint = twist_traj[closest_index]
                pose_derivative_setpoint = calculate_pose_derivative(pose_setpoint,twist_setpoint)
                
                # Set variables in etasl controller, perform 1 simulation step and save joints in a variable
                self.sim.initial_jpos = array_to_dict(np.append(0,current_robotpos),['time']+robot_labels)
                self.sim.setInputTable(input_labels,[pose_setpoint],[pose_derivative_setpoint])
                self.sim.simulate(N=1,dt=1./controller_freq)
                current_robotpos = self.sim.POS[0]
                #print current_robotpos
                #rospy.loginfo(current_robotpos)
                
                print current_robotpos
                print self.sim.OUTP

                # Find closest point to trajectory + estimate progress along current trajectory
                Finit=KDL.Frame()
                qinit = KDL.JntArray(Nq)
                for i in range(Nq):
                    qinit[i] = current_robotpos[i]            
                fkpos_ee.JntToCart(qinit,Finit)
                #print qinit
                
                closest_index = closest_node(tf.toMatrix(Finit)[0:3,3],pose_traj[:,9:12])
                local_progress_var = closest_index/(N-1)
                joint_list.append(np.append(rospy.get_time(),np.append(local_progress_var,tf.toMatrix(Finit)[0:3,:].flatten('F'))))

#                p = Pose()
#                p.position.x = pose_setpoint
#                p.position.y = current_robotpos[1]
#                p.position.z = current_robotpos[2]
#                mat = np.append(np.reshape(pose_traj[closest_index],[3,4],order='F'),[[0,0,0,1]],axis=0)
#                p.orientation = tf.toMsg(tf.fromMatrix(mat)).orientation
#                self.current_pose_pub.publish(p)
                #Toon implementation
                #self.current_pose_pub.publish(tf.toMsg(tf.fromMatrix(np.transpose(np.reshape(pose_setpoint,[4,3])))))                       
                self.current_pose_pub.publish(tf.toMsg(tf.fromMatrix(np.append(tf.toMatrix(Finit)[0:3,:],[[0,0,0,1]],axis=0))))
                
                t = Twist()
                t.angular.x = twist_setpoint[0]
                t.angular.y = twist_setpoint[1]
                t.angular.z = twist_setpoint[2]
                t.linear.x  = twist_setpoint[3]
                t.linear.y  = twist_setpoint[4]
                t.linear.z  = twist_setpoint[5]
                self.current_twist_pub.publish(t)
                        
                jointState = JointState()
                jointState.header.stamp = rospy.Time.now()
                jointState.name         = robot_labels
                jointState.position     = current_robotpos
                jointState.velocity     = self.sim.VEL[0]
                self.jointState_pub.publish(jointState)

                rospy.loginfo('local progress:' + str(local_progress_var) + ' , index:' + str(closest_index))
                rate.sleep()
                
            rospy.loginfo('switched to next traj: ' + str(count+1))
            count += 1
            
        rospy.loginfo('finished etasl')
        np.savetxt(self.results_folder + 'joints.txt' , np.array(joint_list))

    def loop_etasl_simulator(self):
        '''Follow trajectory'''
        
        # Initialization controller
        controller_freq = 200 # frequency
        rate = rospy.Rate(controller_freq)
        input_labels = ['R_11','R_12','R_13','R_21','R_22','R_23','R_31','R_32','R_33','p_x','p_y','p_z'] # input variables  
        #robot_labels = ['x','y','z','yaw','pitch','roll'] # robot variables
        robot_labels = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
#        current_robotpos = np.array([0.3417, 1.65, 1.7, 0.6956, -0.6489, 3.519])
        startpos_ee = KDL.Frame()
        qinit = KDL.JntArray(Nq)
        current_robotpos = np.array([-1.2366, -1.79548, 1.40716, -1.18334, -0.4573147,  -1.12111])
        for i in range(Nq):
            qinit[i] = current_robotpos[i]
        fkpos_ee.JntToCart(qinit, startpos_ee)
        while self.start_traj_pub.get_num_connections() < 1:
            pass
        self.start_traj_pub.publish(tf.toMsg(startpos_ee))
        
        self.sim.initialize(current_robotpos,robot_labels)
        local_progress_var = 0 # progress along current trajectory
        

        rospy.loginfo('waiting for first trajectory to arrive')
        #rospy.wait_for_message("/pose_traj_pub",PoseArray)
        #rospy.wait_for_message("/twist_traj_pub",TwistArray)
        rospy.wait_for_message("/trajectory_pub",Trajectory)
            
        starttime = rospy.get_time()


        # Gather results in this list
        Finit=KDL.Frame()
        qinit = KDL.JntArray(Nq)
        for i in range(Nq):
            qinit[i] = current_robotpos[i]            
        fkpos_ee.JntToCart(qinit,Finit)
        joint_list = []
        joint_list.append(np.append(rospy.get_time(),np.append(local_progress_var,tf.toMatrix(Finit)[0:3,:].flatten('F'))))
                
        # Load in global trajectory of pose and twist
        pose_traj = self.pose_traj
        twist_traj = self.twist_traj
        N = float(len(self.pose_traj))
        
        # Find closest point to trajectory + estimate local progress along current trajectory
        closest_index = closest_node(tf.toMatrix(Finit)[0:3,3],pose_traj[:,9:12])        
        local_progress_var = closest_index/(N-1)
        
        # Progress until we are at the end of the current trajectory
        while local_progress_var < 1 and rospy.get_time()-starttime < 20.0 : 
                                   
            # Get setpoint for pose and twist
            #if closest_index == N-2:
            #    closest_index = closest_index-1
            pose_setpoint = pose_traj[closest_index]
            twist_setpoint = twist_traj[closest_index]    
            pose_derivative_setpoint = calculate_pose_derivative(pose_setpoint,twist_setpoint)
            
            # Set variables in etasl controller, perform 1 simulation step and save joints in a variable
            self.sim.initial_jpos = array_to_dict(np.append(0,current_robotpos),['time']+robot_labels)
            self.sim.setInputTable(input_labels,[pose_setpoint],[pose_derivative_setpoint])
            self.sim.simulate(N=1,dt=1./controller_freq)
            current_robotpos = self.sim.POS[0]
            joint_list.append(np.append(rospy.get_time(),np.append(local_progress_var,tf.toMatrix(Finit)[0:3,:].flatten('F'))))
            #rospy.loginfo(current_robotpos)
            
            # Load in global trajectory of pose and twist
            pose_traj = self.pose_traj
            twist_traj = self.twist_traj
            N = float(len(self.pose_traj))
            
            # Find closest point to trajectory + estimate progress along current trajectory
            Finit = KDL.Frame()
            qinit = KDL.JntArray(Nq)
            for i in range(Nq):
                qinit[i] = current_robotpos[i]            
            fkpos_ee.JntToCart(qinit,Finit)
            #print qinit
            
            closest_index = 1+closest_node(tf.toMatrix(Finit)[0:3,3],pose_traj[:,9:12])
            #if closest_index == N:# or closest_index == N:
            #    closest_index = closest_index-1
            local_progress_var = closest_index/(N-1)
            rospy.loginfo('local progress:' + str(local_progress_var) + ' , index:' + str(closest_index))
            self.current_progress_pub.publish(local_progress_var)
            
            #local_progress_var = (closest_index-1)/(N-1)
            
#                p = Pose()
#                p.position.x = pose_setpoint
#                p.position.y = current_robotpos[1]
#                p.position.z = current_robotpos[2]
#                mat = np.append(np.reshape(pose_traj[closest_index],[3,4],order='F'),[[0,0,0,1]],axis=0)
#                p.orientation = tf.toMsg(tf.fromMatrix(mat)).orientation
#                self.current_pose_pub.publish(p)
            #Toon implementation
            #self.current_pose_pub.publish(tf.toMsg(tf.fromMatrix(np.transpose(np.reshape(pose_setpoint,[4,3])))))                       
            self.current_pose_pub.publish(tf.toMsg(tf.fromMatrix(np.append(tf.toMatrix(Finit)[0:3,:],[[0,0,0,1]],axis=0))))
            
            t = Twist()
            t.angular.x = twist_setpoint[0]
            t.angular.y = twist_setpoint[1]
            t.angular.z = twist_setpoint[2]
            t.linear.x  = twist_setpoint[3]
            t.linear.y  = twist_setpoint[4]
            t.linear.z  = twist_setpoint[5]
            self.current_twist_pub.publish(t)
                    
            jointState = JointState()
            jointState.header.stamp = rospy.Time.now()
            jointState.name         = robot_labels
            jointState.position     = current_robotpos
            jointState.velocity     = self.sim.VEL[0]
            self.jointState_pub.publish(jointState)
       
            rate.sleep()
            
        rospy.loginfo('finished')
        np.savetxt(self.results_folder  + 'joints.txt',np.array(joint_list))
                                            
    def callback_pose_traj(self,data):
        pose_list = []
        for pose in data.poses:
            pose_list.append(tf.toMatrix(tf.fromMsg(pose))[:3,:].flatten('F'))
        self.pose_traj = np.asarray(pose_list)
        
    def callback_twist_traj(self,data):
        twist_list = []
        for t in data.twists:
            twist_list.append(np.array([t.angular.x,t.angular.y,t.angular.z,t.linear.x,t.linear.y,t.linear.z]))
        self.twist_traj = twist_list
        
    def callback_traj(self,data):
        twist_list = []
        for t in data.twists:
            twist_list.append(np.array([t.angular.x,t.angular.y,t.angular.z,t.linear.x,t.linear.y,t.linear.z]))
        self.twist_traj = twist_list
        
        pose_list = []
        for pose in data.poses:
            pose_list.append(tf.toMatrix(tf.fromMsg(pose))[:3,:].flatten('F'))
        self.pose_traj = np.asarray(pose_list)
        rospy.loginfo('received new trajectory')
        
     
if __name__ == '__main__':
    try:  
        # Initialization
        simul = EtaslSimulator()
        
        # Test this component on its own
        test_standalone = False
        
        if test_standalone:
            simul.standalone_test()
        else:      
            simul.loop_etasl_simulator()
            
    except rospy.ROSInterruptException:
        pass
