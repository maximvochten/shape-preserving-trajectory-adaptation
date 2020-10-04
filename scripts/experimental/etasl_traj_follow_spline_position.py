#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Maxim Vochten

ROS component that enables trajectory following in eTaSL
    input: trajectory (+velocity)
    output: robot joint velocities

Limitations etasl driver for Python: input/output channels can only be scalars at the moment, I would like frame/twist
"""

#!/usr/bin/env python
import rospy
import rospkg
import tf_conversions as tf
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseArray
#from etasl_invariants_integration.msg import TwistArray
from sensor_msgs.msg import JointState
from etasl_invariants_integration.msg import Trajectory
try:
    from etasl_py.etasl import etasl_simulator
except ImportError: # sometimes when reimporting it gives an error... (?)
    pass 
import numpy as np
import PyKDL as KDL


def closest_node(point, trajectory):
    ''' Returns the index of the point in the trajectory to which a given point lies closest'''
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
    ''' Make the skew-symmetric matrix '''
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
        #rospy.Subscriber("/trajectory_pub",Trajectory,self.callback_traj)

        # Initialize ROS publishers        
        self.current_pose_pub = rospy.Publisher('robot_state_pose_pub',Pose,queue_size=50)
        self.current_twist_pub = rospy.Publisher('robot_state_twist_pub',Twist,queue_size=50)
        self.current_progress_pub = rospy.Publisher('progress_partial',Float32,queue_size=50)
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
        simple_6DOF_robot_specification="""
            require("context")
            require("geometric")
            
            x = Variable{context=ctx,name="x",vartype='robot'}
            y = Variable{context=ctx,name="y",vartype='robot'}
            z = Variable{context=ctx,name="z",vartype='robot'}
            -- yaw = Variable{context=ctx,name="yaw",vartype='robot'}
            -- pitch = Variable{context=ctx,name="pitch",vartype='robot'}
            -- roll = Variable{context=ctx,name="roll",vartype='robot'}
            
            -- ee = translate_x(x) * translate_y(y) * translate_z(z) * rotate_z(yaw) * rotate_y(pitch) * rotate_x(roll)
            ee = translate_x(x) * translate_y(y) * translate_z(z)
            
            -- ctx:setOutputExpression("ee",ee)
        """
        
        sim.readTaskSpecificationString(simple_6DOF_robot_specification)
        #sim.displayContext()
        
        # Define trajectory tracking task
        point_trajectory_tracking_specification = """  
        -- Input trajectory coordinates
        tgt_x = ctx:createInputChannelScalar("tgt_x",0)
        tgt_y = ctx:createInputChannelScalar("tgt_y",0)
        tgt_z = ctx:createInputChannelScalar("tgt_z",0)
        Constraint{
            context=ctx,
            name='x tracking',
            expr = tgt_x - coord_x(origin(ee)),
            priority = 2,
            K = 10
        }
        Constraint{
            context=ctx,
            name='y tracking',
            expr = tgt_y - coord_y(origin(ee)),
            priority = 2,
            K = 10
        }
        Constraint{
            context=ctx,
            name='z tracking',
            expr = tgt_z - coord_z(origin(ee)),
            priority = 2,
            K = 10
        }
        ctx:setOutputExpression("error_x",coord_x(origin(ee))-tgt_x)
        ctx:setOutputExpression("error_y",coord_y(origin(ee))-tgt_y)
        ctx:setOutputExpression("error_z",coord_z(origin(ee))-tgt_z)
        """
        

        sim.readTaskSpecificationString(point_trajectory_tracking_specification)        


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
        input_labels = ['tgt_x','tgt_x','tgt_x'] # input variables 
        robot_labels = ['x','y','z'] # robot variables
        #robot_labels = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
        #current_robotpos = np.array([0.3441, 1.657, 1.601, 0.4956, -0.4489, 2.7519]) #np.array([0.14,1.5,1.72]) #        
        current_robotpos = np.array([0.14,1.5,1.72]) #np.array([0.14,1.5,1.72]) #        
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



     
if __name__ == '__main__':
    try:  
        # Initialization
        simul = EtaslSimulator()
        
        # Test this component on its own
        simul.standalone_test()

    except rospy.ROSInterruptException:
        pass