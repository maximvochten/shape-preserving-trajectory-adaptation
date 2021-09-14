require "rttlib"
require "math"
tc = rtt.getTC()

------------ Ports -------------
-- Input ports
trajectory = rtt.InputPort("etasl_invariants_integration.Trajectory","trajectory","Discrete trajectory (=array of poses) from invariants")
tc:addPort(trajectory)

pose_robot = rtt.InputPort("KDL.Frame","pose_robot","Current pose of task frame")
tc:addPort(pose_robot)

start_trajectory = rtt.InputPort("KDL.Frame","start_trajectory", "Start of invariants trajectory, defined in eTaSL")
tc:addPort(start_trajectory)
-- Output ports
start_traj = rtt.OutputPort("geometry_msgs/Pose","start_traj", "Start of invariants trajectory, streamed to ROS Topic")
tc:addPort(start_traj)

pose_setpoint = rtt.OutputPort("KDL.Frame","pose_setpoint","Next pose in trajectory")
tc:addPort(pose_setpoint)

twist_setpoint = rtt.OutputPort("KDL.Twist","twist_setpoint","Next twist in trajectory")
tc:addPort(twist_setpoint)

current_progress = rtt.OutputPort("double","current_progress","The progress along the trajectory")
tc:addPort(current_progress)

---------- Properties ----------
start_traj_holder = rtt.Variable("geometry_msgs/Pose")
x = rtt.Variable("double")
y = rtt.Variable("double")
z = rtt.Variable("double")
w = rtt.Variable("double")

rot_quat = rtt.provides("KDL"):provides("Rotation")

------- Helper functions -------
function closest_node(point, traj)
	local closest_distance = 1000
	local closest_index = 0
	for i=1,#traj do 		
		local dist_2 = (point.p.X - traj[i].position.x)^2 + (point.p.Y - traj[i].position.y)^2 + (point.p.Z - traj[i].position.z)^2
		if (dist_2 < closest_distance) then
			closest_distance = dist_2
			closest_index = i
		end
	end
	return closest_index
end

-------- Hook functions --------
function configureHook()
	return true
end

function startHook()
	return true
end

function updateHook()
	local fs,data = trajectory:read()
	local fs2,pose_data = pose_robot:read()
	local fs3,start_traj_data = start_trajectory:read()
        pose = pose_data:totab()
	start_traj_tab = start_traj_data:totab()
        
	local pose_holder = rtt.Variable("KDL.Frame")
        local twist_holder = rtt.Variable("KDL.Twist")

	if (fs ~= 'NoData') then
                datatab = data:totab()
		pose_traj = datatab.poses
                twist_traj = datatab.twists
		closest_index = closest_node(pose,pose_traj)
		if (closest_index <= #pose_traj-1) then
                	current_progress:write(closest_index/#pose_traj)
		
			-- Set target pose as point closest to robot end-effector (task frame)
                	pose_holder.p.X = pose_traj[closest_index].position.x
                	pose_holder.p.Y = pose_traj[closest_index].position.y
                	pose_holder.p.Z = pose_traj[closest_index].position.z

                	qx = pose_traj[closest_index].orientation.x
                	qy = pose_traj[closest_index].orientation.y
                	qz = pose_traj[closest_index].orientation.z
                	qw = pose_traj[closest_index].orientation.w
                	obj_pose_rot = rot_quat:Quaternion(qx,qy,qz,qw)

                	local f = rtt.Variable("KDL.Frame")
                	pose_holder.M = obj_pose_rot*f.M

			-- Set target twist
			twist_holder.vel.X = twist_traj[closest_index].linear.x
			twist_holder.vel.Y = twist_traj[closest_index].linear.y
			twist_holder.vel.Z = twist_traj[closest_index].linear.z

			twist_holder.rot.X = twist_traj[closest_index].angular.x
			twist_holder.rot.Y = twist_traj[closest_index].angular.y
			twist_holder.rot.Z = twist_traj[closest_index].angular.z
		
                	pose_setpoint:write(pose_holder)
			twist_setpoint:write(twist_holder)
		end
	else
		pose_holder.p.X = pose.p.X
	        pose_holder.p.Y = pose.p.Y
       		pose_holder.p.Z = pose.p.Z
		pose_holder.M = pose_data.M
       		pose_setpoint:write(pose_holder)

        end

	if (fs3 ~= 'NoData') then
		start_traj_holder.position.x = start_traj_tab.p.X
		start_traj_holder.position.y = start_traj_tab.p.Y
		start_traj_holder.position.z = start_traj_tab.p.Z
		rot_quat:GetQuaternion(start_traj_data.M,x,y,z,w)
		start_traj_holder.orientation.x = x
		start_traj_holder.orientation.y = y
		start_traj_holder.orientation.z = z
		start_traj_holder.orientation.w = w
		start_traj:write(start_traj_holder)

	end

end

function cleanupHook()
	rttlib.tc_cleanup()
end
