require "rttlib"
require "math"
tc = rtt.getTC()

---------- Ports ----------
-- Input ports
bspline_parameters = rtt.InputPort("ros_spline_fitting_trajectory.Bsplines", "bspline_parameters", "Parameters of bsplines given by trajectory spline fitter")
tc:addPort(bspline_parameters)

velprof_controlpoints = rtt.InputPort("etasl_invariants_integration.VelocityProfile", "velprof_controlpoints", "Control points of the bspline approximating the demonstrated velocity profile")
tc:addPort(velprof_controlpoints)

pose_robot = rtt.InputPort("KDL.Frame", "pose_robot", "Current pose of task frame")
tc:addPort(pose_robot)

start_trajectory = rtt.InputPort("KDL.Frame", "start_trajectory", "...")
tc:addPort(start_trajectory)

s_g = rtt.InputPort("array", "s_g", "test")
tc:addPort(s_g)

-- Output ports
start_traj = rtt.OutputPort("geometry_msgs/Pose", "start_traj", "...")
tc:addPort(start_traj)

cp_x = rtt.OutputPort("array", "cp_x", "test")
tc:addPort(cp_x)
cp_y = rtt.OutputPort("array", "cp_y", "test")
tc:addPort(cp_y)
cp_z = rtt.OutputPort("array", "cp_z", "test")
tc:addPort(cp_z)

cp_qx = rtt.OutputPort("array", "cp_qx", "test")
tc:addPort(cp_qx)
cp_qy = rtt.OutputPort("array", "cp_qy", "test")
tc:addPort(cp_qy)
cp_qz = rtt.OutputPort("array", "cp_qz", "test")
tc:addPort(cp_qz)
cp_qw = rtt.OutputPort("array", "cp_qw", "test")
tc:addPort(cp_qw)

cp_velprof = rtt.OutputPort("array", "cp_velprof", "test")
tc:addPort(cp_velprof)

s_offset = rtt.OutputPort("double", "s_offset", "test")
tc:addPort(s_offset)

robot_pose_msg = rtt.OutputPort("geometry_msgs/Pose", "robot_pose_msg", "...")
tc:addPort(robot_pose_msg)

test_flag = rtt.OutputPort("double", "test_flag", "test")
tc:addPort(test_flag)

---------- Properties ----------
start_traj_holder = rtt.Variable("geometry_msgs/Pose")

x = rtt.Variable("double")
y = rtt.Variable("double")
z = rtt.Variable("double")
w = rtt.Variable("double")
switch_flag = rtt.Variable("double")

holder_velprof = rtt.Variable("array")

previous_progress = rtt.Variable("double")


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
		--else
		--	break
                end
        end
        return closest_index
end

---------- Hook functions ----------
function configureHook()
        return true
end

function startHook()
	switch_flag = 0.0
        previous_progress = 0.0
	
	return true
end

function updateHook()
        local fs,data = bspline_parameters:read()
        local fs2,robot_data = pose_robot:read()
        robot_pose = robot_data:totab()
        local fs3, start_traj_data = start_trajectory:read()
        start_traj_tab = start_traj_data:totab()
	local fs4, velprof_data = velprof_controlpoints:read() 
	local fs5, s_g_data = s_g:read()

	if (fs ~= 'NoData') then
                datatab = data:totab()
                pose_traj = datatab.poses
                knots = datatab.knots
                cp = datatab.control_points
		quat_cp = datatab.control_quaternion
		
		test_flag:write(1.0)

		local holder_cpx = rtt.Variable("array")
		local holder_cpy = rtt.Variable("array")
		local holder_cpz = rtt.Variable("array")

		local holder_quatx = rtt.Variable("array")
                local holder_quaty = rtt.Variable("array")
                local holder_quatz = rtt.Variable("array")
                local holder_quatw = rtt.Variable("array")

		holder_cpx:resize(7)
		holder_cpy:resize(7)
		holder_cpz:resize(7)

		holder_quatx:resize(7)
                holder_quaty:resize(7)
                holder_quatz:resize(7)
                holder_quatw:resize(7)
                for i=1,7 do
			holder_cpx[i-1] = cp[i].x
			holder_cpy[i-1] = cp[i].y
			holder_cpz[i-1] = cp[i].z

                        holder_quatx[i-1] = quat_cp[i].x
                        holder_quaty[i-1] = quat_cp[i].y
                        holder_quatz[i-1] = quat_cp[i].z
                        holder_quatw[i-1] = quat_cp[i].w
                end
		cp_x:write(holder_cpx)
		cp_y:write(holder_cpy)
		cp_z:write(holder_cpz)

                cp_qx:write(holder_quatx)
                cp_qy:write(holder_quaty)
                cp_qz:write(holder_quatz)
                cp_qw:write(holder_quatw)

		if (fs == 'NewData') then
			local s_g_tab = s_g_data:totab()
			local current_subtraction = s_g_tab[1] - previous_progress
        		s_offset:write(current_subtraction)
		end

	else
		local holder_cpx = rtt.Variable("array")
                local holder_cpy = rtt.Variable("array")
                local holder_cpz = rtt.Variable("array")

		local holder_quatx = rtt.Variable("array")
		local holder_quaty = rtt.Variable("array")
		local holder_quatz = rtt.Variable("array")
		local holder_quatw = rtt.Variable("array")

		holder_cpx:resize(7)
		holder_cpy:resize(7)
		holder_cpz:resize(7)

		holder_quatx:resize(7)
		holder_quaty:resize(7)
		holder_quatz:resize(7)
		holder_quatw:resize(7)

		local qx = rtt.Variable("double")
		local qy = rtt.Variable("double")
		local qz = rtt.Variable("double")
		local qw = rtt.Variable("double")

		rot_quat:GetQuaternion(start_traj_data.M,qx,qy,qz,qw)

		for i=1,7 do
			holder_cpx[i-1] = robot_data.p.X
                        holder_cpy[i-1] = robot_data.p.Y
                        holder_cpz[i-1] = robot_data.p.Z

			holder_quatx[i-1] = qx
			holder_quaty[i-1] = qy
			holder_quatz[i-1] = qz
			holder_quatw[i-1] = qw
		end

		cp_x:write(holder_cpx)
		cp_y:write(holder_cpy)
		cp_z:write(holder_cpz)

		cp_qx:write(holder_quatx)
		cp_qy:write(holder_quaty)
		cp_qz:write(holder_quatz)
		cp_qw:write(holder_quatw)
	end


	if (fs2 ~= 'NoData') then
                local robot_pose_holder = rtt.Variable("geometry_msgs/Pose")
                local qx = rtt.Variable("double")
                local qy = rtt.Variable("double")
                local qz = rtt.Variable("double")
                local qw = rtt.Variable("double")
                robot_pose_holder.position.x = robot_pose.p.X
                robot_pose_holder.position.y = robot_pose.p.Y
                robot_pose_holder.position.z = robot_pose.p.Z
                rot_quat:GetQuaternion(robot_data.M,qx,qy,qz,qw)
                robot_pose_holder.orientation.x = qx
                robot_pose_holder.orientation.y = qy
                robot_pose_holder.orientation.z = qz
                robot_pose_holder.orientation.w = qw

                robot_pose_msg:write(robot_pose_holder)
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

	if (fs4 == 'NewData') then
		velprof_tab = velprof_data:totab()
		cp_vp = velprof_tab.control_velocityprofile
		holder_velprof:resize(7)
		for i = 1,7 do
			holder_velprof[i-1] = cp_vp[i]
		end
		cp_velprof:write(holder_velprof)

	end

end

function cleanupHook()
        rttlib.tc_cleanup()
end


