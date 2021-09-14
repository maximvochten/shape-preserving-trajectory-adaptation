require "rttlib"
require "math"
tc = rtt.getTC()

------------ Ports -------------
-- Input ports
tracker_ros = rtt.InputPort("geometry_msgs/Pose", "tracker_ros", "Pose of tracker, read from ROS topic and set as endpose for robot")
tc:addPort(tracker_ros)

pose_robot = rtt.InputPort("KDL.Frame","pose_robot","Current pose of task frame")
tc:addPort(pose_robot)

-- Output ports
tracker_endpose = rtt.OutputPort("KDL.Frame","tracker_endpose","Pose of tracker, given to eTaSL")
tc:addPort(tracker_endpose)

---------- Properties ----------
pose_holder = rtt.Variable("KDL.Frame")

rot_quat = rtt.provides("KDL"):provides("Rotation")

-------- Hook functions --------
function configureHook()
	return true
end

function startHook()
	return true
end

function updateHook()
	local fs,data = tracker_ros:read()
        local fs2,pose_data = pose_robot:read()
	robot_pose = pose_data:totab()
	if (fs ~= 'NoData') then
		pose = data:totab()
		pose_holder.p.X = pose.position.x
		pose_holder.p.Y = pose.position.y
		pose_holder.p.Z = pose.position.z
		
		qx = pose.orientation.x
		qy = pose.orientation.y
		qz = pose.orientation.z
		qw = pose.orientation.w
		tracker_pose_rot = rot_quat:Quaternion(qx,qy,qz,qw)

		local f = rtt.Variable("KDL.Frame")
		pose_holder.M = tracker_pose_rot*f.M

		tracker_endpose:write(pose_holder)
	else
		pose_holder.p.X = robot_pose.p.X
	        pose_holder.p.Y = robot_pose.p.Y
       		pose_holder.p.Z = robot_pose.p.Z
		pose_holder.M = pose_data.M
       		tracker_endpose:write(pose_holder)
        end
end

function cleanupHook()
	rttlib.tc_cleanup()
end
