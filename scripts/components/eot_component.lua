require "rttlib"
require "math"
tc = rtt.getTC()

------------ Ports -------------
-- Input ports
tracker_ros = rtt.InputPort("geometry_msgs/Pose", "tracker_ros", "Pose of tracker, read from ROS topic and set as endpose for robot")
tc:addPort(tracker_ros)

twist_ros = rtt.InputPort("geometry_msgs/Twist", "twist_ros", "Twist of tracker")
tc:addPort(twist_ros)

pose_robot = rtt.InputPort("KDL.Frame","pose_robot","Current pose of task frame")
tc:addPort(pose_robot)

-- Output ports
tracker_endpose = rtt.OutputPort("KDL.Frame","tracker_endpose","Pose of tracker, given to eTaSL")
tc:addPort(tracker_endpose)

tracker_twist = rtt.OutputPort("KDL.Twist", "tracker_twist", "Twist of tracker")
tc:addPort(tracker_twist)
---------- Properties ----------
pose_holder = rtt.Variable("KDL.Frame")
twist_holder = rtt.Variable("KDL.Twist")
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
	local fs3,twist_data = twist_ros:read()
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

		twist = twist_data:totab()
                twist_holder.vel.X = twist.linear.x
                twist_holder.vel.Y = twist.linear.y
                twist_holder.vel.Z = twist.linear.z
                twist_holder.rot.X = twist.angular.x
                twist_holder.rot.Y = twist.angular.y
                twist_holder.rot.Z = twist.angular.z
		
		tracker_twist:write(twist_holder)
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
