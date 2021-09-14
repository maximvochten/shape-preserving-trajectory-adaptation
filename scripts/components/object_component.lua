require "rttlib"
require "math"
tc = rtt.getTC()

------------ Ports -------------
-- Input ports
object_ros = rtt.InputPort("geometry_msgs/Pose", "object_ros", "Pose of object, read from ROS topic and set in the obstacle avoidance script")
tc:addPort(object_ros)

-- Output ports
object_pose = rtt.OutputPort("KDL.Frame","object_pose","Pose of object, given to eTaSL for obstacle avoidance")
tc:addPort(object_pose)

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
	local fs,data = object_ros:read()
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

		object_pose:write(pose_holder)
        end

end

function cleanupHook()
	rttlib.tc_cleanup()
end
