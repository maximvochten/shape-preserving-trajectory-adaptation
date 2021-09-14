require("rttlib")
require("math")

tc = rtt.getTC()

------------ Ports -------------
-- Input ports
bspline_vel = rtt.InputPort("double", "bspline_vel", "Gives desired velocity of progress along bspline trajectory")
tc:addPort(bspline_vel)

end_of_traj = rtt.InputPort("double", "end_of_traj", "Signals end-of-trajectory to Orocos component")
tc:addPort(end_of_traj)

etasl_progress = rtt.InputPort("array", "etasl_progress", "Local progress from eTaSL, which is given as an array")
tc:addPort(etasl_progress)

-- Output ports
weight = rtt.OutputPort("double", "weight", "Cycloidal weighting between feedforward (trajectory) and feedback at end-of-trajectory")
tc:addPort(weight)

deriv_weight = rtt.OutputPort("double", "deriv_weight", "derivative of weighting function")
tc:addPort(deriv_weight)

progress_rate = rtt.OutputPort("double", "progress_rate", "test")
tc:addPort(progress_rate)

ros_progress = rtt.OutputPort("double", "ros_progress", "Transformed etasl_progress datatype to Float64 in order to publish it to a ROS topic")
tc:addPort(ros_progress)

---------- Properties ----------
desired_velocity = rtt.Variable("double")
eot = rtt.Variable("bool")

starttime_eot = rtt.Variable("double")
endtime_eot = rtt.Variable("double")
curtime_eot = rtt.Variable("double")

-------- Hook functions --------
function configureHook()
	eot = false
	return true
end

function startHook()
        return true
end

function updateHook()
	local fs, s_data = etasl_progress:read()
	local fs4, bspline_vel_data = bspline_vel:read()
	local fs5, end_of_traj_data = end_of_traj:read()
	local eot_curr = 0.0
	
	if (fs ~= 'NoData') then
		local s_tab = s_data:totab()
		ros_progress:write(s_tab[1])
	end

	if (fs4 ~= 'NoData') then
		desired_velocity = bspline_vel_data
	end
	progress_rate:write(desired_velocity)

	if (fs5 == 'NewData') then
                eot = true
        	starttime_eot = 0.0
		endtime_eot = 1.0
		curtime_eot = 0.0
	end


	if (eot == true) then
		if curtime_eot < endtime_eot then
			local dt = tc:getPeriod()
			local update_curtime = curtime_eot + dt
			local tau = (update_curtime - starttime_eot)/(endtime_eot - starttime_eot)
			local eot_weight = tau - (1/(2*math.pi))*math.sin(2*math.pi*tau)
			local deriv_eot_weight = (1 - math.cos(2*math.pi*tau))*(1/(endtime_eot - starttime_eot))*dt

			weight:write(eot_weight)
			deriv_weight:write(deriv_eot_weight)
			curtime_eot = update_curtime
		else
			weight:write(1.0)
			deriv_weight:write(0.0)
		end
	else
		weight:write(0.0)
		deriv_weight:write(0.0)
	end

end

function cleanupHook()
        rttlib.tc_cleanup()
end
