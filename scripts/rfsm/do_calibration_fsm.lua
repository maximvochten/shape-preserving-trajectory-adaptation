-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Definition of the state machine
-- KU Leuven 2020
--
-- ==============================================================================
-- Edited for task: "Shape-preserving and reactive adaptation of robot end 
-- 	effector trajectories"
-- Contributors: Glenn Maes, Maxim Vochten
-- ==============================================================================

require("rtt")
require("rttlib")
require("rfsm_timeevent")

gettime = rtt.getTime
rfsm_timeevent.set_gettime_hook(gettime)


tc          = rtt.getTC()
depl        = tc:getPeer("Deployer")
etaslcore   = depl:getPeer("etaslcore")
reporter    = depl:getPeer("Reporter")
solver      = depl:getPeer("solver")

simulation = tc:getProperty("simulation"):get()
robot_etasl_dir = tc:getProperty("robot_etasl_dir"):get()
depl_robot_file = tc:getProperty("depl_robot_file"):get()
robot = require(depl_robot_file)
joint_pos = robot.home_joint_positions()

function driver_particularities()
  if robot.robot_name == "franka_panda" and not simulation then
    --local panda = depl:getPeer("panda")
    --panda:low_level_velocity()
  end

end

-- Desired jointstates for calibration (based upon Franka setup)
target_pos1 = {0.00, 0.00, 0.00, -1.57, 0.00, 1.81, 0.00, 0.00}
target_pos2 = {-0.85, -0.19, -0.25, -1.76, -0.67, 2.11, 0.03, 0.00}
target_pos3 = {-2.11, -0.51, 0.29, -2.57, 2.47, 1.64, -0.96, 0.00}
target_pos4 = {-1.23, -0.35, 0.12, -1.96, 0.91, 2.00, 0.03, 0.00}
target_pos5 = {-0.40, 0.71, -0.75, -2.11, -1.90, 1.93, 0.24, 0.00}


etasl_application_dir = rtt.provides("ros"):find("application_etasl_invariants")
return rfsm.state {
   configured = rfsm.state {
      entry=function()
        -- ================================Configuration for Kuka iiwa: The driver of the iiwa needs to send zero velocities and wait a bit to configure ============
            if robot.robot_name == "kuka_lwr" and not simulation then
              local lwr = depl:getPeer("lwr")
              local port_vel = rttlib.port_clone_conn(lwr:getPort("JointVelocityCommand"))
              depl:import("rtt_motion_control_msgs")
              local vel_values = rtt.Variable("motion_control_msgs.JointVelocities")
              vel_values.names = robot.joint_names()
              vel_values.velocities:fromtab({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
              -- print(vel_values)
              port_vel:write(vel_values)
              rtt.sleep(2,0) --Sleep for 2 seconds
            end
        -- ================================ General configuration ============================

            reporter:configure()
            reporter:start()
          end,
   },

   idle = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	    etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()

      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   idle2 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()

      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   idle3 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()

      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   idle4 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()

      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   idle5 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()

      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   moving_joint_space_1 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_jointspace_trap.lua")
            etaslcore:set_etaslvar("global.maxvel",0.4)
            etaslcore:set_etaslvar("global.maxacc",0.2)
            if robot.is_continuous_joints then --If the function exists it modifies it. Otherwise the defaults are used (non continuous)
                  for i=1,#joint_pos do
                        etaslcore:set_etaslvar("global.continuous_j"..i,robot.is_continuous_joints()[i])
                  end
            end
            for i=1,#joint_pos do
                  etaslcore:set_etaslvar("global.end_j"..i,target_pos1[i])
            end
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()
      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   move_joint_space_2 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_jointspace_trap.lua")
            etaslcore:set_etaslvar("global.maxvel",0.4)
            etaslcore:set_etaslvar("global.maxacc",0.2)
            if robot.is_continuous_joints then --If the function exists it modifies it. Otherwise the defaults are used (non continuous)
                  for i=1,#joint_pos do
                        etaslcore:set_etaslvar("global.continuous_j"..i,robot.is_continuous_joints()[i])
                  end
            end
            for i=1,#joint_pos do
                  etaslcore:set_etaslvar("global.end_j"..i,target_pos2[i])
            end
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()
      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   moving_joint_space_3 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_jointspace_trap.lua")
            etaslcore:set_etaslvar("global.maxvel",0.4)
            etaslcore:set_etaslvar("global.maxacc",0.2)
            if robot.is_continuous_joints then --If the function exists it modifies it. Otherwise the defaults are used (non continuous)
                  for i=1,#joint_pos do
                        etaslcore:set_etaslvar("global.continuous_j"..i,robot.is_continuous_joints()[i])
                  end
            end
            for i=1,#joint_pos do
                  etaslcore:set_etaslvar("global.end_j"..i,target_pos3[i])
            end
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()
      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   moving_joint_space_4 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_jointspace_trap.lua")
            etaslcore:set_etaslvar("global.maxvel",0.4)
            etaslcore:set_etaslvar("global.maxacc",0.2)
            if robot.is_continuous_joints then --If the function exists it modifies it. Otherwise the defaults are used (non continuous)
                  for i=1,#joint_pos do
                        etaslcore:set_etaslvar("global.continuous_j"..i,robot.is_continuous_joints()[i])
                  end
            end
            for i=1,#joint_pos do
                  etaslcore:set_etaslvar("global.end_j"..i,target_pos4[i])
            end
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()
      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   moving_joint_space_5 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/move_jointspace_trap.lua")
            etaslcore:set_etaslvar("global.maxvel",0.4)
            etaslcore:set_etaslvar("global.maxacc",0.2)
            if robot.is_continuous_joints then --If the function exists it modifies it. Otherwise the defaults are used (non continuous)
                  for i=1,#joint_pos do
                        etaslcore:set_etaslvar("global.continuous_j"..i,robot.is_continuous_joints()[i])
                  end
            end
            for i=1,#joint_pos do
                  etaslcore:set_etaslvar("global.end_j"..i,target_pos5[i])
            end
            etaslcore:configure()
            etaslcore:initialize()
            etaslcore:start()
            driver_particularities()
      end,
      exit=function()
            etaslcore:stop()
            etaslcore:cleanup()
      end,
   },

   rfsm.trans {src="initial", tgt="configured" },
-- ============================== RUN calibration sequence ===================================
rfsm.trans {src = "configured",	 tgt="idle",			events={}},
rfsm.trans {src="idle",   tgt="moving_joint_space_1",	events={'e_after(5)'}},
rfsm.trans {src="moving_joint_space_1",	tgt="idle2",	events={"e_finished@etaslcore"}},
rfsm.trans {src="idle2",   tgt="move_joint_space_2",   events={'e_after(5)'}},
rfsm.trans {src="move_joint_space_2", tgt="idle3",     events={"e_finished@etaslcore"}},
rfsm.trans {src="idle3",   tgt="moving_joint_space_3",   events={'e_after(5)'}},
rfsm.trans {src="moving_joint_space_3", tgt="idle4",     events={"e_finished@etaslcore"}},
rfsm.trans {src="idle4",   tgt="moving_joint_space_4",   events={'e_after(5)'}},
rfsm.trans {src="moving_joint_space_4", tgt="idle5",     events={"e_finished@etaslcore"}},
rfsm.trans {src="idle5",   tgt="moving_joint_space_5",   events={'e_after(5)'}},
rfsm.trans {src="moving_joint_space_5", tgt="idle",     events={"e_finished@etaslcore"}},
}
