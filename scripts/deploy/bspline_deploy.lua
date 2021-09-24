-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- KU Leuven 2020
--
-- Adapted for the 2020-2021 Trajectory tracking in eTaSL thesis
--	This script deploys the B-spline parameterized trajectory following
--	and can be seen as the main script of the thesis
-- ==============================================================================

require "rttlib"
require "rttros"
require "deployer_utils"

-- ====================================== User Parameters =========================================

--robot_name: Choose the robot model: "ur_10" or "kinova_gen3" are currently supported
-- use_jr3: Use it if you want to integrate a jr3 wrench sensor
-- freq: Frequency [Hz] at which the eTaSL and the corresponding components run.

-- robot_name = "ur_10"
-- robot_name = "kinova_gen3"
robot_name = "franka_panda"
-- robot_name = "kuka_iiwa"
-- robot_name = "kuka_lwr"
use_jr3 = false
freq = 200

-- ====================================== Standard deployment stuff =========================================
rtt.setLogLevel("Warning")

gs=rtt.provides()
tc=rtt.getTC()
if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
  require("kdlutils")
elseif tc:getName() == "Deployer" then
  depl=tc
end
depl:import("rtt_ros")
depl:import("rtt_rosnode")
depl:import("rtt_roscomm")
depl:import("rtt_visualization_msgs")
ros = gs:provides("ros")
ros:import("etasl_rtt")
ros:import("rtt_rospack")
rttlib.color = true

--depl:import("viz_marker")
depl:import("rtt_etasl_invariants_integration")
depl:import("rtt_ros_spline_fitting_trajectory")

etasl_application_dir = rtt.provides("ros"):find("etasl_invariants_integration")
robot_def_dir = etasl_application_dir .. "/scripts/etasl/robot_def"

-- The following will make run always in simulation, unless you provide "deploy_robot as the first argument"
-- Example to run in real robot: rttlua -i deploy_general.lua "real_robot"
if arg[1] == "real_robot" then
  simulation = false
else
  simulation = true
end


cp=rtt.Variable("ConnPolicy")

-- ====================================== Robot Hardware definition =========================================
depl_robot_file,robot_etasl_dir = determine_robot(robot_name)
robot = require(depl_robot_file)

-- ====================================== eTaSL components ===================================
-- ====================================== Solver
ros:import("etasl_solver_qpoases")
depl:loadComponent("solver","etasl_solver_qpoases")
solver = depl:getPeer("solver")

-- ====================================== jointstate I/O factories
ros:import("etasl_iohandler_jointstate")
depl:loadComponent("jointstate","Etasl_IOHandler_Jointstate")
jointstate = depl:getPeer("jointstate")

-- ====================================== eTaSL core
ros:import("etasl_rtt")
depl:loadComponent("etaslcore", "etasl_rtt")
-- create LuaComponents
etaslcore = depl:getPeer("etaslcore")
depl:connectPeers("etaslcore","solver")
depl:connectPeers("etaslcore","jointstate")

-- ====================================== Input ports task
etaslcore:add_etaslvar_inputport("cp_x", "Control points of x-component of position", s{"cp_x_1", "cp_x_2", "cp_x_3", "cp_x_4", "cp_x_5", "cp_x_6", "cp_x_7"}, d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
etaslcore:add_etaslvar_inputport("cp_y", "Control points of y-component of position", s{"cp_y_1", "cp_y_2", "cp_y_3", "cp_y_4", "cp_y_5", "cp_y_6", "cp_y_7"}, d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
etaslcore:add_etaslvar_inputport("cp_z", "Control points of z-component of position", s{"cp_z_1", "cp_z_2", "cp_z_3", "cp_z_4", "cp_z_5", "cp_z_6", "cp_z_7"}, d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
etaslcore:add_etaslvar_inputport("quat_cpx", "test", s{"cp_qx_1", "cp_qx_2", "cp_qx_3", "cp_qx_4", "cp_qx_5", "cp_qx_6", "cp_qx_7"}, d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
etaslcore:add_etaslvar_inputport("quat_cpy", "test", s{"cp_qy_1", "cp_qy_2", "cp_qy_3", "cp_qy_4", "cp_qy_5", "cp_qy_6", "cp_qy_7"}, d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
etaslcore:add_etaslvar_inputport("quat_cpz", "test", s{"cp_qz_1", "cp_qz_2", "cp_qz_3", "cp_qz_4", "cp_qz_5", "cp_qz_6", "cp_qz_7"}, d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
etaslcore:add_etaslvar_inputport("quat_cpw", "test", s{"cp_qw_1", "cp_qw_2", "cp_qw_3", "cp_qw_4", "cp_qw_5", "cp_qw_6", "cp_qw_7"}, d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})

etaslcore:add_etaslvar_inputport("cp_velprof", "test", s{"cp_vp_1", "cp_vp_2", "cp_vp_3", "cp_vp_4", "cp_vp_5", "cp_vp_6", "cp_vp_7"}, d{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})

etaslcore:add_etaslvar_inputport("eot_weight", "test", s{"eot_weight"}, rtt.Variable("array"))
etaslcore:add_etaslvar_deriv_inputport("deriv_eot_weight", "test", s{"eot_weight"})

etaslcore:add_etaslvar_inputport("progress_rate", "test", s{"progress_rate"}, d{0.0})
etaslcore:add_etaslvar_inputport("s_offset", "test", s{"s_offset"}, d{0.0})
etaslcore:add_etaslvar_inputport("test_flag", "test", s{"test_flag"}, d{0.0})

etaslcore:add_etaslvar_frame_inputport("tracker_frame", "test", "tracker_frame", rtt.Variable("KDL.Frame"))
etaslcore:add_etaslvar_deriv_twist_inputport("tracker_twist", "Target twist of tracker", "tracker_frame")

-- Reactive behaviors
etaslcore:add_etaslvar_frame_inputport("obstacle_pose", "Pose of obstacle used for obstacle avoidance", "obstacle_pose", rtt.Variable("KDL.Frame"))
etaslcore:add_etaslvar_inputport("wrench_in", "wrench ef", s{"global.Fx","global.Fy","global.Fz","global.Tx","global.Ty","global.Tz"}, d{})

-- ====================================== Output ports task
etaslcore:add_etaslvar_outputport("tf_pose","Executed pose of the task frame",s{"x_tf","y_tf","z_tf","roll_tf","pitch_tf","yaw_tf"})
etaslcore:add_etaslvar_outputport("path_coordinate","Information of the path coordinate",s{"s"})
etaslcore:add_etaslvar_frame_outputport("pose_robot", "Task frame pose??","pose_robot")
etaslcore:add_etaslvar_frame_outputport("start_trajectory", "Start of invariants trajectory", "start_trajectory")

etaslcore:add_etaslvar_outputport("task_error", "Error on desired task", s{"task_error"})

etaslcore:add_etaslvar_outputport("s_g", "test", s{"s_g"})
etaslcore:add_etaslvar_outputport("obst_dist", "test", s{"distance"})
-- Simulation of obstacle
--etaslcore:add_etaslvar_frame_outputport("obstacle_frame","gives pose of simulated obstacle","Fobstacle")

-- ====================================== Configure eTaSL ports for the robot
robot.create_etasl_ports(etaslcore,jointstate)

depl:setActivity("etaslcore", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

-- ====================================== Bsplines
depl:loadComponent("bsplines", "OCL::LuaComponent")
bsplines = depl:getPeer("bsplines")
bsplines:exec_file(etasl_application_dir.."/scripts/components/bspline_invariants_component.lua")
depl:setActivity("bsplines", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

depl:connect("etaslcore.pose_robot", "bsplines.pose_robot", cp)

depl:connect("bsplines.s_offset", "etaslcore.s_offset", cp)
depl:connect("etaslcore.s_g", "bsplines.s_g", cp)
depl:connect("bsplines.test_flag", "etaslcore.test_flag", cp)

depl:connect("bsplines.cp_x", "etaslcore.cp_x", cp)
depl:connect("bsplines.cp_y", "etaslcore.cp_y", cp)
depl:connect("bsplines.cp_z", "etaslcore.cp_z", cp)
depl:connect("bsplines.cp_qx", "etaslcore.quat_cpx", cp)
depl:connect("bsplines.cp_qy", "etaslcore.quat_cpy", cp)
depl:connect("bsplines.cp_qz", "etaslcore.quat_cpz", cp)
depl:connect("bsplines.cp_qw", "etaslcore.quat_cpw", cp)

depl:connect("bsplines.cp_velprof", "etaslcore.cp_velprof", cp)

depl:connect("etaslcore.start_trajectory", "bsplines.start_trajectory", cp)

depl:stream("bsplines.start_traj", ros:topic("start_traj_pub"))

depl:stream("bsplines.bspline_parameters", ros:topic("/bspline_pub"))
depl:stream("bsplines.velprof_controlpoints", ros:topic("/velprof_pub"))
depl:stream("bsplines.robot_pose_msg", ros:topic("current_pose_pub"))

bsplines:configure()
bsplines:start()

depl:loadComponent("progress", "OCL::LuaComponent")
progress = depl:getPeer("progress")
progress:exec_file(etasl_application_dir.."/scripts/components/progress_component.lua")
depl:setActivity("progress", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

depl:connect("etaslcore.path_coordinate", "progress.etasl_progress", cp)

depl:connect("progress.weight", "etaslcore.eot_weight", cp)
depl:connect("progress.deriv_weight", "etaslcore.deriv_eot_weight", cp)

depl:connect("progress.progress_rate", "etaslcore.progress_rate", cp)

depl:stream("progress.bspline_vel", ros:topic("/bspline_vel_pub"))
depl:stream("progress.end_of_traj", ros:topic("/eot_pub"))
depl:stream("progress.ros_progress", ros:topic("/progress_partial"))
progress:configure()
progress:start()


depl:loadComponent("feedback_eot", "OCL::LuaComponent")
feedback_eot = depl:getPeer("feedback_eot")
feedback_eot:exec_file(etasl_application_dir.."/scripts/components/eot_component.lua")
depl:setActivity("feedback_eot", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

depl:stream("feedback_eot.tracker_ros", ros:topic("/target_pose_pub"))
depl:connect("feedback_eot.tracker_endpose", "etaslcore.tracker_frame", cp)

depl:stream("feedback_eot.twist_ros", ros:topic("/target_vel_pub"))
depl:connect("feedback_eot.tracker_twist", "etaslcore.tracker_twist", cp)
feedback_eot:configure()
feedback_eot:start()

depl:loadComponent("object_avoidance", "OCL::LuaComponent")
object_avoidance = depl:getPeer("object_avoidance")
object_avoidance:exec_file(etasl_application_dir.."/scripts/components/object_component.lua")
depl:setActivity("object_avoidance", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

depl:connect("object_avoidance.object_pose", "etaslcore.obstacle_pose", cp)
depl:stream("object_avoidance.object_ros", ros:topic("/object_pose_pub"))

object_avoidance:configure()
object_avoidance:start()

-- ====================================== simulation
if simulation then
-- deploy simulated robot:
    depl:loadComponent("simrobot", "OCL::LuaComponent")
    simrobot = depl:getPeer("simrobot")
    simrobot:exec_file(etasl_application_dir.."/scripts/components/simple_robot_sim.lua")
    init_jnts = robot.initial_joints_states()
    simrobot:getProperty("initial_position"):set( init_jnts )
    depl:setActivity("simrobot", 1/freq, 50, rtt.globals.ORO_SCHED_RT)
    depl:connect("etaslcore.jointvel","simrobot.jointvel",cp )
    depl:connect("simrobot.jointpos","etaslcore.jointpos",cp )
    simrobot:configure()
    simrobot:start()
    -- Simulation of obstacle
   -- depl:loadComponent("viz","Viz_Marker")
   -- viz = depl:getPeer("viz")
   -- viz:getProperty("nrofmarkers"):set(1)
   -- viz:getProperty("frameid"):set("base_link")
   -- viz:getProperty("type"):set(3)
   -- viz:getProperty("scalex"):set(0.2)
   -- viz:getProperty("scaley"):set(0.2)
   -- viz:getProperty("scalez"):set(0.5)
   -- viz:getProperty("red"):set(1)
   -- viz:getProperty("green"):set(0)
   -- viz:getProperty("blue"):set(0)
   -- viz:getProperty("alpha"):set(1)
   -- viz:getProperty("lifetime"):set(-1)
   -- viz:configure()
   -- depl:setActivity("viz", 0.03, 50, rtt.globals.ORO_SCHED_RT)
   -- depl:stream("viz.output", ros:topic("/marker_visualizations"))
   -- viz:start()
   -- depl:connect("etaslcore.obstacle_frame","viz.input1",cp)

else
-- ====================================== Real robot
    robot.deploy_driver(1/freq)

-- ====================================== Wrench processing component
    ros:import("wrench_processing")
    depl:loadComponent("wrench_processing_franka","WrenchSensor")
    wrench_processing_franka = depl:getPeer("wrench_processing_franka")

    depl:connect("panda.tool_external_wrench","wrench_processing_franka.wrench",cp)
    wrench_processing_franka:getProperty('cuttoff_freq'):set(5)
    wrench_processing_franka:getProperty('sample_rate'):set(1000)

    wrench_processing_franka:getProperty('force_threshold'):set(0)
    wrench_processing_franka:getProperty('torque_threshold'):set(0)

    wrench_processing_franka:configure()
    wrench_processing_franka:start()
    depl:connect("wrench_processing_franka.wrench_array", "etaslcore.wrench_in",cp)


end

-- ====================================== Reporter =========================================
function exists(file)
  local ok, err, code = os.rename(file, file)
  if not ok then
if code == 13 then
  -- Permission denied, but it exists
  return true
end
  end
  return ok
end

function isdir(path)
  -- "/" works on both Unix and Windows
  return exists(path.."/")
end

date = os.date("%Y_%m_%d")
tmstamp = os.date("%Y_%m_%d_%H_%M_%S")
dir_name = "/reports_from_" .. date
dir_path = etasl_application_dir .. "/reports/all_data" .. dir_name

file_name = "/report_of_" .. tmstamp ..'.dat'

if not isdir(dir_path) then
  os.execute("mkdir -p " .. dir_path )
  print('Directory ' .. dir_name .. ' created')
end

depl:loadComponent("Reporter","OCL::FileReporting")
reporter=depl:getPeer("Reporter")
depl:connectPeers("etaslcore","Reporter")
reporter:reportPort("etaslcore","jointvel")
reporter:reportPort("etaslcore","tf_pose")
reporter:reportPort("etaslcore","path_coordinate")
reporter:reportPort("etaslcore","task_error")
reporter:reportPort("etaslcore","obst_dist")
reporter:getProperty("ReportFile"):set(dir_path .. file_name)

-- ====================================== Force/Torque Sensor =========================================
if not simulation and use_jr3 then
    FTsensor = require("etasl_FT_JR3")
    FTsensor.FTsensor_deployer(etaslcore,1/freq)
end
-- ====================================== Supervisor =========================================
depl:loadComponent("Supervisor", "OCL::LuaComponent")
sup = depl:getPeer("Supervisor")

define_property( sup, "simulation", "bool", simulation, "Boolean value to set simulation mode" )
define_property( sup, "robot_etasl_dir", "string", robot_etasl_dir, "Directory of the etasl robot definition" )
define_property( sup, "depl_robot_file", "string", depl_robot_file, "Directory of the file containing deployment of the robot" )

sup:exec_file(etasl_application_dir.."/scripts/components/fsm_component.lua")
sup:getProperty("state_machine"):set(etasl_application_dir.."/scripts/rfsm/bspline_fsm.lua")
sup:addPeer(depl)
depl:setActivity("Supervisor", 1/freq, 50, rtt.globals.ORO_SCHED_RT) --needed to use the time events of rfsm
sup:configure()
sup:start()
cmd = rttlib.port_clone_conn(sup:getPort("events"))

-- connect ports:
if not simulation then
  robot.connect_ports_driver(etaslcore,1/freq)
end
depl:connect("etaslcore.eventPort","Supervisor.events",cp)
depl:stream("etaslcore.joint_state", ros:topic("/joint_states"))
