-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- KU Leuven 2020
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

etasl_application_dir = rtt.provides("ros"):find("application_etasl_invariants")
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
-- robot = require("etasl_UR10")
-- robot = require("etasl_kinova_gen3")
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
etaslcore:add_etaslvar_frame_inputport("tracker_frame", "Target frame of tracker", "tracker_frame", rtt.Variable("KDL.Frame"))
-- ====================================== Output ports task
etaslcore:add_etaslvar_outputport("tf_pose","Executed pose of the task frame",s{"x_tf","y_tf","z_tf","roll_tf","pitch_tf","yaw_tf"})
etaslcore:add_etaslvar_outputport("path_coordinate","Information of the path coordinate",s{"s"})
etaslcore:add_etaslvar_frame_outputport("pose_robot", "Task frame pose??","pose_robot")

-- ====================================== Configure eTaSL ports for the robot
robot.create_etasl_ports(etaslcore,jointstate)

depl:setActivity("etaslcore", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

-- ====================================== Calibration
depl:loadComponent("calibration", "OCL::LuaComponent")
calibration = depl:getPeer("calibration")
calibration:exec_file(etasl_application_dir.."/scripts/components/calibration_component.lua")
depl:setActivity("calibration", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

depl:connect("etaslcore.pose_robot", "calibration.pose_robot", cp)
depl:connect("calibration.tracker_endpose", "etaslcore.tracker_frame", cp)
depl:stream("calibration.tracker_ros", ros:topic("/target_pose_pub"))
calibration:configure()
calibration:start()

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

else
-- ====================================== Real robot
    robot.deploy_driver(1/freq)
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
sup:getProperty("state_machine"):set(etasl_application_dir.."/scripts/rfsm/calibration_fsm.lua")
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
