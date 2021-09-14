-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Main file to define default robot and ports
-- KU Leuven 2020
-- ==============================================================================
require("context")
require("geometric")
-- robot = require("etasl_UR10")
-- robot = require("etasl_kinova_gen3")
-- robot.etasl_specification()
-- ==================== Input Ports =====================================

Fx_raw = ctx:createInputChannelScalar("Fx")
Fy_raw = ctx:createInputChannelScalar("Fy")
Fz_raw = ctx:createInputChannelScalar("Fz")
Tx_raw = ctx:createInputChannelScalar("Tx")
Ty_raw = ctx:createInputChannelScalar("Ty")
Tz_raw = ctx:createInputChannelScalar("Tz")

W_Fx = ctx:createInputChannelScalar("W_Fx")
W_Fy = ctx:createInputChannelScalar("W_Fy")
W_Fz = ctx:createInputChannelScalar("W_Fz")
W_Tx = ctx:createInputChannelScalar("W_Tx")
W_Ty = ctx:createInputChannelScalar("W_Ty")
W_Tz = ctx:createInputChannelScalar("W_Tz")

------- Bsplines ----------
cp_x = {}
cp_y = {}
cp_z = {}
cp_qx = {}
cp_qy = {}
cp_qz = {}
cp_qw = {}
cp_vp = {}
for i=1,7 do
	cp_x[i] = ctx:createInputChannelScalar("cp_x_"..i)
	cp_y[i] = ctx:createInputChannelScalar("cp_y_"..i)
	cp_z[i] = ctx:createInputChannelScalar("cp_z_"..i)

	cp_qx[i] = ctx:createInputChannelScalar("cp_qx_"..i)
        cp_qy[i] = ctx:createInputChannelScalar("cp_qy_"..i)
        cp_qz[i] = ctx:createInputChannelScalar("cp_qz_"..i)
        cp_qw[i] = ctx:createInputChannelScalar("cp_qw_"..i)
	cp_vp[i] = ctx:createInputChannelScalar("cp_vp_"..i)
end

eot_weight = ctx:createInputChannelScalar("eot_weight")
tracker_frame = ctx:createInputChannelFrame("tracker_frame")

s_offset = ctx:createInputChannelScalar("s_offset")
progress_rate = ctx:createInputChannelScalar("progress_rate")
test_flag = ctx:createInputChannelScalar("test_flag")

obstacle_pose = ctx:createInputChannelFrame("obstacle_pose")

max_joint_vels ={}
min_joint_vels ={}
current_joint_vels ={}
for i=1,#robot_joints do
  max_joint_vels[i] = ctx:createInputChannelScalar("maxvel_j"..i)
  min_joint_vels[i] = ctx:createInputChannelScalar("minvel_j"..i)
  current_joint_vels[i] = ctx:createInputChannelScalar("curr_vel_j"..i)
end

-- ==================== Output ports ==================================

ctx:setOutputExpression("FT_frame",FT_frame)

ctx:setOutputExpression("pose_robot",task_frame)

ctx:setOutputExpression("start_trajectory",task_frame)

ctx:setOutputExpression("distance", constant(0))
--ctx:setOutputExpression("Fobstacle",task_frame)

ctx:setOutputExpression("x_tf",constant(0))
ctx:setOutputExpression("y_tf",constant(0))
ctx:setOutputExpression("z_tf",constant(0))

ctx:setOutputExpression("roll_tf",constant(0))
ctx:setOutputExpression("pitch_tf",constant(0))
ctx:setOutputExpression("yaw_tf",constant(0))

ctx:setOutputExpression("Fx",constant(0))
ctx:setOutputExpression("Fy",constant(0))
ctx:setOutputExpression("Fz",constant(0))

ctx:setOutputExpression("Tx",constant(0))
ctx:setOutputExpression("Ty",constant(0))
ctx:setOutputExpression("Tz",constant(0))

ctx:setOutputExpression("s",constant(0))

ctx:setOutputExpression("task_error",constant(0))

ctx:setOutputExpression("s_g",constant(0))
