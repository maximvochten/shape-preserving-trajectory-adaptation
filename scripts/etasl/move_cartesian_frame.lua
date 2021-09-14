 -- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Main file to move linearly along a frame axis with trapezoidal velocity
-- profile
-- KU Leuven 2020
-- ==============================================================================

require("context")
require("geometric")
utils_ts = require("utils_ts")

-- ========================================= PARAMETERS ===================================
maxvel    = ctx:createInputChannelScalar("maxvel" ,0.1)
maxacc    = ctx:createInputChannelScalar("maxacc" ,0.1)
eqradius  = ctx:createInputChannelScalar("eq_r"   ,0.08)
delta_x   = ctx:createInputChannelScalar("delta_x",0.0)
delta_y   = ctx:createInputChannelScalar("delta_y",0.0)
delta_z   = ctx:createInputChannelScalar("delta_z",0.0)

-- ======================================== FRAMES ========================================
tf = task_frame

-- =============================== INITIAL POSE ==============================
startpose = initial_value(time, tf)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- =============================== END POSE ==============================
endpose = traj_frame
endpos = origin(endpose)
endrot = rotation(endpose)

-- ========================== CONSTRAINT SPECIFICATION =================================
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(endpose)*task_frame,
    K       = 1,
    weight  = 1,
    priority= 2
}

task_error = norm( vector((coord_x(origin(tf)) - coord_x(origin(endpose))), (coord_y(origin(tf)) - coord_y(origin(endpose))), (coord_z(origin(tf)) - coord_z(origin(endpose)))) )

-- ============================== OUTPUT THROUGH PORTS===================================
ctx:setOutputExpression("pose_robot",tf)

ctx:setOutputExpression("start_trajectory",startpose)

ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
ctx:setOutputExpression("z_tf",coord_z(origin(tf)))

roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)

ctx:setOutputExpression("task_error",task_error)
