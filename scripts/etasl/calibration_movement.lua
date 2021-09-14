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
rotation_symmetry = false

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
endpose = tracker_frame
endpos = origin(endpose)
endrot = rotation(endpose)

-- =========================== VELOCITY PROFILE ============================================

-- compute distances for displacements and rotations:
diff                    = cached(endpos-startpos)
diff, distance          = utils_ts.normalize( diff )

diff_rot                = cached(  getRotVec( inv(startrot)*endrot )) -- eq. axis of rotation for rotation from start to end:w
diff_rot, angle         = utils_ts.normalize( diff_rot )


-- plan trapezoidal motion profile in function of time:
mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
mp:addOutput(constant(0), distance, maxvel, maxacc)
mp:addOutput(constant(0), angle*eqradius, maxvel, maxacc)
d  = get_output_profile(mp,0)            -- progression in distance
r  = get_output_profile(mp,1)/eqradius   -- progression in distance_rot (i.e. rot*eqradius)

-- =========================== TARGET POSE ============================================

targetpos = startpos + diff*d
targetrot = startrot*rotVec(diff_rot,r)

target    = frame(targetrot,targetpos)

-- Orientational freedom
R_ee_w = rotation(tf)
R_tra_w = endrot
R_ee_tra = inv(R_tra_w)*R_ee_w

R_tgt_w = targetrot
R_tgt_ee = inv(R_ee_w)*R_tgt_w

rot_ee = getRotVec(R_tgt_ee)
rot_tra = R_ee_tra*rot_ee

-- ========================== CONSTRAINT SPECIFICATION =================================
if rotation_symmetry == false then
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*task_frame,
    K       = 1,
    weight  = 1,
    priority= 2
}
else
Constraint {
        context = ctx,
        name = "follow_position",
        expr = targetpos - origin(tf),
        K = 1,
        priority = 2,
        weight = 1
}

Constraint {
        context = ctx,
        name = "rotation_constraint_x",
        expr = coord_x(rot_tra),
        K = 1,
        priority = 2,
        weight = 1
}

Constraint {
        context = ctx,
        name = "rotation_constraint_y",
        expr = coord_y(rot_tra),
        K = 1,
        priority = 2,
        weight = 1
}

--Constraint {
--        context = ctx,
--        name = "rotation_constraint_z",
--        expr = coord_z(rot_tra),
--        K = 1,
--        priority = 2,
--        weight = 1
--}
end

-- ============================== OUTPUT THROUGH PORTS===================================
ctx:setOutputExpression("pose_robot",tf)

ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
ctx:setOutputExpression("z_tf",coord_z(origin(tf)))

roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)
