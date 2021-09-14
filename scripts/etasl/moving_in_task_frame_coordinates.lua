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

require "math"
-- ========================================= PARAMETERS ===================================
maxvel    = ctx:createInputChannelScalar("maxvel" ,0.1)
maxacc    = ctx:createInputChannelScalar("maxacc" ,0.1)
eqradius  = ctx:createInputChannelScalar("eq_r"   ,0.08)
delta_x   = ctx:createInputChannelScalar("delta_x",0.3)
delta_y   = ctx:createInputChannelScalar("delta_y",0.0)
delta_z   = ctx:createInputChannelScalar("delta_z",0.2)

-- ======================================== FRAMES ========================================

tf = task_frame

-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, tf)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- =============================== END POSE ==============================
-- Practical values:

-- KUKA LWR
--endpos = vector(0.345-0.9, 1.648-1.4, 1.599-0.8)

-- Franka Panda
endpos = vector(0.35, -0.4, 0.5)

---==== Quaternion test ====---
qx = 0.6533
qy = 0.2706
qz = 0.6533
qw = -0.2706

--qx = 1
--qy = 0
--qz = 0
--qw = 0

q_norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw + constant(1E-8))
qxn = qx/q_norm
qyn = qy/q_norm
qzn = qz/q_norm
qwn = qw/q_norm

R11 = 1-2*(qyn*qyn + qzn*qzn)
R12 = 2*(qxn*qyn - qwn*qzn)
R13 = 2*(qxn*qzn + qwn*qyn)
R21 = 2*(qxn*qyn + qwn*qzn)
R22 = 1-2*(qxn*qxn + qzn*qzn)
R23 = 2*(qyn*qzn - qwn*qxn)
R31 = 2*(qxn*qzn - qwn*qyn)
R32 = 2*(qyn*qzn + qwn*qxn)
R33 = 1-2*(qxn*qxn + qyn*qyn)

R = construct_rotation_from_vectors(vector(R11,R21,R31), vector(R12,R22,R32), vector(R13,R23,R33))

endrot = R
endpose = frame(endrot,endpos)
start_trajectory = endpose
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

-- ========================== CONSTRAINT SPECIFICATION =================================
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*tf,
    K       = 3,
    weight  = 1,
    priority= 2
}

-- =========================== MONITOR ============================================
Monitor{
        context=ctx,
        name='finish_after_motion',
        upper=0.0,
        actionname='exit',
        expr=time-get_duration(mp) - constant(0.1)
}


-- ============================== OUTPUT THROUGH PORTS===================================
ctx:setOutputExpression("start_trajectory",endpose)

ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
ctx:setOutputExpression("z_tf",coord_z(origin(tf)))

roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)
