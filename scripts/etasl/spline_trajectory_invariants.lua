require("context")
require("geometric")
bs = require("bspline")
utils_ts = require("utils_ts")

-- ======== Setting up behavior + pre-processing ======== --
-- Desired behaviors
rotation_symmetry = false

feedback_eot = false

-- Frames
tf = task_frame

-- Initial pose
startpose = initial_value(time, tf)
startpos = origin(startpose)
startrot = rotation(startpose)

-- ======== Bsplines ======== --
-- Degree of advancement (definition of progress variables)
s_l = Variable{context = ctx, name = 'local_progress', vartype = 'feature', initial = 0.0}
s_g = Variable{context = ctx, name = 'global_progress', vartype = 'feature', initial = 0.0}

s_l = s_g - s_offset

prog_rate = conditional(constant(0.5)-test_flag, constant(0), progress_rate)
Constraint{
      context = ctx,
      expr = s_g - time*prog_rate,
      K = 0,
      priority = 2,
      weight = 0.05
}

-- Bspline parameters
degree = 3

knots = bs.linspace(0,1,5)
knots = bs.augknots(knots, degree)

nr_of_cp = #knots - degree + 1

-- Position
spl_x = bs.generate_spline(s_l,knots,cp_x,degree)
spl_y = bs.generate_spline(s_l,knots,cp_y,degree)
spl_z = bs.generate_spline(s_l,knots,cp_z,degree)

curve = vector(spl_x,spl_y,spl_z)

-- Orientation
spl_qx = bs.generate_spline(s_l,knots,cp_qx,degree)
spl_qy = bs.generate_spline(s_l,knots,cp_qy,degree)
spl_qz = bs.generate_spline(s_l,knots,cp_qz,degree)
spl_qw = bs.generate_spline(s_l,knots,cp_qw,degree)

q_norm = sqrt( spl_qx*spl_qx + spl_qy*spl_qy + spl_qz*spl_qz + spl_qw*spl_qw + constant(1E-8) )

spln_qx = spl_qx/q_norm
spln_qy = spl_qy/q_norm
spln_qz = spl_qz/q_norm
spln_qw = spl_qw/q_norm

R11 = 1-2*(spln_qy*spln_qy + spln_qz*spln_qz)
R12 = 2*(spln_qx*spln_qy - spln_qw*spln_qz)
R13 = 2*(spln_qx*spln_qz + spln_qw*spln_qy)
R21 = 2*(spln_qx*spln_qy + spln_qw*spln_qz)
R22 = 1-2*(spln_qx*spln_qx + spln_qz*spln_qz)
R23 = 2*(spln_qy*spln_qz - spln_qw*spln_qx)
R31 = 2*(spln_qx*spln_qz - spln_qw*spln_qy)
R32 = 2*(spln_qy*spln_qz + spln_qw*spln_qx)
R33 = 1-2*(spln_qx*spln_qx + spln_qy*spln_qy)

R = construct_rotation_from_vectors(vector(R11,R21,R31), vector(R12,R22,R32), vector(R13,R23,R33))

-- Combining position and orientation in frame expression
des_ee = frame(R, curve)
--des_ee = frame(rotation(startpose),curve) -- If you don't want to use the demonstrated orientation

-- Orientation freedom
R_ee_w = rotation(tf)
R_tra_w = rotation(tracker_frame)

R_ee_tra = inv(R_tra_w)*R_ee_w

R_tgt_w = rotation(des_ee)
R_tgt_ee = inv(R_ee_w)*R_tgt_w

rot_ee = getRotVec(R_tgt_ee)
rot_tra = R_ee_tra*rot_ee

--====== Constraints ======--
-- B-spline trajectory following
if rotation_symmetry then		-- Assumes symmetry around target z-axis
Constraint {
      context = ctx,
      name = "follow_position",
      expr = curve - origin(tf),
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

else
	if feedback_eot then		-- Target tracking near end of trajectory; other part of constraint set is given below
	Constraint {
        	context = ctx,
        	name = "feedback_eot",
        	expr = inv(tf)*tracker_frame,
        	K = 1,
        	priority = 2,
        	weight = eot_weight
	}
	else
	Constraint {
		context = ctx,
		name = "follow_path",
		expr = inv(des_ee)*tf,
		K = 1,
		priority = 2,
		weight = 1
	}
	end
end

if feedback_eot then			-- Second part of target tracking constraint set
Constraint {
        context = ctx,
        name = "follow_path",
        expr = inv(des_ee)*tf,
        K = 1,
        priority = 2,
        weight = 1-eot_weight
}
end

-- ======== Calculate task error during impedance control  ======== --
if not impedance_control then
task_error = norm( vector((coord_x(origin(tf)) - coord_x(curve)), (coord_y(origin(tf)) - coord_y(curve)), (coord_z(origin(tf)) - coord_z(curve))) )
else
task_error = constant(0.0)
end
-- ======== Set outputs ======== --
ctx:setOutputExpression("start_trajectory", startpose)
ctx:setOutputExpression("s", s_l)

ctx:setOutputExpression("task_error", task_error)

ctx:setOutputExpression("s_g", s_g)
