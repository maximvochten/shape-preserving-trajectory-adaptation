require("context")
require("geometric")
require("collision")

tf = task_frame

startpose = initial_value(time, tf)
startpos = origin(startpose)

--obstacle_pose = startpose*translate_x(0.036)*translate_z(-0.1)
--obstacle_pose = startpose*translate_x(0.036+0.02*time)*translate_y(-0.05*time)*translate_z(-0.1-0.05*time)
--obstacle_pose = startpos*translate_x(0.036)*translate_y(-0.9)*translate_z(-0.1)
--obstacle_pose = startpos + vector(0.1,0.1,0.1)

-- ====== Viz marker obstacle =====
p_x = -coord_x(origin(obstacle_pose))
p_y = -coord_y(origin(obstacle_pose))
p_z = coord_z(origin(obstacle_pose))
p = vector(p_x,p_y,p_z)
obstacle_pose_viz = frame(rotation(obstacle_pose),p)

d = distance_between(obstacle_pose, Sphere(0.50), 0.00, 0.001, tf, CapsuleZ(0.30,0.40), 0.00, 0.001)

Constraint {
	context = ctx,
	name = "collision_avoidance",
	expr = d,
	target_lower = 0.3,
	weight = 2,
	K = 0.5, 	
	priority = 1
}


-- ===== Set output of etaslcore =====
ctx:setOutputExpression("Fobstacle",obstacle_pose_viz)
ctx:setOutputExpression("distance", d)
