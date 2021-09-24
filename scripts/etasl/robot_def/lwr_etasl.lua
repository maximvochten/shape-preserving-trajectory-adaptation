require("context")
require("geometric")

local u=UrdfExpr();
u:readFromFile(rospack_find("etasl_invariants_integration").."/robot_description/urdf/lwr/use_case_setup_lwr.urdf")
u:addTransform("tool_frame","lwr_arm_link_7","world")

local r = u:getExpressions(ctx)
robot_joints=u:getAllJointNames()

task_frame = r.tool_frame
FT_frame = r.tool_frame
