function [robot_timing, trajectory] = load_robot_trajectory(filename)

joints = load(filename);
time0 = joints(1,1);
robot_timing = joints(:,1) - time0;
%dt = mean(diff(joints(:,1)));

trajectory.Obj_location = joints(:,3:5);
for i=1:size(joints,1)
    trajectory.Obj_frames(:,:,i) = rpy2R([joints(i,8) joints(i,7) joints(i,6)]);
end
