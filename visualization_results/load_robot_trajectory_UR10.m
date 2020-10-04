function [robot_timing, trajectory] = load_robot_trajectory_UR10(filename)

joints = load(filename);
time0 = joints(1,1);
robot_timing = joints(:,1) - time0;
%dt = mean(diff(joints(:,1)));


trajectory.Obj_location = joints(:,12:14);
for i=1:size(joints,1)
    trajectory.Obj_frames(:,:,i) = reshape(joints(i,3:11),3,3);
end
