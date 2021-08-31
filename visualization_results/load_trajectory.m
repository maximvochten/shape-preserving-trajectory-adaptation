function [robot_timing, trajectory] = load_trajectory(filename)
%LOAD_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

% fileID = fopen(filename, 'r');
% formatSpec = '%f';
% 
% data = fscanf(fileID, formatSpec);

[time, R11, R21, R31, R12, R22, R32, R13, R23, R33, p1, p2, p3] = textread(filename, '%f %f %f %f %f %f %f %f %f %f %f %f %f');
time0 = time(1);
robot_timing = time(:)-time0;

trajectory.Obj_location = [p1 p2 p3];
for i=1:size(time)
    trajectory.Obj_frames(:,:,i) = [ R11(i) R12(i) R13(i); ...
        R21(i) R22(i) R23(i); ...
        R31(i) R32(i) R33(i) ];
end
end