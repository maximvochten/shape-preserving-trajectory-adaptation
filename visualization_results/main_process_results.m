close all;
clc;
clear;

% Config paths
addpath('plotting_functions/')
addpath('plotting_functions/export_fig')
results_folder = '../data/results/';
addpath(results_folder);

% Parameters
parameterization = 'geometric';
dt = 1/60; % timestep
show_demonstration = 1;
nb = 16; % nb of trajectories
save_videos = 0;
show_obstacle = 1;

%% Load measurements + invariants
if show_demonstration
    [tim, measured_pose_coordinates] = load_vive_data('../data/demonstrated_trajectories/sinus.txt');
    N = length(measured_pose_coordinates);
    meas_traj_time.Obj_location = measured_pose_coordinates(:,1:3); % position
    for i=1:N
        meas_traj_time.Obj_frames(:,:,i) = quat2rot(measured_pose_coordinates(i,4:7)); % rotation matrix
    end
    % For geometric invariants we also transform the time-trajectory R(t) and p(t) to the geometric trajectory R(theta) and p(s)
    if strcmp(parameterization,'geometric')
        [meas_traj,s,theta] = reparameterize_trajectory_geom(meas_traj_time,dt);
        h = 1/N;
    else
        meas_traj = meas_traj_time;
        h = dt;
    end
    plot_trajectory(meas_traj_time,[],'measurements',1,0);
    
    % Load calculated invariants + reconstructed trajectory
    demo_invars = load([results_folder,'invariants_demo.txt']);
    demo_traj = load([results_folder,'traj_demo.txt']);
    
    demon_traj.Obj_location = demo_traj(:,11:13);
    for i=1:N
        demon_traj.Obj_frames(:,:,i) = reshape(demo_traj(i,2:10),3,3); % rotation matrix
    end
    plot_trajectory2(meas_traj,demon_traj,'measurements(blue), calculated(red)',1);
    plot_descriptor(demo_invars,[],'calculated invariants',h,parameterization,'frenetserret')
    
    % Plot first new trajectory
    trajectory = load([results_folder,['traj',num2str(0),'.txt']]);
    n_traj.Obj_location = trajectory(:,11:13);
    for i=1:N
        n_traj.Obj_frames(:,:,i) = reshape(trajectory(i,2:10),3,3); % rotation matrix
    end
    plot_trajectory(demon_traj,n_traj,'measurements(blue), calculated(red)',1,0);
end

%% Load robot trajectory and global trajectories

%[robot_times,robot_trajectory] = load_robot_trajectory([results_folder,'joints.txt']);
[robot_times,robot_trajectory] = load_robot_trajectory_UR10([results_folder,'joints.txt']);
[traj_triggers,trajectories,invariants,sample_triggers] = load_invariant_trajectories(results_folder,nb);

plot_trajectory(robot_trajectory,trajectories{1},'robot trajectory',1,show_obstacle);

plot_robot_trajectories_all(robot_trajectory,trajectories);

% plot robot progress along local
%figure; plot(linspace(0,1,size(robot_trajectory),joints(:,2),'k-','linewidth',2); grid on; axis equal;
drawnow
%if 0
plot_robot_trajectories_animated(robot_trajectory,trajectories,robot_times,traj_triggers,save_videos,show_obstacle);

%
%    export_fig 'figures/3D_obstacle_v2.png' -transparent -m2.5
%end

%% Plot invariants

plot_descriptors_all(demo_invars,invariants,'invariants',h,parameterization,'frenetserret',sample_triggers)

%% Plot pose coordinates

% Demonstration
demo_posecoords = zeros(N,6);
demo_posecoords(:,4:6) = demon_traj.Obj_location;
for i=1:N
    demo_posecoords(i,1:3) = R2rpy(demon_traj.Obj_frames(:,:,i));
end

for k=1:nb
    traj = trajectories{k};
    Nt = size(traj.Obj_location,1);
    traj_posecoords = zeros(Nt,6);
    traj_posecoords(:,4:6) = traj.Obj_location;
    for i=1:Nt
        traj_posecoords(i,1:3) = R2rpy(traj.Obj_frames(:,:,i));
    end
    posecoords{k} = traj_posecoords;
end

% Trajectories
plot_descriptors_all(posecoords{1},posecoords,'',h,parameterization,'pose',sample_triggers)

