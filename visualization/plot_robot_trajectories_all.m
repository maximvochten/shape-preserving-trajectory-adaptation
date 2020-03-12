function plot_robot_trajectories_all(robot_trajectory,generated_trajectories)

p_robot = robot_trajectory.Obj_location;
R_robot = robot_trajectory.Obj_frames;

% Parameters for plotting
nb_rigidbodies = 5; % how many rigid bodies are drawn along trajectory
% scale calculation to get an idea of how big trajectory is
scale = (max(p_robot(:,1))-min(p_robot(:,1))) + (max(p_robot(:,2))-min(p_robot(:,2))) + (max(p_robot(:,3))-min(p_robot(:,3)));
linestyles = {'-','--',':','-.','-','--',':','-.','-','--',':','-.'}; % different linestyles for the different trajectories
titletext = 'empty';
view_angles = [-150,45];

% Parameters for plotting rigid-body orientation
lencx = 0.1*scale; % length of cube in X-direction body frame
lency = 0.060*scale; % length of cube in Y-direction body frame
lencz = 0.060*scale; % length of cube in Z-direction body frame
cube_lengths = [lencx lency lencz];
len = 1.2*lencx; % length of arrow
t = 1; % thickness arrowhead
linewidth = '1'; % width arrow
draw_arrows_trajectories = 0;
draw_arrows_robot = 0;
draw_cubes_trajectories = 1;
draw_cubes_robot = 1;

% Styling the plot
h = figure; clf;
set(h,'units','normalized','outerposition',[0 0 0.6 0.7]);
hold on; axis equal; view(-161,25); grid on; box on;
xlabel('$x$[m]','Interpreter','LaTex','FontSize',18)
ylabel('$y$[m]','Interpreter','LaTex','FontSize',18)
zlabel('$z$[m]','Interpreter','LaTex','FontSize',18)
title(titletext)

%% Trajectory 1 (robot)

% plot trajectory reference point
plot3(p_robot(:,1),p_robot(:,2),p_robot(:,3),'b.-','linewidth',1.5);
plot3(p_robot(1,1),p_robot(1,2),p_robot(1,3),'bo','MarkerSize',7,'LineWidth',2);
plot3(p_robot(end,1),p_robot(end,2),p_robot(end,3),'b*','MarkerSize',7,'LineWidth',2);

% draw orientation rigid-body demonstration
if draw_arrows_robot
    draw_arrows(p_robot,R_robot,len,linewidth,t,nb_rigidbodies)
end
axis equal;
if draw_cubes_robot 
    draw_cubes(p_robot,R_robot,cube_lengths,nb_rigidbodies,1); 
end

%% Trajectory 2 (generated)

M = size(generated_trajectories,1);
for m=1:M
    p_obj_recon = generated_trajectories{m}.Obj_location;
    R_recon = generated_trajectories{m}.Obj_frames;
     
    % plot trajectory reference point
    plot3(p_obj_recon(:,1),p_obj_recon(:,2),p_obj_recon(:,3),'r.','linewidth',1.5);
    plot3(p_obj_recon(1,1),p_obj_recon(1,2),p_obj_recon(1,3),'ro','MarkerSize',7,'LineWidth',2);
    plot3(p_obj_recon(end,1),p_obj_recon(end,2),p_obj_recon(end,3),'r*','MarkerSize',7,'LineWidth',2);
    
    % draw orientation rigid-body demonstration
    if draw_arrows_trajectories
        draw_arrows(p_obj_recon,R_recon,len/4,linewidth,t/4,nb_rigidbodies)
    end
    axis equal;
    if draw_cubes_trajectories
        draw_cubes(p_obj_recon,R_recon,cube_lengths/2,nb_rigidbodies,0);
    end
end

view(view_angles(1),view_angles(2));
axis equal;