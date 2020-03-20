function plot_robot_trajectories_animated(robot_trajectory,generated_trajectories,robot_times,traj_triggers,save_videos)

traj_triggers = [traj_triggers;Inf];

% Load robot trajectory
p_robot = robot_trajectory.Obj_location;
R_robot = robot_trajectory.Obj_frames;

% Number of generated trajectories
nb_trajs = length(generated_trajectories);

% Determine overall size of figure
xmin = min(p_robot(:,1));
xmax = max(p_robot(:,1));
ymin = min(p_robot(:,2));
ymax = max(p_robot(:,2));
zmin = min(p_robot(:,3));
zmax = max(p_robot(:,3));
scale = sqrt( (xmax-xmin)^2 + (ymax-ymin)^2 + (zmax-zmin)^2 );
margin = 0.35;

% Parameters for plotting rigid-body orientation: cube + arrows
lencx = 0.1; % length of cube in X-direction body frame
lency = 0.060; % length of cube in Y-direction body frame
lencz = 0.060; % length of cube in Z-direction body frame
cube_lengths = [lencx lency lencz];
len = 2*lencx; % length of arrow
t = 1; % thickness arrowhead
linewidth = '1'; % width arrow
% draw_arrows_trajectories = 1;
% draw_arrows_robot = 1;
% draw_cubes_trajectories = 1;
% draw_cubes_robot = 1;

% Styling the plot
h = figure; clf;
set(h,'units','normalized','outerposition',[0 0 0.6 0.7]);
hold on; axis equal; view(60,6); grid on; box on;
xlabel('$x$[m]','Interpreter','LaTex','FontSize',18)
ylabel('$y$[m]','Interpreter','LaTex','FontSize',18)
zlabel('$z$[m]','Interpreter','LaTex','FontSize',18)
xlim([xmin-margin xmax+margin])
ylim([ymin-margin-0.1 ymax+margin])
zlim([zmin-margin zmax+margin])
title('')

%% Initialize animation

% Initialize animated position of robot
lineplot_robot = animatedline('Linewidth',3,'Color','b','LineStyle','-');
addpoints(lineplot_robot,p_robot(1,1),p_robot(1,2),p_robot(1,3));
marker = plot3(p_robot(1,1),p_robot(1,2),p_robot(1,3),'o','MarkerFaceColor','blue','MarkerSize',7);

% Plot complete first trajectory
p_obj = generated_trajectories{1}.Obj_location;
R_obj = generated_trajectories{1}.Obj_frames;
plot3(p_obj(:,1),p_obj(:,2),p_obj(:,3),'LineWidth',1.5);

% Setpoint robot controller
% closepoint = plot3(trajectories{1}.Obj_location(1,1),trajectories{1}.Obj_location(1,2),trajectories{1}.Obj_location(1,3),'o','MarkerFaceColor','blue');

% Initialize orientation robot with arrows
p_demo = [p_robot(1,1);p_robot(1,2);p_robot(1,3)]';
h1 = arrow3(p_demo,p_demo+len*R_robot(:,1,1)',['_r' linewidth],t,2*t); %_y
h2 = arrow3(p_demo,p_demo+len*R_robot(:,2,1)',['_e' linewidth],t,2*t); %_m
h3 = arrow3(p_demo,p_demo+len*R_robot(:,3,1)',['_b' linewidth],t,2*t); %_c

% Initialize orientation target pose with arrows
p_target = [p_obj(end,1);p_obj(end,2);p_obj(end,3)]';
h4 = arrow3(p_target,p_target+len*R_obj(:,1,end)',['_y' linewidth],t,2*t); %_y
h5 = arrow3(p_target,p_target+len*R_obj(:,2,end)',['_m' linewidth],t,2*t); %_m
h6 = arrow3(p_target,p_target+len*R_obj(:,3,end)',['_c' linewidth],t,2*t); %_c
arrow3(p_target,p_target+len*R_obj(:,1,end)',['_y' linewidth],t,2*t); %_y
arrow3(p_target,p_target+len*R_obj(:,2,end)',['_m' linewidth],t,2*t); %_m
arrow3(p_target,p_target+len*R_obj(:,3,end)',['_c' linewidth],t,2*t); %_c

% Initialize orientation robot with cube
R = R_robot(:,:,1);
p = p_demo(:,1);
fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
vm = repmat(p,8,1) + (R(1:3,1:3)*[ [-lencx/2; -lency/2; -lencz/2] ...
    [lencx/2; -lency/2; -lencz/2] [lencx/2; lency/2; -lencz/2] ...
    [-lencx/2; lency/2; -lencz/2] [-lencx/2; -lency/2; lencz/2] ...
    [lencx/2; -lency/2; lencz/2] [lencx/2; lency/2; lencz/2] ...
   [-lencx/2; lency/2; lencz/2] ])';
h_cube = patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30,'EdgeColor',[0.15 0.15 0.90],'LineWidth',1.00);

% % Initialize movie structure
% allTheFrames = cell(1+length(1:5:length(robot_times)),1);
% allTheFrames(:) = {zeros(671, 1152, 3, 'uint8')};
% % Next get a cell array with all the colormaps.
% allTheColorMaps = cell(1+length(1:5:length(robot_times)),1);
% allTheColorMaps(:) = {zeros(256, 3)};
% % Now combine these to make the array of structures.
% F = struct('cdata', allTheFrames, 'colormap', allTheColorMaps);

% Arrow from start to end
arrow3(p_obj(end,:),p_robot(end,:),['_k1.5'],0);

%% Obstacle
%cylinder mesh
Ncyl = 100;
r = 0.05;
h = 0.5;
[xcyl, ycyl, zcyl] = cylinder(r,Ncyl); %input r, gives back unit height cylinder
[xcyl2, ycyl2, zcyl2] = cylinder(0.9*r,Ncyl); %input r, gives back unit height cylinder

%cylinder position
T_w_obstacle = [1 0 0 0.04 ; 0 1 0 -0.42 ; 0 0 1 0.38; 0 0 0 1];

%inside cylinder
h_obstacle = surf(xcyl,ycyl,h*zcyl-h/2,'EdgeAlpha',0.0,'FaceColor',[0.2 0.8 0.8],'FaceAlpha',1,'FaceLighting','phong');
h_hole = surf(xcyl2,ycyl2,h*zcyl-h/2,'EdgeAlpha',0.0,'FaceColor',[0.1 0.5 0.5],'FaceAlpha',1.0,'FaceLighting','phong');

%draw everything
combinedobjects = hgtransform('parent',gca);
set(h_obstacle, 'parent', combinedobjects);
set(h_hole, 'parent', combinedobjects);

set(combinedobjects, 'matrix', T_w_obstacle);
drawnow

%% Animation

count = 1;
F(count) = getframe(gcf);
drawnow
traj_index = 1;
for i=1:10:length(robot_times)
    % p_obj = trajectories{i}.Obj_location;
    % plot3(p_obj(:,1),p_obj(:,2),p_obj(:,3),'LineWidth',1.5);
    
    % Delete previous arrows
    delete(h1);
    delete(h2);
    delete(h3);
    
    % Check if time exceeds the trigger to load in the next trajectory
    if (robot_times(i) > traj_triggers(traj_index+1))
        traj_index = traj_index + 1; % next trajectory
        p_obj = generated_trajectories{traj_index}.Obj_location;
        plot3(p_obj(:,1),p_obj(:,2),p_obj(:,3),'LineWidth',1.5); % plot next trajectory
        
        %if traj_index ~= nb_trajs
        delete(h4);
        delete(h5);
        delete(h6);
        
        p_obj = generated_trajectories{traj_index}.Obj_location;
        R_obj = generated_trajectories{traj_index}.Obj_frames;
        plot3(p_obj(end,1),p_obj(end,2),p_obj(end,3),'o','MarkerFaceColor','black','MarkerSize',1);
        
        % Plot orientation target with arrows
        p_target = [p_obj(end,1);p_obj(end,2);p_obj(end,3)]';
        h4 = arrow3(p_target,p_target+len*R_obj(:,1,end)',['_y' linewidth],t,2*t); %_y
        h5 = arrow3(p_target,p_target+len*R_obj(:,2,end)',['_m' linewidth],t,2*t); %_m
        h6 = arrow3(p_target,p_target+len*R_obj(:,3,end)',['_c' linewidth],t,2*t); %_c
        %end
    end
    
    
    
    % Plot robot orientation using arrows
    p_demo = [p_robot(i,1);p_robot(i,2);p_robot(i,3)]';
    h1 = arrow3(p_demo,p_demo+len*R_robot(:,1,i)',['_r' linewidth],t,2*t);   %_y
    h2 = arrow3(p_demo,p_demo+len*R_robot(:,2,i)',['_e' linewidth],t,2*t);    %_m
    h3 = arrow3(p_demo,p_demo+len*R_robot(:,3,i)',['_b' linewidth],t,2*t);    %_c
    
    % Update marker with position of robot
    marker.XData = p_robot(i,1);
    marker.YData = p_robot(i,2);
    marker.ZData = p_robot(i,3);
    addpoints(lineplot_robot,p_robot(i,1),p_robot(i,2),p_robot(i,3))
    
    % Update robot orientation using box
    R = R_robot(:,:,i);
    p = p_demo;
    vm = repmat(p,8,1) + (R(1:3,1:3)*[ [-lencx/2; -lency/2; -lencz/2] ...
        [lencx/2; -lency/2; -lencz/2] [lencx/2; lency/2; -lencz/2] ...
        [-lencx/2; lency/2; -lencz/2] [-lencx/2; -lency/2; lencz/2] ...
        [lencx/2; -lency/2; lencz/2] [lencx/2; lency/2; lencz/2] ...
        [-lencx/2; lency/2; lencz/2] ])';
    h_cube.Vertices = vm;
    
    %%Setpoint robot controller
    %index_closepoint = round(joints(k,2)*152);
    %closepoint.XData = trajectories{1}.Obj_location(index_closepoint,1);
    %closepoint.YData = trajectories{1}.Obj_location(index_closepoint,2);
    %closepoint.ZData = trajectories{1}.Obj_location(index_closepoint,3);
    
    %%Arrow of target point
    %arrow3(p_obj(end,:),p_robot(end,:),['_k1.5'],1.5);
    %plot3(p_obj(end,1),p_obj(end,2),p_obj(end,3),'o','MarkerFaceColor','black','MarkerSize',8);
    
    count = count+1;
    F(count) = getframe(gcf);
    drawnow
    %pause(0.01)
    
end
hold off

if save_videos
    video = VideoWriter('test3');
    video.FrameRate = 30;
    open(video)
    writeVideo(video,F)
    close(video)
end




