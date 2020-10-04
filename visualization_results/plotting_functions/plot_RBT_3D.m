function plot_RBT_3D_compare(T_demo,T_recon,titel,view_angles,adapt,halflength_position_coordinates,R_FS)

if nargin < 5
    adapt=0;
end

pose_coordinates_demo = poseMatrix2poseCoordinates(T_demo);
if ~isempty(T_recon)
    pose_coordinates_recon = poseMatrix2poseCoordinates(T_recon);
end

%h2=figure(11);
h2 = figure;

set(gcf,'units','normalized','outerposition',[0 0 0.6 0.7]);


%movegui(h2,'northwest')
clf
hold on
grid on;
%box on;
axis equal;
xlabel('$z$[mm]','Interpreter','LaTex','FontSize',18)
ylabel('$y$[mm]','Interpreter','LaTex','FontSize',18)
zlabel('$x$[mm]','Interpreter','LaTex','FontSize',18)

inc_fs = 50;

N = size(T_demo,3);
inc = 5; % how many rigid bodies drawn along trajectory - normal
%inc = 8; % how many rigid bodies drawn along trajectory - sine motion / hammering
len = 50; % length of arrow
lencx = 51.2; % length of cube
lency = 32; % length of cube
lencz = 32; % length of cube
 %t = 0.453; % thickness arrow - sine motion
 t = 0.453; % thickness arrow - normal
a=3; %xcoordinate
b=2;
c=1;

offset = 0*[0 ;  0 ; -100]; % for change in starting orientation
% Draw orientation rigid body demonstration


if ~isempty(T_recon)
    % Draw orientation rigid body reconstruction
    Rx_FS = []; Ry_FS = []; Rz_FS = []; p_recon = [];
    for j=round(linspace(1,N,inc_fs))
        Rx_FS = [Rx_FS ; R_FS(1:3,1,j)'];
        Ry_FS = [Ry_FS ; R_FS(1:3,2,j)'];
        Rz_FS = [Rz_FS ; R_FS(1:3,3,j)'];
        p_recon = [p_recon ; T_recon(1:3,4,j)'+ offset'];
        if j==N
            plot3(offset(3)+pose_coordinates_recon(j,a),offset(2)+pose_coordinates_recon(j,b),offset(1)+pose_coordinates_recon(j,c),'r*','MarkerSize',7,'LineWidth',2);
        else
            plot3(offset(3)+pose_coordinates_recon(j,a),offset(2)+pose_coordinates_recon(j,b),offset(1)+pose_coordinates_recon(j,c),'r.','MarkerSize',15);
        end
    end
    p_recon=p_recon(:,[a b c]);
    Rx_FS=Rx_FS(:,[a b c]);
    Ry_FS=Ry_FS(:,[a b c]);
    Rz_FS=Rz_FS(:,[a b c]);
    arrow3(p_recon,p_recon+len*Rx_FS,'_r',t)
    arrow3(p_recon,p_recon+len*Ry_FS,'_e',t)
    arrow3(p_recon,p_recon+len*Rz_FS,'_b',t)
end


% Plot demonstration
plot3(pose_coordinates_demo(:,a),pose_coordinates_demo(:,b),pose_coordinates_demo(:,c),'b-.','linewidth',1.5);
if ~isempty(T_recon)
    plot3(offset(3)+pose_coordinates_recon(:,a),offset(2)+pose_coordinates_recon(:,b),offset(1)+pose_coordinates_recon(:,c),'r','linewidth',1.5);
end
% Plot reconstruction
%plot3(T_recon(1,4),T_recon(2,4),T_recon(3,4),'r','linewidth',1.5);


if adapt
    plot3(halflength_position_coordinates(a),halflength_position_coordinates(b),halflength_position_coordinates(c),'k.','MarkerSize',35,'LineWidth',2);

    
    enddemo = pose_coordinates_recon(end,[a b c]);
    endrecon = pose_coordinates_demo(end,[a b c]);
    
    arrow3(endrecon,enddemo,'2k',2*t)
    
end

%title(titel)
view(view_angles(1),view_angles(2));

%grid on;
%box on;
axis equal;
zoom(0.9)

