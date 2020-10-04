function plot_RBT_3D_compare(T_demo,T_recon,titel,view_angles,adapt,halflength_position_coordinates)

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



N = size(T_demo,3);
inc = 4; % how many rigid bodies drawn along trajectory - normal
%inc = 8; % how many rigid bodies drawn along trajectory - sine motion / hammering
len = 44.8; % length of arrow
lencx = 51.2; % length of cube
lency = 32; % length of cube
lencz = 32; % length of cube
 %t = 0.453; % thickness arrow - sine motion
 t = 0.853; % thickness arrow - normal
a=3; %xcoordinate
b=2;
c=1;

offset = 0*[0 ;  0 ; -100]; % for change in starting orientation
% Draw orientation rigid body demonstration
Rx_demo = []; Ry_demo = []; Rz_demo = []; p_demo = [];
for j=round(linspace(1,N,inc))
    Rx_demo = [Rx_demo ; T_demo(1:3,1,j)'];
    Ry_demo = [Ry_demo ; T_demo(1:3,2,j)'];
    Rz_demo = [Rz_demo ; T_demo(1:3,3,j)'];
    p_demo = [p_demo ; T_demo(1:3,4,j)'];
    if j==N
        plot3(pose_coordinates_demo(j,a),pose_coordinates_demo(j,b),pose_coordinates_demo(j,c),'b*','MarkerSize',7,'LineWidth',2);
    else
        plot3(pose_coordinates_demo(j,a),pose_coordinates_demo(j,b),pose_coordinates_demo(j,c),'b.','MarkerSize',15);
    end
end
p_demo=p_demo(:,[a b c]);
Rx_demo=Rx_demo(:,[a b c]);
Ry_demo=Ry_demo(:,[a b c]);
Rz_demo=Rz_demo(:,[a b c]);
arrow3(p_demo,p_demo+len*Rx_demo,'_r',t)
arrow3(p_demo,p_demo+len*Ry_demo,'_e',t)
arrow3(p_demo,p_demo+len*Rz_demo,'_b',t)

if ~isempty(T_recon)
    % Draw orientation rigid body reconstruction
    Rx_recon = []; Ry_recon = []; Rz_recon = []; p_recon = [];
    for j=round(linspace(1,N,inc))
        Rx_recon = [Rx_recon ; T_recon(1:3,1,j)'];
        Ry_recon = [Ry_recon ; T_recon(1:3,2,j)'];
        Rz_recon = [Rz_recon ; T_recon(1:3,3,j)'];
        p_recon = [p_recon ; T_recon(1:3,4,j)'+ offset'];
        if j==N
            plot3(offset(3)+pose_coordinates_recon(j,a),offset(2)+pose_coordinates_recon(j,b),offset(1)+pose_coordinates_recon(j,c),'r*','MarkerSize',7,'LineWidth',2);
        else
            plot3(offset(3)+pose_coordinates_recon(j,a),offset(2)+pose_coordinates_recon(j,b),offset(1)+pose_coordinates_recon(j,c),'r.','MarkerSize',15);
        end
    end
    p_recon=p_recon(:,[a b c]);
    Rx_recon=Rx_recon(:,[a b c]);
    Ry_recon=Ry_recon(:,[a b c]);
    Rz_recon=Rz_recon(:,[a b c]);
    arrow3(p_recon,p_recon+len*Rx_recon,'_r',t)
    arrow3(p_recon,p_recon+len*Ry_recon,'_e',t)
    arrow3(p_recon,p_recon+len*Rz_recon,'_b',t)
end

% Draw cube demonstration
for j=round(linspace(1,N,inc))
    fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    vm = repmat(T_demo(1:3,4,j)',8,1) + [ zeros(3,1) ...
        T_demo(1:3,1:3,j)*[lencx; 0; 0] T_demo(1:3,1:3,j)*[lencx; lency; 0] ...
        T_demo(1:3,1:3,j)*[0; lency; 0] T_demo(1:3,1:3,j)*[0; 0; lencz] ...
        T_demo(1:3,1:3,j)*[lencx; 0; lencz] T_demo(1:3,1:3,j)*[lencx; lency; lencz] ...
        T_demo(1:3,1:3,j)*[0; lency; lencz] ]';
    vm=vm(:,[a b c]);
    if j==1
        patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',1.0,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30);
    else
        patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30);
    end
end

if ~isempty(T_recon)
    % Draw cube reconstruction
    for j=round(linspace(1,N,inc))
        fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
        vm = repmat(T_recon(1:3,4,j)'+offset',8,1) + [ zeros(3,1) ...
            T_recon(1:3,1:3,j)*[lencx; 0; 0] T_recon(1:3,1:3,j)*[lencx; lency; 0] ...
            T_recon(1:3,1:3,j)*[0; lency; 0] T_recon(1:3,1:3,j)*[0; 0; lencz] ...
            T_recon(1:3,1:3,j)*[lencx; 0; lencz] T_recon(1:3,1:3,j)*[lencx; lency; lencz] ...
            T_recon(1:3,1:3,j)*[0; lency; lencz] ]';
        vm=vm(:,[a b c]);
        if j==1
            patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',1.0,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.05);
        else
            patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.05);
        end
    end
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

