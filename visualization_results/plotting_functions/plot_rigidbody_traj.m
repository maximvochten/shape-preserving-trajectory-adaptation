function plot_rigidbody_traj(R_demo,p_obj_demo,R_recon,p_obj_recon,constraints,titel)

figure;
set(gcf,'units','normalized','outerposition',[0 0 0.6 0.7]);
clf
hold on
grid on;
box on;
axis equal;
xlabel('$x$[m]','Interpreter','LaTex','FontSize',18)
ylabel('$y$[m]','Interpreter','LaTex','FontSize',18)
zlabel('$z$[m]','Interpreter','LaTex','FontSize',18)

% scale calculation to get an idea of how big figure is
scale1 = (max(p_obj_demo(:,1))-min(p_obj_demo(:,1))) + (max(p_obj_demo(:,2))-min(p_obj_demo(:,2))) + (max(p_obj_demo(:,3))-min(p_obj_demo(:,3)));
if ~isempty(R_recon)
    scale2 = (max(p_obj_recon(:,1))-min(p_obj_recon(:,1))) + (max(p_obj_recon(:,2))-min(p_obj_recon(:,2))) + (max(p_obj_recon(:,3))-min(p_obj_recon(:,3)));
else
    scale2 = 0;
end
scale = max(scale1,scale2);

N = size(R_demo,3);

inc = 10; % how many rigid bodies drawn along trajectory %5 for pouring, 8 for sine, 4 for obstacle avoidance

lencx = 0.07*scale; % length of cube
lency = 0.05*scale; % length of cube
lencz = 0.05*scale; % length of cube

view_angles = [-70 ; 12];

len = 0.95*lencx; % length arrow
t = 1; % thickness arrowhead % 0.85 for sine, 1 for pouring
linewidth = '1'; % width arrow

a=1; %xcoordinate
b=2;
c=3;

offset = 0*[0 ;  0 ; -100]; % for change in starting orientation
% Draw orientation rigid body demonstration
Rx_demo = []; Ry_demo = []; Rz_demo = []; p_demo = [];
for j=round(linspace(1,N,inc))
    Rx_demo = [Rx_demo ; R_demo(1:3,1,j)'];
    Ry_demo = [Ry_demo ; R_demo(1:3,2,j)'];
    Rz_demo = [Rz_demo ; R_demo(1:3,3,j)'];
    p_demo = [p_demo ; p_obj_demo(j,:)];
    if j==N
        plot3(p_obj_demo(j,a),p_obj_demo(j,b),p_obj_demo(j,c),'b*','MarkerSize',7,'LineWidth',2);
    else
        plot3(p_obj_demo(j,a),p_obj_demo(j,b),p_obj_demo(j,c),'b.','MarkerSize',15);
    end
end
p_demo=p_demo(:,[a b c]);
Rx_demo=Rx_demo(:,[a b c]);
Ry_demo=Ry_demo(:,[a b c]);
Rz_demo=Rz_demo(:,[a b c]);
view(view_angles(1),view_angles(2));
arrow3(p_demo,p_demo+len*Rx_demo,['_r' linewidth],t,2*t)
arrow3(p_demo,p_demo+len*Ry_demo,['_e' linewidth],t,2*t)
arrow3(p_demo,p_demo+len*Rz_demo,['_b' linewidth],t,2*t)

if ~isempty(R_recon)
    % Draw orientation rigid body reconstruction
    Rx_recon = []; Ry_recon = []; Rz_recon = []; p_recon = [];
    for j=round(linspace(1,N,inc))
        Rx_recon = [Rx_recon ; R_recon(1:3,1,j)'];
        Ry_recon = [Ry_recon ; R_recon(1:3,2,j)'];
        Rz_recon = [Rz_recon ; R_recon(1:3,3,j)'];
        p_recon = [p_recon ; p_obj_recon(j,:)];
        if j==N
            plot3(offset(3)+p_obj_recon(j,a),offset(2)+p_obj_recon(j,b),offset(1)+p_obj_recon(j,c),'r*','MarkerSize',7,'LineWidth',2);
        else
            plot3(offset(3)+p_obj_recon(j,a),offset(2)+p_obj_recon(j,b),offset(1)+p_obj_recon(j,c),'r.','MarkerSize',15);
        end
    end
    p_recon=p_recon(:,[a b c]);
    Rx_recon=Rx_recon(:,[a b c]);
    Ry_recon=Ry_recon(:,[a b c]);
    Rz_recon=Rz_recon(:,[a b c]);
    arrow3(p_recon,p_recon+len*Rx_recon,['_r' linewidth],t,2*t)
    arrow3(p_recon,p_recon+len*Ry_recon,['_e' linewidth],t,2*t)
    arrow3(p_recon,p_recon+len*Rz_recon,['_b' linewidth],t,2*t)
end

% Draw cube demonstration
for j=round(linspace(1,N,inc))
    fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    vm = repmat(p_obj_demo(j,:),8,1) + [ zeros(3,1) ...
        R_demo(1:3,1:3,j)*[lencx; 0; 0] R_demo(1:3,1:3,j)*[lencx; lency; 0] ...
        R_demo(1:3,1:3,j)*[0; lency; 0] R_demo(1:3,1:3,j)*[0; 0; lencz] ...
        R_demo(1:3,1:3,j)*[lencx; 0; lencz] R_demo(1:3,1:3,j)*[lencx; lency; lencz] ...
        R_demo(1:3,1:3,j)*[0; lency; lencz] ]';
    vm=vm(:,[a b c]);
    if j==1
        patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',1.0,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30,'EdgeColor',[0.15 0.15 0.90],'LineWidth',0.75);
        patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',1.0,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.85,'EdgeColor',[0.15 0.15 0.90],'LineWidth',0.75);
    else
        patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30,'EdgeColor',[0.15 0.15 0.90],'LineWidth',0.75);
        patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',0.8,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.85,'EdgeColor',[0.15 0.15 0.90],'LineWidth',0.75);
    end
end

if ~isempty(R_recon)
    % Draw cube reconstruction
    for j=round(linspace(1,N,inc))
        fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
        vm = repmat(p_obj_recon(j,:)+offset',8,1) + [ zeros(3,1) ...
            R_recon(1:3,1:3,j)*[lencx; 0; 0] R_recon(1:3,1:3,j)*[lencx; lency; 0] ...
            R_recon(1:3,1:3,j)*[0; lency; 0] R_recon(1:3,1:3,j)*[0; 0; lencz] ...
            R_recon(1:3,1:3,j)*[lencx; 0; lencz] R_recon(1:3,1:3,j)*[lencx; lency; lencz] ...
            R_recon(1:3,1:3,j)*[0; lency; lencz] ]';
        vm=vm(:,[a b c]);
        if j==1
            patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',1.0,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30,'EdgeColor',[0.15 0.15 0.90],'LineWidth',0.75);
            patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',1.0,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.95,'EdgeColor',[0.15 0.15 0.90],'LineWidth',0.75);
        else
            patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.10,'EdgeColor',[0.90 0.15 0.15],'LineWidth',0.75);
            patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.40,'EdgeColor',[0.90 0.15 0.15],'LineWidth',0.75);
        end
    end
end

% Plot demonstration
plot3(p_obj_demo(:,a),p_obj_demo(:,b),p_obj_demo(:,c),'b-.','linewidth',1.5);
if ~isempty(R_recon)
    plot3(offset(3)+p_obj_recon(:,a),offset(2)+p_obj_recon(:,b),offset(1)+p_obj_recon(:,c),'r','linewidth',1.5);
end

% Plot reconstruction
%plot3(R_recon(1,4),R_recon(2,4),R_recon(3,4),'r','linewidth',1.5);

% Plot obstacle
if isfield(constraints,'obstacle')
    
    r = constraints.obstacle.safety_radius;
    [x,y,z] = sphere(50);
    x0 = constraints.obstacle.location(a); y0 = constraints.obstacle.location(b); z0 = constraints.obstacle.location(c);
    x = x*r + x0;
    y = y*r + y0;
    z = z*r + z0;
    
    lightGrey = [34,139,34]/255; %0.8*[1 1 1]; % It looks better if the lines are lighter
    surface(x,y,z,'FaceColor', lightGrey,'facealpha',0.75,'EdgeColor','none')
    
    
end

if ~isempty(R_recon) && sum(p_demo(end,:)-p_recon(end,:))~=0
    arrow3(p_demo(end,:),p_recon(end,:),['_k2'],1.5);
end

%title(titel)
view(view_angles(1),view_angles(2));
%grid on;
%box on;
axis equal;
%zoom(0.75)

