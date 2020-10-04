function plot_rigidbody_traj_struct(R_demo,p_obj_demo,R_recon_struct,p_obj_recon_struct,q,constraints_struct,title,paper_experiment_type,show_rotation)

if isempty(constraints_struct)
    M=1;
else
    M = length(constraints_struct);
end

if nargin == 8
    show_rotation = 1;
end

figure;
set(gcf,'units','normalized','outerposition',[0 0 0.6 0.7]);
clf
hold on
grid on;
%box on;
axis equal;
xlabel('$x$[m]','Interpreter','LaTex','FontSize',18)
ylabel('$y$[m]','Interpreter','LaTex','FontSize',18)
zlabel('$z$[m]','Interpreter','LaTex','FontSize',18)

linestyles = {'-','--',':','-.','-','--',':','-.','-','--',':','-.'};

% scale calculation to get an idea of how big figure is
scale1 = (max(p_obj_demo(:,1))-min(p_obj_demo(:,1))) + (max(p_obj_demo(:,2))-min(p_obj_demo(:,2))) + (max(p_obj_demo(:,3))-min(p_obj_demo(:,3)));
% if ~isempty(R_recon{1})
%     scale2 = (max(p_obj_recon(:,1))-min(p_obj_recon(:,1))) + (max(p_obj_recon(:,2))-min(p_obj_recon(:,2))) + (max(p_obj_recon(:,3))-min(p_obj_recon(:,3)));
% else
%     scale2 = 0;
% end
%scale = max(scale1,scale2);
scale = scale1;

N = size(R_demo,3);

inc = 5; % how many rigid bodies drawn along trajectory %5 for pouring, 8 for sine, 4 for obstacle avoidance

lencx = 0.090*scale; % length of cube
lency = 0.040*scale; % length of cube
lencz = 0.040*scale; % length of cube

len = 1.2*lencx; % length arrow
t = 1; % thickness arrowhead % 0.85 for sine, 1 for pouring
linewidth = '1'; % width arrow

if strcmp(paper_experiment_type,'jerkaccuracy') || strcmp(paper_experiment_type,'dmp') || strcmp(paper_experiment_type,'robot')
    a=1;b=2;c=3;
    view_angles = [-70 ; 12];
    inc = 3;
else
    a=3; %xcoordinate
    b=2;
    c=1;
    view_angles = [-119 ; 16];
end
if strcmp(paper_experiment_type,'refpoint_sai')
    view_angles = [-126 ; 12];
end
if strcmp(paper_experiment_type,'robot')
    R_recon = R_recon_struct{1};
    for i=1:N
        R_demo(:,:,i) = R_demo(:,:,i)*rpy2R([0 -pi/2 0]);
        R_recon(:,:,i) = R_recon(:,:,i)*rpy2R([0 -pi/2 0]);
    end
    R_recon_struct{1} = R_recon;
end



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
    elseif j==1
        plot3(p_obj_demo(j,a),p_obj_demo(j,b),p_obj_demo(j,c),'bo','MarkerSize',7,'LineWidth',2);
    else
        if show_rotation
            plot3(p_obj_demo(j,a),p_obj_demo(j,b),p_obj_demo(j,c),'b.','MarkerSize',15);
        else
            plot3(p_obj_demo(j,a),p_obj_demo(j,b),p_obj_demo(j,c),'b','MarkerSize',15);
        end
    end
end
p_demo=p_demo(:,[a b c]);
Rx_demo=Rx_demo(:,[a b c]);
Ry_demo=Ry_demo(:,[a b c]);
Rz_demo=Rz_demo(:,[a b c]);
view(view_angles(1),view_angles(2));
if show_rotation
    arrow3(p_demo,p_demo+len*Rx_demo,['_r' linewidth],t,2*t)
    arrow3(p_demo,p_demo+len*Ry_demo,['_e' linewidth],t,2*t)
    arrow3(p_demo,p_demo+len*Rz_demo,['_b' linewidth],t,2*t)
end

for m=1:M
    R_recon = R_recon_struct{m};
    p_obj_recon = p_obj_recon_struct{m};
    
    % Draw orientation rigid body reconstruction
    Rx_recon = []; Ry_recon = []; Rz_recon = []; p_recon = [];
    for j=round(linspace(1,N,inc))
        Rx_recon = [Rx_recon ; R_recon(1:3,1,j)'];
        Ry_recon = [Ry_recon ; R_recon(1:3,2,j)'];
        Rz_recon = [Rz_recon ; R_recon(1:3,3,j)'];
        p_recon = [p_recon ; p_obj_recon(j,:)];
        if j==N
            plot3(offset(3)+p_obj_recon(j,a),offset(2)+p_obj_recon(j,b),offset(1)+p_obj_recon(j,c),'r*','MarkerSize',7,'LineWidth',2);
        elseif j==1 && ~strcmp(paper_experiment_type,'target_pose')
            plot3(offset(3)+p_obj_recon(j,a),offset(2)+p_obj_recon(j,b),offset(1)+p_obj_recon(j,c),'ro','MarkerSize',7,'LineWidth',2);
        elseif j~=1 && j~=N
            if show_rotation
                plot3(offset(3)+p_obj_recon(j,a),offset(2)+p_obj_recon(j,b),offset(1)+p_obj_recon(j,c),'r.','MarkerSize',15);
            else
                plot3(offset(3)+p_obj_recon(j,a),offset(2)+p_obj_recon(j,b),offset(1)+p_obj_recon(j,c),'r','MarkerSize',15);
            end
        end
    end
    p_recon=p_recon(:,[a b c]);
    Rx_recon=Rx_recon(:,[a b c]);
    Ry_recon=Ry_recon(:,[a b c]);
    Rz_recon=Rz_recon(:,[a b c]);
    if show_rotation
        arrow3(p_recon,p_recon+len*Rx_recon,['_r' linewidth],t,2*t)
        arrow3(p_recon,p_recon+len*Ry_recon,['_e' linewidth],t,2*t)
        arrow3(p_recon,p_recon+len*Rz_recon,['_b' linewidth],t,2*t)
    end
end

% Draw cube demonstration
if show_rotation
    for j=round(linspace(1,N,inc))
        fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
        vm = repmat(p_obj_demo(j,:),8,1) + [ zeros(3,1) ...
            R_demo(1:3,1:3,j)*[lencx; 0; 0] R_demo(1:3,1:3,j)*[lencx; lency; 0] ...
            R_demo(1:3,1:3,j)*[0; lency; 0] R_demo(1:3,1:3,j)*[0; 0; lencz] ...
            R_demo(1:3,1:3,j)*[lencx; 0; lencz] R_demo(1:3,1:3,j)*[lencx; lency; lencz] ...
            R_demo(1:3,1:3,j)*[0; lency; lencz] ]';
        vm=vm(:,[a b c]);
        if j==1
            patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',1.0,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30,'EdgeColor',[0.15 0.15 0.90],'LineWidth',1.00);
            patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',1.0,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.85,'EdgeColor',[0.15 0.15 0.90],'LineWidth',0.75);
            %elseif j==N && strcmp(paper_experiment_type,'initial_pose')
        else
            patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.30,'EdgeColor',[0.15 0.15 0.90],'LineWidth',1.00);
            patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',0.8,'FaceColor',[0.25 0.80 0.80],'FaceAlpha',0.85,'EdgeColor',[0.15 0.15 0.90],'LineWidth',0.75);
        end
    end
end

for m=1:M
    R_recon = R_recon_struct{m};
    p_obj_recon = p_obj_recon_struct{m};
    % Draw cube reconstruction
    if show_rotation
        for j=round(linspace(1,N,inc))
            fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
            vm = repmat(p_obj_recon(j,:)+offset',8,1) + [ zeros(3,1) ...
                R_recon(1:3,1:3,j)*[lencx; 0; 0] R_recon(1:3,1:3,j)*[lencx; lency; 0] ...
                R_recon(1:3,1:3,j)*[0; lency; 0] R_recon(1:3,1:3,j)*[0; 0; lencz] ...
                R_recon(1:3,1:3,j)*[lencx; 0; lencz] R_recon(1:3,1:3,j)*[lencx; lency; lencz] ...
                R_recon(1:3,1:3,j)*[0; lency; lencz] ]';
            vm=vm(:,[a b c]);
            if j==1 && ~strcmp(paper_experiment_type,'target_pose')
                %             patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',1.0,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.30,'EdgeColor',[0.90 0.15 0.15],'LineWidth',1.00);
                %             patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.40,'EdgeColor',[0.90 0.15 0.15],'LineWidth',0.75);
                patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.10,'EdgeColor',[0.90 0.15 0.15],'LineWidth',1.00);
                patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.40,'EdgeColor',[0.90 0.15 0.15],'LineWidth',0.75);
            elseif j==N && strcmp(paper_experiment_type,'initial_pose')
            elseif j~=1
                patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.10,'EdgeColor',[0.90 0.15 0.15],'LineWidth',1.00);
                patch('Vertices',vm([2 3 7 6],:),'Faces',[1 2 3 4], 'EdgeAlpha',0.8,'FaceColor',[0.80 0.25 0.25],'FaceAlpha',0.40,'EdgeColor',[0.90 0.15 0.15],'LineWidth',0.75);
            end
        end
    end
    
    % Plot reconstruction
    if ~isempty(R_recon)
        if ~isempty(constraints_struct) && strcmp(paper_experiment_type,'robot')
            plot3(offset(a)+p_obj_recon(:,a),offset(b)+p_obj_recon(:,b),offset(c)+p_obj_recon(:,c),'r','linewidth',2,'LineStyle','--');
        else
            plot3(offset(a)+p_obj_recon(:,a),offset(b)+p_obj_recon(:,b),offset(c)+p_obj_recon(:,c),'r','linewidth',2,'LineStyle',linestyles{m});
        end
    end
    
    if strcmp(paper_experiment_type,'target_pose')  || strcmp(paper_experiment_type,'timevsgeom') %|| strcmp(paper_experiment_type,'jerkaccuracy') %||  strcmp(paper_experiment_type,'dmp')
        if ~isempty(R_recon) && sum(p_demo(end,:)-p_obj_recon(end,[a b c]))~=0
            arrow3(p_demo(end,:),p_obj_recon(end,[a b c]),['_k1.5'],1.5);
        end
    end
    
    if strcmp(paper_experiment_type,'initial_pose')
        if ~isempty(R_recon) && sum(p_demo(1,:)-p_obj_recon(1,[a b c]))~=0
            arrow3(p_demo(1,:),p_obj_recon(1,[a b c]),['_k1.5'],1.5);
        end
    end
    
    if strcmp(paper_experiment_type,'adaptive')
        if ~isempty(R_recon) && sum(p_demo(end,:)-p_obj_recon(end,[a b c]))~=0
            arrow3(p_demo(end,:),p_obj_recon(end,[a b c]),['_k1.5'],1.5);
        end
        plot3(offset(3)+p_obj_recon(round(N/2),a),offset(2)+p_obj_recon(round(N/2),b),offset(1)+p_obj_recon(round(N/2),c),'k.','MarkerSize',50);
    end
    
    if strcmp(paper_experiment_type,'target_direction')
        R_FS_end = constraints_struct{m}.end_FS_frame;
        if ~isempty(R_recon)
            arrow3(p_obj_recon(end,[a b c]),p_obj_recon(end,[a b c])+1*len*R_FS_end([a b c],1)',['_k1.5'],1.5);
        end
    end
    
end

% Plot demonstration
plot3(p_obj_demo(:,a),p_obj_demo(:,b),p_obj_demo(:,c),'b-','linewidth',1.5);

% Plot obstacle
for j=1:M
    if ~isempty(constraints_struct) && isfield(constraints_struct{j},'obstacle')
        r = constraints_struct{j}.obstacle.safety_radius;
        [x,y,z] = sphere(50);
        x0 = constraints_struct{j}.obstacle.location(a); y0 = constraints_struct{j}.obstacle.location(b); z0 = constraints_struct{j}.obstacle.location(c);
        x = x*r + x0;
        y = y*r + y0;
        z = z*r + z0;
        
        lightGrey = [34,139,34]/255; %0.8*[1 1 1]; % It looks better if the lines are lighter
        surface(x,y,z,'FaceColor', lightGrey,'facealpha',0.75,'EdgeColor','none')
    end
end

if ~isempty(q)
    for i=round(linspace(1,N-1,3))
        T_ee =  forwardkin_UR10(q(i,:));
        
        b_T_oo = eye(4);
        b_T_o1  = DH_T(q(i,1),0.1273,0,-1.5707963);
        o1_T_o2 = DH_T(q(i,2),0,0.6127,0);
        o2_T_o3 = DH_T(q(i,3),0,0.5716,0);
        o3_T_o4 = DH_T(q(i,4),0.1639,0,-1.5707963);
        o4_T_o5 = DH_T(q(i,5),0.1157,0,1.5707963);
        o5_T_ee = DH_T(q(i,6),0.0922,0,0);
        b_T_ee = b_T_o1*o1_T_o2*o2_T_o3*o3_T_o4*o4_T_o5*o5_T_ee;
        
        b_T_o2 = b_T_o1*o1_T_o2;
        b_T_o3 = b_T_o1*o1_T_o2*o2_T_o3;
        b_T_o4 = b_T_o1*o1_T_o2*o2_T_o3*o3_T_o4;
        b_T_o5 = b_T_o1*o1_T_o2*o2_T_o3*o3_T_o4*o4_T_o5;
        
        px = [0; b_T_o1(a,4) ;  b_T_o2(a,4) ; b_T_o3(a,4)  ;b_T_o4(a,4)  ;b_T_o5(a,4)    ;b_T_ee(a,4) ] ;
        py = [0; b_T_o1(b,4) ;  b_T_o2(b,4) ; b_T_o3(b,4)  ;b_T_o4(b,4)  ;b_T_o5(b,4)    ;b_T_ee(b,4) ] ;
        pz = [0; b_T_o1(c,4) ;  b_T_o2(c,4) ; b_T_o3(c,4)  ;b_T_o4(c,4)  ;b_T_o5(c,4)    ;b_T_ee(c,4) ] ;
        
        plot3(px,py,pz,'k.-','linewidth',5,'MarkerSize',15);
        plot3(px(end),py(end),pz(end),'k.-','MarkerSize',25);
        
        
        cyl_len = 0.025;
        
        nb_points = 20;
        cyl_r = [0 repmat(0.025,1,nb_points) 0];
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_oo([a b c],4)'-cyl_len*b_T_oo([a b c],3)',b_T_oo([a b c],4)'+cyl_len*b_T_oo([a b c],3)');
        surf(X,Y,Z,'EdgeAlpha',0.3,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o1([a b c],4)'-cyl_len*b_T_o1([a b c],3)',b_T_o1([a b c],4)'+cyl_len*b_T_o1([a b c],3)');
        surf(X,Y,Z,'EdgeAlpha',0.3,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o2([a b c],4)'-cyl_len*b_T_o2([a b c],3)',b_T_o2([a b c],4)'+cyl_len*b_T_o2([a b c],3)');
        surf(X,Y,Z,'EdgeAlpha',0.3,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o3([a b c],4)'-cyl_len*b_T_o3([a b c],3)',b_T_o3([a b c],4)'+cyl_len*b_T_o3([a b c],3)');
        surf(X,Y,Z,'EdgeAlpha',0.3,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o4([a b c],4)'-cyl_len*b_T_o4([a b c],3)',b_T_o4([a b c],4)'+cyl_len*b_T_o4([a b c],3)');
        surf(X,Y,Z,'EdgeAlpha',0.3,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o5([a b c],4)'-cyl_len*b_T_o5([a b c],3)',b_T_o5([a b c],4)'+cyl_len*b_T_o5([a b c],3)');
        surf(X,Y,Z,'EdgeAlpha',0.3,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(0.5*cyl_r,nb_points,b_T_ee([a b c],4)'-0.5*cyl_len*b_T_ee([a b c],3)',b_T_ee([a b c],4)'+0.5*cyl_len*b_T_ee([a b c],3)');
        surf(X,Y,Z,'EdgeAlpha',0.3,'FaceColor',[0.8 0.0 0.0],'FaceLighting','phong');
        
        %arrow3(b_T_ee(1:3,4)',b_T_ee(1:3,4)'+2*len*b_T_ee(1:3,1)','r',2*t)
        %arrow3(b_T_ee(1:3,4)',b_T_ee(1:3,4)'+2*len*b_T_ee(1:3,2)','e',2*t)
        %arrow3(b_T_ee(1:3,4)',b_T_ee(1:3,4)'+2*len*b_T_ee(1:3,3)','b',2*t)
        
        %sphere
        %[x, y, z] = sphere(50);
        %r = 1.4;
        %surf(r*x,r*y,r*z,'EdgeAlpha',0.0,'FaceColor',[0.2 0.2 0.2],'FaceAlpha',0.3,'FaceLighting','phong');
    end
end

%title(titel)
view(view_angles(1),view_angles(2));

%grid on;
%box on;
axis equal;
%zoom(0.75)
end

function T = DH_T(theta,d,a,alfa)
T = zeros(4,4);
T(1,1) = cos(theta);T(1,2) = -cos(alfa)*sin(theta);T(1,3) = sin(alfa)*sin(theta);T(1,4) = a*cos(theta);
T(2,1) = sin(theta);T(2,2) = cos(alfa)*cos(theta);T(2,3) = -sin(alfa)*cos(theta);T(2,4) = a*sin(theta);
T(3,1) = 0;T(3,2) = sin(alfa);T(3,3) = cos(alfa);T(3,4) = d;
T(4,1) = 0;T(4,2) = 0;T(4,3)= 0;T(4,4) = 1    ;
end
