function plot_RBT_3D_compare_robotmodel(T_demo,T_recon,titel,view_angles,adapt,halflength_position_coordinates,q,showdemo,plotarrows)

if nargin < 5
    adapt=0;
    plotarrows = 1;
end

pose_coordinates_demo = poseMatrix2poseCoordinates(T_demo);
if ~isempty(T_recon)
    pose_coordinates_recon = poseMatrix2poseCoordinates(T_recon);
end

%h2=figure(11);
h2 = figure;

set(gcf,'units','normalized','outerposition',[0 0 1.0 1.0]);
N = size(T_demo,3);
Nr = size(T_recon,3);
teller=0;
for i=1:1:Nr
    
    movegui(h2,'northwest')
    clf
    hold on
    grid on;
    %box on;
    axis equal
    %axis([-0.75 1.4 -0.2 1.4 -0.2 1.4])
    
    xlabel('$x$[m]','Interpreter','LaTex','FontSize',18)
    ylabel('$y$[m]','Interpreter','LaTex','FontSize',18)
    zlabel('$z$[m]','Interpreter','LaTex','FontSize',18)
    
    
    
    
    inc = 4; % how many rigid bodies drawn along trajectory - normal
    inc = 8; % how many rigid bodies drawn along trajectory - sine motion / hammering
    len = 2*44.8/1000; % length of arrow
    lencx = 51.2/1000; % length of cube
    lency = 32/1000; % length of cube
    lencz = 32/1000; % length of cube
    t = 0.453; % thickness arrow - sine motion
    % t = 0.853; % thickness arrow - normal
    a=1; %xcoordinate
    b=2;
    c=3;
    
    offset = 0*[0 ;  0 ; -100]; % for change in starting orientation
    % Draw orientation rigid body demonstration
    if showdemo
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
        if plotarrows
            arrow3(p_demo,p_demo+len*Rx_demo,'_r',t)
            arrow3(p_demo,p_demo+len*Ry_demo,'_e',t)
            arrow3(p_demo,p_demo+len*Rz_demo,'_b',t)
        end
    end
    
    if ~isempty(T_recon)
        % Draw orientation rigid body reconstruction
        Rx_recon = []; Ry_recon = []; Rz_recon = []; p_recon = [];
        for j=round(linspace(1,Nr,inc))
            Rx_recon = [Rx_recon ; T_recon(1:3,1,j)'];
            Ry_recon = [Ry_recon ; T_recon(1:3,2,j)'];
            Rz_recon = [Rz_recon ; T_recon(1:3,3,j)'];
            p_recon = [p_recon ; T_recon(1:3,4,j)'+ offset'];
            if j==Nr
                plot3(offset(3)+pose_coordinates_recon(j,a),offset(2)+pose_coordinates_recon(j,b),offset(1)+pose_coordinates_recon(j,c),'r*','MarkerSize',7,'LineWidth',2);
            else
                plot3(offset(3)+pose_coordinates_recon(j,a),offset(2)+pose_coordinates_recon(j,b),offset(1)+pose_coordinates_recon(j,c),'r.','MarkerSize',15);
            end
        end
        p_recon=p_recon(:,[a b c]);
        Rx_recon=Rx_recon(:,[a b c]);
        Ry_recon=Ry_recon(:,[a b c]);
        Rz_recon=Rz_recon(:,[a b c]);
        if plotarrows
            arrow3(p_recon,p_recon+len*Rx_recon,'_r',t)
            arrow3(p_recon,p_recon+len*Ry_recon,'_e',t)
            arrow3(p_recon,p_recon+len*Rz_recon,'_b',t)
        end
    end
    
    if showdemo
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
    end
    
    if ~isempty(T_recon)
        % Draw cube reconstruction
        for j=round(linspace(1,Nr,inc))
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
    
    if showdemo
        % Plot demonstration
        plot3(pose_coordinates_demo(:,a),pose_coordinates_demo(:,b),pose_coordinates_demo(:,c),'b-.','linewidth',1.5);
    end
    if ~isempty(T_recon)
        plot3(offset(3)+pose_coordinates_recon(:,a),offset(2)+pose_coordinates_recon(:,b),offset(1)+pose_coordinates_recon(:,c),'r','linewidth',1.5);
    end
    % Plot reconstruction
    %plot3(T_recon(1,4),T_recon(2,4),T_recon(3,4),'r','linewidth',1.5);
    
    
    
    fm = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    vm = repmat(T_recon(1:3,4,i)'+offset',8,1) + [ zeros(3,1) ...
        T_recon(1:3,1:3,i)*[lencx; 0; 0] T_recon(1:3,1:3,i)*[lencx; lency; 0] ...
        T_recon(1:3,1:3,i)*[0; lency; 0] T_recon(1:3,1:3,i)*[0; 0; lencz] ...
        T_recon(1:3,1:3,i)*[lencx; 0; lencz] T_recon(1:3,1:3,i)*[lencx; lency; lencz] ...
        T_recon(1:3,1:3,i)*[0; lency; lencz] ]';
    vm=vm(:,[a b c]);
    
    patch('Vertices',vm,'Faces',fm, 'EdgeAlpha',1.0,'FaceColor',[0.25 0.25 0.25],'FaceAlpha',0.05);
    
    
    
    if adapt
        plot3(halflength_position_coordinates(a),halflength_position_coordinates(b),halflength_position_coordinates(c),'k.','MarkerSize',35,'LineWidth',2);
        
        
        enddemo = pose_coordinates_recon(end,[a b c]);
        endrecon = pose_coordinates_demo(end,[a b c]);
        if plotarrows
            arrow3(endrecon,enddemo,'2k',2*t)
        end
    end
    
    show_robot=1;
    if show_robot && i in 1:
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
        
        px = [0; b_T_o1(1,4) ;  b_T_o2(1,4) ; b_T_o3(1,4)  ;b_T_o4(1,4)  ;b_T_o5(1,4)    ;b_T_ee(1,4) ] ;
        py = [0; b_T_o1(2,4) ;  b_T_o2(2,4) ; b_T_o3(2,4)  ;b_T_o4(2,4)  ;b_T_o5(2,4)    ;b_T_ee(2,4) ] ;
        pz = [0; b_T_o1(3,4) ;  b_T_o2(3,4) ; b_T_o3(3,4)  ;b_T_o4(3,4)  ;b_T_o5(3,4)    ;b_T_ee(3,4) ] ;
        
        plot3(px,py,pz,'k.-','linewidth',5,'MarkerSize',15);
        plot3(px(end),py(end),pz(end),'k.-','MarkerSize',25);
        
        
        cyl_len = 0.025;
        
        nb_points = 20;
        cyl_r = [0 repmat(0.025,1,nb_points) 0];
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_oo(1:3,4)'-cyl_len*b_T_oo(1:3,3)',b_T_oo(1:3,4)'+cyl_len*b_T_oo(1:3,3)');
        surf(X,Y,Z,'EdgeAlpha',0.5,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o1(1:3,4)'-cyl_len*b_T_o1(1:3,3)',b_T_o1(1:3,4)'+cyl_len*b_T_o1(1:3,3)');
        surf(X,Y,Z,'EdgeAlpha',0.5,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o2(1:3,4)'-cyl_len*b_T_o2(1:3,3)',b_T_o2(1:3,4)'+cyl_len*b_T_o2(1:3,3)');
        surf(X,Y,Z,'EdgeAlpha',0.5,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o3(1:3,4)'-cyl_len*b_T_o3(1:3,3)',b_T_o3(1:3,4)'+cyl_len*b_T_o3(1:3,3)');
        surf(X,Y,Z,'EdgeAlpha',0.5,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o4(1:3,4)'-cyl_len*b_T_o4(1:3,3)',b_T_o4(1:3,4)'+cyl_len*b_T_o4(1:3,3)');
        surf(X,Y,Z,'EdgeAlpha',0.5,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(cyl_r,nb_points,b_T_o5(1:3,4)'-cyl_len*b_T_o5(1:3,3)',b_T_o5(1:3,4)'+cyl_len*b_T_o5(1:3,3)');
        surf(X,Y,Z,'EdgeAlpha',0.5,'FaceColor',[0.0 0.8 0.8],'FaceLighting','phong');
        [X,Y,Z] = cylinder2P(0.5*cyl_r,nb_points,b_T_ee(1:3,4)'-0.5*cyl_len*b_T_ee(1:3,3)',b_T_ee(1:3,4)'+0.5*cyl_len*b_T_ee(1:3,3)');
        surf(X,Y,Z,'EdgeAlpha',0.5,'FaceColor',[0.8 0.0 0.0],'FaceLighting','phong');
        
        %arrow3(b_T_ee(1:3,4)',b_T_ee(1:3,4)'+2*len*b_T_ee(1:3,1)','r',2*t)
        %arrow3(b_T_ee(1:3,4)',b_T_ee(1:3,4)'+2*len*b_T_ee(1:3,2)','e',2*t)
        %arrow3(b_T_ee(1:3,4)',b_T_ee(1:3,4)'+2*len*b_T_ee(1:3,3)','b',2*t)
        
        %sphere
        %[x y z] = sphere(50);
        %r = 1.4;
        %surf(r*x,r*y,r*z,'EdgeAlpha',0.0,'FaceColor',[0.2 0.2 0.2],'FaceAlpha',0.3,'FaceLighting','phong');
        
    end
    %title(titel)
    view(view_angles(1),view_angles(2));
    
    %grid on;
    %box on;
    axis equal;
    zoom(0.70)
    
    save_frames =1;
    if(save_frames)
        %make sure that directory 'figures' exists!
        %set(gcf, 'color', 'none');
        %set(gca,'color','none') ;
        %export_fig 'figures/new_start_rot_3D.png'  -transparent -m2.5
        %export_fig(['figures/new_' num2str(teller) '.png'])
        
        export_fig(['figures/pouring2_' num2str(teller) '.png'],'-m1.5')
        %save2pdf(['figures/sinus_' num2str(teller) '.pdf'],h2)
    end
    teller=teller+1;
end
end
function T = DH_T(theta,d,a,alfa)

T = zeros(4,4);
T(1,1) = cos(theta);T(1,2) = -cos(alfa)*sin(theta);T(1,3) = sin(alfa)*sin(theta);T(1,4) = a*cos(theta);
T(2,1) = sin(theta);T(2,2) = cos(alfa)*cos(theta);T(2,3) = -sin(alfa)*cos(theta);T(2,4) = a*sin(theta);
T(3,1) = 0;T(3,2) = sin(alfa);T(3,3) = cos(alfa);T(3,4) = d;
T(4,1) = 0;T(4,2) = 0;T(4,3)= 0;T(4,4) = 1    ;

end


