function  plot_robot( q )
%PLOT_ROBOT Summary of this function goes here
%   Detailed explanation goes here
%h2=figure(11);

h2 = figure;

set(gcf,'units','normalized','outerposition',[0 0 0.6 0.7]);


%movegui(h2,'northwest')
clf
hold on
grid on;
%box on;
axis equal;

xlabel('$x$[m]','Interpreter','LaTex','FontSize',18)
ylabel('$y$[m]','Interpreter','LaTex','FontSize',18)
zlabel('$z$[m]','Interpreter','LaTex','FontSize',18)


inc=15;

for i = 1:inc:length(q)

    T_ee =  forwardkin_UR10(q(i,:));
    
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
    
    px = [0; b_T_o1(1,4) ;  b_T_o2(1,4) ; b_T_o3(1,4)  ;b_T_o4(1,4)  ;b_T_o5(1,4) ;b_T_ee(1,4) ] ;
    py = [0; b_T_o1(2,4) ;  b_T_o2(2,4) ; b_T_o3(2,4)  ;b_T_o4(2,4)  ;b_T_o5(2,4)    ;b_T_ee(2,4) ] ;
    pz = [0; b_T_o1(3,4) ;  b_T_o2(3,4) ; b_T_o3(3,4)  ;b_T_o4(3,4)  ;b_T_o5(3,4)    ;b_T_ee(3,4) ] ; 
    
    plot3(px,py,pz,'b.-','MarkerSize',15);
    plot3(px(end),py(end),pz(end),'r.-','MarkerSize',15);
    %sphere
    [x y z] = sphere(128);
    surf(1.4*x,1.4*y,1.4*z,'EdgeAlpha',0.0,'FaceColor',[0.2 0.2 0.2],'FaceAlpha',0.3,'FaceLighting','phong');


end
axis equal
end
function T = DH_T(theta,d,a,alfa)

T = zeros(4,4);
T(1,1) = cos(theta);T(1,2) = -cos(alfa)*sin(theta);T(1,3) = sin(alfa)*sin(theta);T(1,4) = a*cos(theta);
T(2,1) = sin(theta);T(2,2) = cos(alfa)*cos(theta);T(2,3) = -sin(alfa)*cos(theta);T(2,4) = a*sin(theta);
T(3,1) = 0;T(3,2) = sin(alfa);T(3,3) = cos(alfa);T(3,4) = d;
T(4,1) = 0;T(4,2) = 0;T(4,3)= 0;T(4,4) = 1    ;

end


