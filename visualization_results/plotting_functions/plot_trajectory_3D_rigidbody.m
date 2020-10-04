function plot_trajectory_3D_rigidbody(T_obj,titel)
%make a 3D plot and a XYZ plot of the given coordinates

%pose_coordinates xyz rpy
x = reshape(T_obj(1,4,:),[],1);
y = reshape(T_obj(2,4,:),[],1);
z = reshape(T_obj(3,4,:),[],1);

figure
hold on
plot3(x,y,z,'b','linewidth',2);
p = [x y z];

title(titel)
axis equal
grid on;
box on;

% for i=1:20:size(T_obj,3)
%     R = T_obj(1:3,1:3,i);
%     
%     arrow3(p(i,:),p(i,:)+30.0*R(1:3,1)','r',0.5)
%     arrow3(p(i,:),p(i,:)+30.0*R(1:3,2)','e',0.5)
%     arrow3(p(i,:),p(i,:)+30.0*R(1:3,3)','b',0.5)
% end

view(-36,30)



