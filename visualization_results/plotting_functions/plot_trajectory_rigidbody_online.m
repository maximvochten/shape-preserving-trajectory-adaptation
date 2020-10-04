function plot_trajectory_rigidbody_online(time_meas,T_obj_online,T_obj,index,titel,save_frames,i)

pose_coordinates_online = poseMatrix2poseCoordinates(T_obj_online);
pose_coordinates = poseMatrix2poseCoordinates(T_obj);

%==========
%xyz
%==========

h=figure(10);
movegui(h,'northeast')
clf
subplot(6,1,1);
hold on
plot(time_meas,pose_coordinates(:,1),'b','linewidth',1);
plot(time_meas,pose_coordinates_online(:,1),'r','linewidth',1);
%plot(time_meas(1:index),pose_coordinates(1:index,1),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('x[mm]')
subplot(6,1,2);
hold on
plot(time_meas,pose_coordinates(:,2),'b','linewidth',1);
plot(time_meas,pose_coordinates_online(:,2),'r','linewidth',1);
%plot(time_meas(1:index),pose_coordinates(1:index,2),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('y[mm]')
subplot(6,1,3);
hold on
plot(time_meas,pose_coordinates(:,3),'b','linewidth',1);
plot(time_meas,pose_coordinates_online(:,3),'r','linewidth',1);
%plot(time_meas(1:index),pose_coordinates(1:index,3),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('z[mm]')
subplot(6,1,4);
hold on
plot(time_meas,pose_coordinates(:,4),'b','linewidth',1);
plot(time_meas,pose_coordinates_online(:,4),'r','linewidth',1);
%plot(time_meas(1:index),pose_coordinates(1:index,4),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('roll[rad]')
subplot(6,1,5);
hold on
plot(time_meas,pose_coordinates(:,5),'b','linewidth',1);
plot(time_meas,pose_coordinates_online(:,5),'r','linewidth',1);
%plot(time_meas(1:index),pose_coordinates(1:index,5),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('pitch[rad]')
subplot(6,1,6);
hold on
plot(time_meas,pose_coordinates(:,6),'b','linewidth',1);
plot(time_meas,pose_coordinates_online(:,6),'r','linewidth',1);
%plot(time_meas(1:index),pose_coordinates(1:index,6),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('yaw[rad]')

suptitle('Pose coordinates')

%==========
%3D
%==========



h2=figure(11);
movegui(h2,'northwest')
clf
hold on
    grid on;
box on;
axis equal;
xlabel('z[mm]')
ylabel('y[mm]')
zlabel('x[mm]')

plot3(pose_coordinates(:,1),pose_coordinates(:,2),pose_coordinates(:,3),'b','linewidth',1);
plot3(pose_coordinates_online(:,1),pose_coordinates_online(:,2),pose_coordinates_online(:,3),'r','linewidth',1);
plot3(pose_coordinates(1:index,1),pose_coordinates(1:index,2),pose_coordinates(1:index,3),'b.');


p = zeros(size(pose_coordinates,1),3);
Rx = zeros(size(pose_coordinates,1),3);
Ry = zeros(size(pose_coordinates,1),3);
Rz = zeros(size(pose_coordinates,1),3);

for j=1:20:size(T_obj,3)
    Rx(j,:) = T_obj_online(1:3,1,j);
    Ry(j,:) = T_obj_online(1:3,2,j);
    Rz(j,:) = T_obj_online(1:3,3,j);
    p(j,:) = T_obj_online(1:3,4,j);
end

    arrow3(p,p+30.0*Rx,'r',0.5)   %n x 3 matrix
    arrow3(p,p+30.0*Ry,'e',0.5)
    arrow3(p,p+30.0*Rz,'b',0.5)

    

% titel = 'rigid body plot note refpoint';
% title(titel)
% % 

%view(-36,30)
%view(125,34); %tas uitgieten 2
%view(-109,4); %tas wegzetten
%view(-57,34); %tas wegzetten 48
view(79,30); %tas uitgieten
view(-29,46);

if(save_frames && mod(i,2)==0)
    %make sure that directory 'figures' exists!
    save2pdf(['figures/plot_scheppeneten_3D_' num2str(i/2-1) '.pdf'],h2)
    save2pdf(['figures/plot_scheppeneten_pose_' num2str(i/2-1) '.pdf'],h)
end