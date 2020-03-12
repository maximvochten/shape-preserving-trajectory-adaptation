function plot_trajectory_3D_xyz_online(time_meas,xcoordinates,ycoordinates,zcoordinates,index,title)

h=figure(10);
movegui(h,'east')
clf
ax1 = subplot(3,1,1);
hold on
plot(time_meas,xcoordinates(:,1),'b','linewidth',1);
plot(time_meas,xcoordinates(:,2),'r','linewidth',1);
plot(time_meas(1:index),xcoordinates(1:index,1),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('x')
ax2 = subplot(3,1,2);
hold on
plot(time_meas,ycoordinates(:,1),'b','linewidth',1);
plot(time_meas,ycoordinates(:,2),'r','linewidth',1);
plot(time_meas(1:index),ycoordinates(1:index,1),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('y')
ax3 = subplot(3,1,3);
hold on
plot(time_meas,zcoordinates(:,1),'b','linewidth',1);
plot(time_meas,zcoordinates(:,2),'r','linewidth',1);
plot(time_meas(1:index),zcoordinates(1:index,1),'b.');
yL = get(gca,'YLim');
line([time_meas(index) time_meas(index)],yL,'Color','r');
ylabel('z')
suptitle(title)
        
h2=figure(11);
movegui(h2,'west')
clf
hold on
plot3(zcoordinates(:,1),ycoordinates(:,1),xcoordinates(:,1),'b','linewidth',1);
plot3(zcoordinates(:,2),ycoordinates(:,2),xcoordinates(:,2),'r','linewidth',1);
plot3(zcoordinates(1:index,2),ycoordinates(1:index,2),xcoordinates(1:index,2),'b.');
grid on;
box on;
axis equal;
view(125,34); %tas uitgieten 2
%view(-109,4); %tas wegzetten
%view(-57,34); %tas wegzetten 48
view(214,50); %tas uitgieten
xlabel('z')
ylabel('y')
zlabel('x')

        %         if(save_frames)
        %             %make sure that directory 'figures' exists!
        %             save2pdf(['figures/plot_model_online_good_3D_' num2str(i) '.pdf'],h2)
        %             save2pdf(['figures/plot_model_online_good_xyz_' num2str(i) '.pdf'],h)
        %             save2pdf(['figures/plot_model_online_good_invariants_' num2str(i) '.pdf'],h_invariants)
        %         end