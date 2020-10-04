function plot_trajectory_3D_xyz(time_online_meas,xcoordinates,ycoordinates,zcoordinates,title)
%make a 3D plot and a XYZ plot of the given coordinates

nb_of_plots = size(xcoordinates,2);
colors = {'b','r','k','g','y',[.5 .6 .7],[.8 .2 .6]};

figure
for i=1:nb_of_plots
    hold on
    plot3(xcoordinates(:,i),ycoordinates(:,i),zcoordinates(:,i),'color',colors{i},'linewidth',3);
end
grid on;
box on;

figure
for i=1:nb_of_plots
    subplot(3,1,1)
    hold on
    plot(time_online_meas,xcoordinates(:,i),'color',colors{i},'linewidth',1);
    ylabel('x')
    subplot(3,1,2)
    hold on
    plot(time_online_meas,ycoordinates(:,i),'color',colors{i},'linewidth',1);
    ylabel('y')
    subplot(3,1,3)
    hold on
    plot(time_online_meas,zcoordinates(:,i),'color',colors{i},'linewidth',1);
    ylabel('z')
end
suptitle(title)