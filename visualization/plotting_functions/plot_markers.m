function plot_markers(markerdata_1,markerdata_2,titletext)
% Plot marker coordinates

figure;
hold on
axis equal; grid on; box on;
view(30,30)
linewidth = 1;
plot3(markerdata_2(:,1:3:end),markerdata_2(:,2:3:end),markerdata_2(:,3:3:end),'r','LineWidth',linewidth);
plot3(markerdata_1(:,1:3:end),markerdata_1(:,2:3:end),markerdata_1(:,3:3:end),'b.-','LineWidth',linewidth,'MarkerSize',5);
title(titletext);