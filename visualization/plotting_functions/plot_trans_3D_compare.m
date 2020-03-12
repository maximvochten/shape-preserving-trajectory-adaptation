function plot_trans_3D_compare(position_init,positions_new,titel,view_angles,dmp)


%h2=figure(11);
h2 = figure;
set(gcf,'units','normalized','outerposition',[0 0 0.6 0.7]);

%movegui(h2,'northwest')
clf
hold on
grid on;
%box on;
axis equal;
xlabel('$x$[mm]','Interpreter','LaTex','FontSize',18)
ylabel('$y$[mm]','Interpreter','LaTex','FontSize',18)
zlabel('$z$[mm]','Interpreter','LaTex','FontSize',18)

a=1; %xcoordinate
b=2;
c=3;

% Plot demonstration
plot3(position_init(:,a),position_init(:,b),position_init(:,c),'b','linewidth',1.5);
plot3(position_init(end,a),position_init(end,b),position_init(end,c),'b*','MarkerSize',7,'LineWidth',2);

if dmp
    colorspec = {[0.6 0.3 0.3]; [0.7 0.2 0.2]; [0.9 0.4 0.4]; ...
  [0.5 0.1 0.1]};
for i=1:size(positions_new,2)
    plot3(positions_new(i).Data(a,:),positions_new(i).Data(b,:),positions_new(i).Data(c,:),'-.', 'Color', colorspec{i},'linewidth',1.5);
    plot3(positions_new(i).Data(a,end),positions_new(i).Data(b,end),positions_new(i).Data(c,end),'*', 'Color', colorspec{i},'MarkerSize',7,'LineWidth',2);
end
    
else
    colorspec = {[0.6 0.3 0.3]; [0.7 0.2 0.2]; [0.9 0.4 0.4]; ...
  [0.5 0.1 0.1]};
    for i=1:size(positions_new,3)
        
    plot3(positions_new(a,:,i),positions_new(b,:,i),positions_new(c,:,i),'-.', 'Color', colorspec{i},'linewidth',1.5);
    plot3(positions_new(a,end,i),positions_new(b,end,i),positions_new(c,end,i),'*', 'Color', colorspec{i},'MarkerSize',7,'LineWidth',2);
    end
    
end
% Plot reconstruction
%plot3(T_recon(1,4),T_recon(2,4),T_recon(3,4),'r','linewidth',1.5);

%title(titel)
view(view_angles(1),view_angles(2));

%grid on;
%box on;
axis equal;
%zoom(0.9)

