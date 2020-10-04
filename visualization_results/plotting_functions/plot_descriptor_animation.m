function plot_descriptor_animation(data1,data2,titel,h,parametrization,descriptor_type)
% Can plot many types of trajectory descriptors

labelfontsize_x = 14;
labelfontsize_y = 18;
axisfontsize = 12;
if isempty(data2)
    M = 0;
    N2 = 0;
else
    N2 = size(data2,1);
    M = 1;
end
if isempty(data1)
    M1 = 0;
    N1 = 0;
else
    N1 = length(data1);
    M1 = 1;
    
end
N = max(N2,N1);

switch parametrization
    case 'timebased'
        switch descriptor_type
            case 'pose'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\alpha_z[rad]$','$\beta_y[rad]$','$\gamma_x[rad]$','$p_x[m]$','$p_y[m]$','$p_z[m]$'};
            case 'pose_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'screw_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$i_{r1}[\frac{rad}{s}]$','$i_{r2}[\frac{rad}{s}]$','$i_{r3}[\frac{rad}{s}]$','$i_{t1}[\frac{m}{s}]$','$i_{t2}[\frac{rad}{s}]$','$i_{t3}[\frac{rad}{s}]$'};
            case 'screw_axis'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_1[\frac{rad}{s}]$','$\omega_2[\frac{rad}{s}]$','$\omega_3[\frac{rad}{s}]$','$v_1[\frac{m}{s}]$','$v_2[\frac{m}{s}]$','$v_3[\frac{m}{s}]$'};
            case 'jerkaccuracy'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\ddot{\omega}_x[\frac{rad}{s^3}]$','$\ddot{\omega}_y[\frac{rad}{s^3}]$','$\ddot{\omega}_z[\frac{rad}{s^3}]$','$\ddot{v}_x[\frac{m}{s^3}]$','$\ddot{v}_y[\frac{m}{s^3}]$','$\ddot{v}_z[\frac{m}{s^3}]$'};
        end
        
    case 'timebased_contour'
        switch descriptor_type
            case 'pose_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'screw_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$i_{r1}[\frac{rad}{s}]$','$i_{r2}[\frac{rad}{s}]$','$i_{r3}[\frac{rad}{s}]$','$i_{t1}[\frac{m}{s}]$','$i_{t2}[\frac{rad}{s}]$','$i_{t3}[\frac{rad}{s}]$'};
            case 'screw_axis'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_1[\frac{rad}{s}]$','$\omega_2[\frac{rad}{s}]$','$\omega_3[\frac{rad}{s}]$','$v_1[\frac{m}{s}]$','$v_2[\frac{m}{s}]$','$v_3[\frac{m}{s}]$'};
            case 'jerkaccuracy'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\ddot{\omega}_x[\frac{rad}{s^3}]$','$\ddot{\omega}_y[\frac{rad}{s^3}]$','$\ddot{\omega}_z[\frac{rad}{s^3}]$','$\ddot{v}_x[\frac{m}{s^3}]$','$\ddot{v}_y[\frac{m}{s^3}]$','$\ddot{v}_z[\frac{m}{s^3}]$'};
        end
        
    case 'timebased_o1'
        switch descriptor_type
            case 'screw_axis'
                xvector = [0; cumsum(data2(:,1)*h)];
                label_x = '$\Theta$[rad]';
                plotNames = {'$\omega_1[\frac{rad}{s}]$','$\omega_2[\frac{rad}{s}]$','$\omega_3[\frac{rad}{s}]$','$v_1[\frac{m}{s}]$','$v_2[\frac{m}{s}]$','$v_3[\frac{m}{s}]$'};
        end
    case 'geometric_o1'
        switch descriptor_type
            case 'screw_axis'
                xvector = cumsum(data2(:,1)*h);
                data2 = cumsum(data2,1)*h;
                label_x = '$\Theta$[rad]';
                plotNames = {'$\Omega_1$[rad]','$\Omega_2$[rad]','$\Omega_3$[rad]','$V_1$[m]','$V_2$[m]','$V_3$[m]'};
        end
    case 'geometric'
        switch descriptor_type
            case 'pose'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\alpha_z[rad]$','$\beta_y[rad]$','$\gamma_x[rad]$','$p_x[m]$','$p_y[m]$','$p_z[m]$'};
            case 'pose_twist'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_x$[rad]','$\Omega_y$[rad]','$\Omega_z$[rad]','$V_x$[m]','$V_y$[m]','$V_z$[m]'};
            case 'screw_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'{$I_{r1}$[rad]}','{$I_{r2}$[rad]}','{$I_{r3}$[rad]}','{$I_{t1}$[m]}','{$I_{t2}$[rad]}','{$I_{t3}$[rad]}'};
            case 'screw_axis'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_1$[rad]','$\Omega_2$[rad]','$\Omega_3$[rad]','$V_1$[m]','$V_2$[m]','$V_3$[m]'};
        end
    case 'geometric_contour'
        switch descriptor_type
            case 'pose_twist'
                deltas = norm(data2(1,4:6));
                xvector = linspace(0,deltas,N);
                label_x = '$s$[m]';
                plotNames = {'$\Omega_x$[rad]','$\Omega_y$[rad]','$\Omega_z$[rad]','$V_x$[m]','$V_y$[m]','$V_z$[m]'};
                %             case 'screw_twist'
                %                 xvector = linspace(0,h*N,N);
                %                 label_x = '$t$[s]';
                %                 plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                deltas = abs(data2(1,4)/N);
                xvector = linspace(0,deltas*N,N);
                label_x = '$s$[m]';
                plotNames = {'{$I_{r1}$[rad]}','{$I_{r2}$[rad]}','{$I_{r3}$[rad]}','{$I_{t1}$[m]}','{$I_{t2}$[rad]}','{$I_{t3}$[rad]}'};
            case 'screw_axis'
                xvector = linspace(0,h*N,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_1$[rad]','$\Omega_2$[rad]','$\Omega_3$[rad]','$V_1$[m]','$V_2$[m]','$V_3$[m]'};
        end
    case 'dimensionless'
        switch descriptor_type
            case 'pose_twist'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_x$[-]','$\Omega_y$[-]','$\Omega_z$[-]','$V_x$[-]','$V_y$[-]','$V_z$[-]'};
            case 'screw_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$I_{r1}$[-]','$I_{r2}$[-]','$I_{r3}$[-]','$I_{t1}$[-]','$I_{t2}$[-]','$I_{t3}$[-]'};
            case 'screw_axis'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_1$[-]','$\Omega_2$[-]','$\Omega_3$[-]','$V_1$[-]','$V_2$[-]','$V_3$[-]'};
        end
end

%colorspec = {[1.0 0.0 0.0];  [0.8 0.2 0.2]; [0.7 0.2 0.2]; ...
%[0.6 0.2 0.2]};

linestyles = {'-','--',':','-.'};

% Create figure
figure1 = figure('Name',titel,'Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.5    0.2    0.5    0.7]);

% Create subplot
subplot1 = subplot(2,3,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot1,'on');
hold(subplot1,'all');
xmin = min(xvector);
xmax = max(xvector);
ymin = min(data2(:,1));
ymax = max(data2(:,1));
xlim([xmin xmax])
if round(ymin,3) < round(ymax,3)
    ylim([ymin ymax])
elseif round(ymin,3) == round(ymax,3)
    ylim([-ymin ymin])
end
% Create plot
% for j=1:M1
%     plot(xvector(1:N1),data1(:,1),'-','Parent',subplot1,'LineWidth',2);
% end
plot(0,0)
% for j=1:M
%     plot(xvector(1:N2),data2(:,1),'Parent',subplot1,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
% end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot2 = subplot(2,3,4,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot2,'on');
hold(subplot2,'all');
xmin = min(xvector);
xmax = max(xvector);
ymin = min(data2(:,4));
ymax = max(data2(:,4));
xlim([xmin xmax])
if round(ymin,3) < round(ymax,3)
    ylim([ymin ymax])
elseif round(ymin,3) == round(ymax,3)
    ylim([-ymin ymin])
end
% Create plot
% for j=1:M1
%     plot(xvector(1:N1),data1(:,4),'-','Parent',subplot2,'LineWidth',2,'Color','b');
% end
plot(0,0)
% for j=1:M
%     plot(xvector(1:N2),data2(:,4),'Parent',subplot2,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
% end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot3 = subplot(2,3,2,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot3,'on');
hold(subplot3,'all');
xmin = min(xvector);
xmax = max(xvector);
ymin = min(data2(:,2));
ymax = max(data2(:,2));
xlim([xmin xmax])
if round(ymin,3) < round(ymax,3)
    ylim([ymin ymax])
elseif round(ymin,3) == round(ymax,3)
    ylim([-ymin ymin])
end
% Create plot
% for j=1:M1
%     plot(xvector(1:N1),data1(:,2),'-','Parent',subplot3,'LineWidth',2);
% end
plot(0,0)
% for j=1:M
%     plot(xvector(1:N2),data2(:,2),'Parent',subplot3,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
% end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot4 = subplot(2,3,5,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot4,'on');
hold(subplot4,'all');
xmin = min(xvector);
xmax = max(xvector);
ymin = min(data2(:,5));
ymax = max(data2(:,5));
xlim([xmin xmax])
if round(ymin,3) < round(ymax,3)
    ylim([ymin ymax])
elseif round(ymin,3) == round(ymax,3)
    ylim([-ymin ymin])
end
% Create plot
% for j=1:M1
%     plot(xvector(1:N1),data1(:,5),'-','Parent',subplot4,'LineWidth',2,'Color','b');
% end
plot(0,0)
% for j=1:M
%     plot(xvector(1:N2),data2(:,5),'Parent',subplot4,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
% end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{5},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{5},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot5 = subplot(2,3,3,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot5,'on');
hold(subplot5,'all');
xmin = min(xvector);
xmax = max(xvector);
ymin = min(data2(:,3));
ymax = max(data2(:,3));
xlim([xmin xmax])
if round(ymin,3) < round(ymax,3)
    ylim([ymin ymax])
elseif round(ymin,3) == round(ymax,3)
    ylim([-ymin ymin])
end
% Create plot
% for j=1:M1
%     plot(xvector(1:N1),data1(:,3),'-','Parent',subplot5,'LineWidth',2);
% end
plot(0,0)
% for j=1:M
%     plot(xvector(1:N2),data2(:,3),'Parent',subplot5,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
% end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot6 = subplot(2,3,6,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot6,'on');
hold(subplot6,'all');
xmin = min(xvector);
xmax = max(xvector);
ymin = min(data2(:,6));
ymax = max(data2(:,6));
xlim([xmin xmax])
if round(ymin,3) < round(ymax,3)
    ylim([ymin ymax])
elseif round(ymin,3) == round(ymax,3)
    ylim([-ymin ymin])
end
% Create plot
% for j=1:M1
%     plot(xvector(1:N1),data1(:,6),'-','Parent',subplot6,'LineWidth',2,'Color','b');
% end
plot(0,0)
% for j=1:M
%     plot(xvector(1:N2),data2(:,6),'Parent',subplot6,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
% end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% if ~isempty(titel)
%     suptitle(titel);
% end



if ~isempty(data1)
    for j = 1 : M1
        x(1) = xvector(1);
        y11(1) = data1(1,1);
        y14(1) = data1(1,4);
        y12(1) = data1(1,2);
        y15(1) = data1(1,5);
        y13(1) = data1(1,3);
        y16(1) = data1(1,6);
        h11 = plot(x,y11,'-','Parent',subplot1,'LineWidth',2,'Color','b');
        h14 = plot(x,y14,'-','Parent',subplot2,'LineWidth',2,'Color','b');
        h12 = plot(x,y12,'-','Parent',subplot3,'LineWidth',2,'Color','b');
        h15 = plot(x,y15,'-','Parent',subplot4,'LineWidth',2,'Color','b');
        h13 = plot(x,y13,'-','Parent',subplot5,'LineWidth',2,'Color','b');
        h16 = plot(x,y16,'-','Parent',subplot6,'LineWidth',2,'Color','b');
    end
end

if ~isempty(data2)
    for j = 1 : M
        x(1) = xvector(1);
        y21(1) = data2(1,1);
        y24(1) = data2(1,4);
        y22(1) = data2(1,2);
        y25(1) = data2(1,5);
        y23(1) = data2(1,3);
        y26(1) = data2(1,6);
        h21 = plot(x,y21,'Parent',subplot1,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
        h22 = plot(x,y24,'Parent',subplot2,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
        h23 = plot(x,y22,'Parent',subplot3,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
        h24 = plot(x,y25,'Parent',subplot4,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
        h25 = plot(x,y23,'Parent',subplot5,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
        h26 = plot(x,y26,'Parent',subplot6,'LineWidth',1.5,'Color',[1.0 0.0 0.0],'LineStyle',linestyles{j});
    end
end

F(1) = getframe(gcf);

if ~isempty(data1) && isempty(data2)
    for j = 1 : M1
        for k = 2 : N1
            x(k) = xvector(k);
            y11(k) = data1(k,1);
            y14(k) = data1(k,4);
            y12(k) = data1(k,2);
            y15(k) = data1(k,5);
            y13(k) = data1(k,3);
            y16(k) = data1(k,6);
            set(h11,'XData',x,'YData',y11)
            set(h14,'XData',x,'YData',y14)
            set(h12,'XData',x,'YData',y12)
            set(h15,'XData',x,'YData',y15)
            set(h13,'XData',x,'YData',y13)
            set(h16,'XData',x,'YData',y16)
            drawnow
            F(k) = getframe(gcf);
            pause(0.01)
        end
    end
elseif ~isempty(data2) && isempty(data1)
    for j = 1 : M
        for k = 2 : N2
            x(k) = xvector(k);
            y21(k) = data2(k,1);
            y24(k) = data2(k,4);
            y22(k) = data2(k,2);
            y25(k) = data2(k,5);
            y23(k) = data2(k,3);
            y26(k) = data2(k,6);
            set(h21,'XData',x,'YData',y21)
            set(h24,'XData',x,'YData',y24)
            set(h22,'XData',x,'YData',y22)
            set(h25,'XData',x,'YData',y25)
            set(h23,'XData',x,'YData',y23)
            set(h26,'XData',x,'YData',y26)
            drawnow
            F(k) = getframe(gcf);
            pause(0.01)
        end
    end
elseif ~isempty(data2) && ~isempty(data1)
    for k = 2 : N
        x(k) = xvector(k);
        y11(k) = data1(k,1);
        y14(k) = data1(k,4);
        y12(k) = data1(k,2);
        y15(k) = data1(k,5);
        y13(k) = data1(k,3);
        y16(k) = data1(k,6);
        set(h11,'XData',x,'YData',y11)
        set(h14,'XData',x,'YData',y14)
        set(h12,'XData',x,'YData',y12)
        set(h15,'XData',x,'YData',y15)
        set(h13,'XData',x,'YData',y13)
        set(h16,'XData',x,'YData',y16)
        y21(k) = data2(k,1);
        y24(k) = data2(k,4);
        y22(k) = data2(k,2);
        y25(k) = data2(k,5);
        y23(k) = data2(k,3);
        y26(k) = data2(k,6);
        set(h21,'XData',x,'YData',y21)
        set(h24,'XData',x,'YData',y24)
        set(h22,'XData',x,'YData',y22)
        set(h25,'XData',x,'YData',y25)
        set(h23,'XData',x,'YData',y23)
        set(h26,'XData',x,'YData',y26)
        drawnow
        F(k) = getframe(gcf);
        pause(0.01)
    end
end

video = VideoWriter([titel,'_',parametrization,'_',descriptor_type]);
open(video)
writeVideo(video,F)
close(video)











