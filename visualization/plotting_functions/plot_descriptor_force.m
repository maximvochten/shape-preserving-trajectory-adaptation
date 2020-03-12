function plot_descriptor_force(data1,data2,titel,h,parametrization,descriptor_type,L)
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
if isempty(L)
    L=1;
end
N = max(N2,N1);

switch parametrization
    case 'timebased'
        switch descriptor_type
            case 'pose_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$F_x[N]$','$F_y[N]$','$F_z[N]$','$T_x[N.m]$','$T_y[N.m]$','$T_z[N.m]$'};
            case 'screw_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$F_x[N]$','$F_y[N]$','$F_z[N]$','$T_x[N.m]$','$T_y[N.m]$','$T_z[N.m]$'};
            case 'frenetserret'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$i_{t1}[N]$','$i_{t2}[\frac{rad}{s}]$','$i_{t3}[\frac{rad}{s}]$','$i_{r1}[N.m]$','$i_{r2}[\frac{rad}{s}]$','$i_{r3}[\frac{rad}{s}]$'};
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
                plotNames = {'$F_x[N]$','$F_y[N]$','$F_z[N]$','$T_x[N.m]$','$T_y[N.m]$','$T_z[N.m]$'};
            case 'screw_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$F_x[N]$','$F_y[N]$','$F_z[N]$','$T_x[N.m]$','$T_y[N.m]$','$T_z[N.m]$'};
            case 'frenetserret'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$i_{t1}[N]$','$i_{t2}[\frac{rad}{s}]$','$i_{t3}[\frac{rad}{s}]$','$i_{r1}[N.m]$','$i_{r2}[\frac{rad}{s}]$','$i_{r3}[\frac{rad}{s}]$'};
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
            case 'pose_twist'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$F_x[N]$','$F_y[N]$','$F_z[N]$','$T_x[N.m]$','$T_y[N.m]$','$T_z[N.m]$'};
            case 'screw_twist'
                xvector = linspace(0,h*N,N);
                label_x = '$t$[s]';
                plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'{$I_{r1}$[N]}','{$I_{r2}$[rad]}','{$I_{r3}$[rad]}','{$I_{t1}$[Nm]}','{$I_{t2}$[rad]}','{$I_{t3}$[rad]}'};
                %plotNames = {'{\boldmath $I_{r1}[rad]$}','{\boldmath $I_{r2}[-]$}','{\boldmath $I_{r3}[-]$}','{\boldmath $I_{t1}[m]$}','{\boldmath $I_{t2}[-]$}','{\boldmath $I_{t3}[-]$}'};
            case 'screw_axis'
                xvector = linspace(0,1,N);
                label_x = '$\tau$[-]';
                plotNames = {'$\Omega_1$[rad]','$\Omega_2$[rad]','$\Omega_3$[rad]','$V_1$[m]','$V_2$[m]','$V_3$[m]'};
        end
    case 'geometric_contour'
        switch descriptor_type
            case 'pose_twist'
                xvector = linspace(0,L,N);
                label_x = '$s$[m]';
                plotNames = {'$F_x[N]$','$F_y[N]$','$F_z[N]$','$T_x[N.m]$','$T_y[N.m]$','$T_z[N.m]$'};
%             case 'screw_twist'
%                 xvector = linspace(0,h*N,N);
%                 label_x = '$s$[m]';
%                 plotNames = {'$\omega_x[\frac{rad}{s}]$','$\omega_y[\frac{rad}{s}]$','$\omega_z[\frac{rad}{s}]$','$v_x[\frac{m}{s}]$','$v_y[\frac{m}{s}]$','$v_z[\frac{m}{s}]$'};
            case 'frenetserret'
                xvector = linspace(0,L,N);
                label_x = '$s$[m]';
                plotNames = {'{$I_{f1}$[N]}','{$I_{f2}$[rad]}','{$I_{f3}$[rad]}','{$I_{m1}$[Nm]}','{$I_{m2}$[rad]}','{$I_{m3}$[rad]}'};
                %plotNames = {'{\boldmath $I_{r1}[rad]$}','{\boldmath $I_{r2}[-]$}','{\boldmath $I_{r3}[-]$}','{\boldmath $I_{t1}[m]$}','{\boldmath $I_{t2}[-]$}','{\boldmath $I_{t3}[-]$}'};
%             case 'screw_axis'
%                 xvector = linspace(0,h*N,N);
%                 label_x = '$s$[m]';
%                 plotNames = {'$\Omega_1$[rad]','$\Omega_2$[rad]','$\Omega_3$[rad]','$V_1$[m]','$V_2$[m]','$V_3$[m]'};
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
                plotNames = {'$I_{f1}$[-]','$I_{f2}$[-]','$I_{f3}$[-]','$I_{m1}$[-]','$I_{m2}$[-]','$I_{m3}$[-]'};
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
set(gcf,'Units','normalized','OuterPosition',[0.5359    0.1819    0.4297    0.6806]);

% Create subplot
subplot1 = subplot(2,3,1,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot1,'on');
hold(subplot1,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,1),'-','Parent',subplot1,'LineWidth',2);
end
plot(0,0)
for j=1:M
    plot(xvector(1:N2),data2(:,1),'Parent',subplot1,'LineWidth',1.5,'Color',[0.0 1.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{1},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot2 = subplot(2,3,4,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot2,'on');
hold(subplot2,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,4),'-','Parent',subplot2,'LineWidth',2,'Color','b');
end
plot(0,0)
for j=1:M
    plot(xvector(1:N2),data2(:,4),'Parent',subplot2,'LineWidth',1.5,'Color',[0.0 1.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{4},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot3 = subplot(2,3,2,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot3,'on');
hold(subplot3,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,2),'-','Parent',subplot3,'LineWidth',2);
end
plot(0,0)
for j=1:M
    plot(xvector(1:N2),data2(:,2),'Parent',subplot3,'LineWidth',1.5,'Color',[0.0 1.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{2},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot4 = subplot(2,3,5,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot4,'on');
hold(subplot4,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,5),'-','Parent',subplot4,'LineWidth',2,'Color','b');
end
plot(0,0)
for j=1:M
    plot(xvector(1:N2),data2(:,5),'Parent',subplot4,'LineWidth',1.5,'Color',[0.0 1.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{5},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{5},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot5 = subplot(2,3,3,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot5,'on');
hold(subplot5,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,3),'-','Parent',subplot5,'LineWidth',2);
end
plot(0,0)
for j=1:M
    plot(xvector(1:N2),data2(:,3),'Parent',subplot5,'LineWidth',1.5,'Color',[0.0 1.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{3},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% Create subplot
subplot6 = subplot(2,3,6,'Parent',figure1,'YGrid','on','FontSize',axisfontsize);
box(subplot6,'on');
hold(subplot6,'all');
% Create plot
for j=1:M1
    plot(xvector(1:N1),data1(:,6),'-','Parent',subplot6,'LineWidth',2,'Color','b');
end
plot(0,0)
for j=1:M
    plot(xvector(1:N2),data2(:,6),'Parent',subplot6,'LineWidth',1.5,'Color',[0.0 1.0 0.0],'LineStyle',linestyles{j});
end
% Create xlabel
xlabel(label_x,'Interpreter','LaTex','FontSize',labelfontsize_x,'HorizontalAlignment', 'left');
% Create ylabel
%ylabel(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'Rotation',90,'HorizontalAlignment','right');
title(plotNames{6},'Interpreter','LaTex','FontSize',labelfontsize_y,'HorizontalAlignment', 'center');

% if ~isempty(titel)
%     suptitle(titel);
% end
